# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Running re-identification over a store, and writing what it decided.

:mod:`~dimos.utils.tracklet_reid` is deliberately pure -- numpy and nothing
else -- so it knows nothing about stores, frames, or embedding models, and
lives in ``utils`` because nothing in it is about belief. This is the half that
does: it reads sightings, crops what they saw, embeds the
crops with whatever model the caller hands over, and turns the merges back into
:class:`~dimos.experimental.memory_belief.identity.IdentityClaim` records.

**Merges are appended, never applied in place.** A reid claim covers the same
sightings the tracker's claims did and names one entity for all of them; because
``resolve_identity`` lets later claims win, folding the stream again groups them
together. The tracker's original claims stay in the stream, so a merge this
decided can be inspected, retracted, or simply ignored by a reader that trusts
only ``basis="tracker:..."``.

The embedding model is a callable, injected. That keeps torch out of every
import path that does not embed anything, and lets the choice of model be a
deployment decision rather than one frozen here.
"""

from __future__ import annotations

import collections
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.experimental.memory_belief.identity import IdentityClaim
from dimos.experimental.memory_belief.types import SCHEMA_VERSION
from dimos.experimental.memory_belief.write import belief_stream
from dimos.utils.tracklet_reid import ReidPolicy, Tracklet, connected, find_merges

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

#: Pixels a crop must span on both sides to be worth embedding. A twelve-pixel
#: sliver of a chair embeds to something, and that something is noise the
#: similarity threshold cannot tell from a match.
MIN_CROP_PX = 32

#: Crops embedded per tracklet. The comparison averages its best few, so more
#: views past this buy little and cost a forward pass each.
VIEWS_PER_TRACK = 24


def _crop(store: Any, record: Any, cache: dict[float, Any]) -> Any:
    """The image region one sighting was made from, or None.

    Frames are cached by timestamp and the cache is bounded: a frame is
    megabytes, and a long recording has thousands of them.
    """
    evidence = record.evidence[0] if record.evidence else None
    if evidence is None or not record.bbox:
        return None
    if evidence.ts not in cache:
        if len(cache) > 256:
            cache.pop(next(iter(cache)))
        found = store.stream(evidence.stream).at(evidence.ts, tolerance=0.05).limit(1).to_list()
        cache[evidence.ts] = found[0].data.data if found else None
    image = cache.get(evidence.ts)
    if image is None:
        return None
    x1, y1, x2, y2 = (int(v) for v in record.bbox)
    crop = image[max(0, y1) : y2, max(0, x1) : x2]
    if crop.shape[0] < MIN_CROP_PX or crop.shape[1] < MIN_CROP_PX:
        return None
    return crop


def tracklets_from(
    store: Any,
    embed: Callable[[Any], Any],
    *,
    views_per_track: int = VIEWS_PER_TRACK,
    report: Callable[[str], None] | None = None,
) -> list[Tracklet]:
    """Group a store's sightings by tracker id and embed a sample of each.

    ``embed`` takes one crop -- an ``(H, W, 3)`` array -- and returns a vector.
    Whatever it returns is L2-normalised here, because the comparison is a dot
    product and a caller that forgets to normalise gets silently wrong scores
    rather than an error.
    """
    say = report or (lambda _m: None)
    by_track: dict[str, list[Any]] = collections.defaultdict(list)
    for observation in belief_stream(store):
        record = observation.data
        if record.identity_basis and record.bbox and record.evidence:
            by_track[record.identity_basis.split(":")[-1]].append(record)
    say(f"reid: {len(by_track)} tracks to embed")

    cache: dict[float, Any] = {}
    tracklets: list[Tracklet] = []
    for track_id, records in by_track.items():
        step = max(1, len(records) // views_per_track)
        sampled = records[::step][:views_per_track]
        vectors, positions = [], []
        for record in sampled:
            crop = _crop(store, record, cache)
            if crop is None:
                continue
            raw = getattr(embed(crop), "vector", None)
            if raw is None:
                continue
            # A model on a GPU hands back a device tensor, which numpy refuses
            # to convert. Detaching here rather than asking every caller to.
            if hasattr(raw, "detach"):
                raw = raw.detach().cpu().numpy()
            vector = np.asarray(raw, float).ravel()
            norm = float(np.linalg.norm(vector))
            if not norm:
                continue
            vectors.append(vector / norm)
            if record.target_pose is not None:
                positions.append(record.target_pose)
        if not vectors:
            continue
        tracklets.append(
            Tracklet(
                key=track_id,
                t_start=min(r.valid_ts for r in records),
                t_end=max(r.valid_ts for r in records),
                positions=np.asarray(positions, float) if positions else None,
                embeddings=np.stack(vectors),
                label=sampled[0].label,
            )
        )
    say(f"reid: {len(tracklets)} tracklets carry embeddings")
    return tracklets


def merge_claims(
    store: Any,
    tracklets: list[Tracklet],
    *,
    session: str,
    policy: ReidPolicy | None = None,
    basis: str = "reid:appearance",
    report: Callable[[str], None] | None = None,
) -> Iterator[IdentityClaim]:
    """One claim per merged group, naming every sighting in it as one entity.

    Emitted after the tracker's own claims and read by ``resolve_identity``,
    where a later claim wins -- so folding the stream again puts the group's
    sightings under one entity without the tracker's claims being removed.

    The confidence carried is the weakest similarity in the group, not the
    strongest: a group chained through a marginal pair is only as good as that
    pair, and reporting its best link would hide exactly the merge a reader
    should doubt.
    """
    say = report or (lambda _m: None)
    policy = policy or ReidPolicy()
    merges, run = find_merges(tracklets, policy)
    say(f"reid: {run.summary()}")

    weakest: dict[frozenset[str], float] = {}
    # Tracklets and policy passed so a chained group that spans further than a
    # real object can be dropped -- every pair inside it passed its own check.
    groups = list(connected(merges, tracklets, policy))
    for group in groups:
        scores = [m.similarity for m in merges if set(m.keys) <= group]
        weakest[group] = min(scores) if scores else 0.0

    refs_by_track: dict[str, list[str]] = collections.defaultdict(list)
    for observation in belief_stream(store):
        record = observation.data
        if record.identity_basis:
            refs_by_track[record.identity_basis.split(":")[-1]].append(record.target_ref)

    ends = {t.key: t.t_end for t in tracklets}
    for index, group in enumerate(groups):
        refs = tuple(ref for key in sorted(group) for ref in refs_by_track.get(key, ()))
        if not refs:
            continue
        yield IdentityClaim(
            schema_version=SCHEMA_VERSION,
            claim_id=f"{session}:reid:{index}",
            entity_id=f"{session}:reid:{index}",
            target_refs=refs,
            basis=basis,
            confidence=round(weakest[group], 3),
            valid_ts=max(ends.get(key, 0.0) for key in group),
        )
