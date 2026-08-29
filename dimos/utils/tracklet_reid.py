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

"""Deciding that two tracklets are the same thing, after the tracker lost it.

A tracker associates across adjacent frames, so it breaks whenever continuity
breaks: an occlusion, a robot turning away, a detector miss. Every break makes a
new entity. Measured on one 8-minute office capture, a scene with perhaps thirty
distinct objects produced 1,329.

**Appearance alone gets this wrong, in the direction that matters.** Two chairs
of the same model are genuinely identical to a similarity model, so a threshold
high enough to merge one chair's fragments also merges two different chairs.
That failure is worse than fragmentation: fragments are honest, a wrong merge
invents an object that was never there. What separates identical objects is not
how they look but where and when they were -- a thing cannot be in two places at
once, and a thing that is not moving does not teleport.

So similarity is evidence and geometry is a veto. The constraints are checked
first and cheaply; the embedding comparison only runs on pairs that survive.

**Dependencies.** numpy, and nothing else. Embeddings arrive as arrays, already
computed by whatever model the caller chose; positions and times arrive as
plain numbers. Nothing here imports torch, a store, a message type, or a
module system, so the same code runs offline over a recording, live on a robot,
or in a notebook against a CSV.

That is why it sits in ``dimos/utils`` rather than beside the belief layer that
prompted it: nothing in it is about belief. Tracklets, positions and times are
what any tracker produces, and the half that knows about stores and crops lives
separately, in
:mod:`~dimos.experimental.memory_belief.reid_pass`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import itertools
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class Tracklet:
    """One run of sightings a tracker believed was a single thing.

    The unit of re-identification is the tracklet, not the sighting: a decision
    made from thirty views of an object is worth more than thirty decisions made
    from one view each, and the tracker has already done the easy grouping.
    """

    key: str
    """Whatever the caller calls this run. Returned verbatim in a merge."""

    t_start: float
    t_end: float

    #: One row per sighting that had a position. May be empty: a tracklet with
    #: no position can still be merged on appearance, it just cannot be vetoed
    #: by geometry, and :class:`ReidPolicy` decides whether that is allowed.
    positions: NDArray[np.float64] | None = None

    #: ``(N, D)``, L2-normalised, one row per sighting that was embedded. Empty
    #: or ``None`` means this tracklet cannot be compared at all.
    embeddings: NDArray[np.float64] | None = None

    label: str | None = None

    def centroid(self) -> NDArray[np.float64] | None:
        if self.positions is None or not len(self.positions):
            return None
        mean: NDArray[np.float64] = np.asarray(self.positions, float).mean(axis=0)
        return mean

    def overlaps(self, other: Tracklet) -> bool:
        return self.t_start <= other.t_end and other.t_start <= self.t_end


@dataclass(frozen=True, slots=True)
class ReidPolicy:
    """What counts as the same thing. Every field is a claim about the world.

    Defaults are a starting point, not a measurement: the similarity threshold
    in particular belongs to whichever embedding model the caller supplies, and
    a person-re-identification model's threshold is not a chair's.
    """

    #: Cosine similarity a surviving pair needs. **Belongs to the embedding
    #: model, not to this code.** Measured on one 8-minute office capture with
    #: CLIP: same-object pairs sat at median 0.967, different-object pairs at
    #: 0.862, and 0.925 separated them at 93.7% balanced accuracy. The 0.63 a
    #: person-re-identification system uses would have merged nearly every
    #: different-object pair. Re-measure when the model or the scene changes;
    #: the halves of one tracklet give same-object pairs and tracklets that
    #: overlap in time give different-object pairs, so it needs no labelling.
    similarity: float = 0.925

    #: Metres a thing may be found from where it was and still be the same
    #: thing. This is the veto that keeps two identical chairs apart. Checked
    #: between two tracklets; see ``max_spread_m`` for the group it lands in.
    max_move_m: float = 2.0

    #: Metres a whole merged group may span. A pairwise check cannot see this:
    #: a-b within range and b-c within range puts a and c twice as far apart,
    #: and a room of identical chairs chains into one entity spanning the room.
    #: Measured on an 8-minute office capture, an unbounded closure produced a
    #: "chair" of 16,730 sightings spread over 4.28 m, where a real entity's
    #: sightings spread by centimetres. Default is twice ``max_move_m``: a group
    #: is allowed to be looser than any one link, but not unboundedly so.
    max_spread_m: float = 4.0

    #: Views a tracklet needs before it may be merged at all. One view of a
    #: chair's back and one of its front are not similar, and one view either
    #: side of a merge decision is not evidence.
    min_views: int = 5

    #: Similarities averaged when comparing two tracklets. Comparing means
    #: washes out the informative views; comparing maxima trusts a single lucky
    #: pair. The mean of the best few is neither.
    top_k: int = 20

    #: Whether a tracklet with no position may be merged. False refuses rather
    #: than guessing -- a sighting with no depth could be anywhere, so nothing
    #: vetoes it and appearance decides alone.
    merge_without_position: bool = False

    #: Whether two tracklets must agree on their label. Off by default: the
    #: labels flicker between synonyms (chair / stool / electric chair) far more
    #: than the appearance does.
    require_same_label: bool = False


@dataclass(frozen=True, slots=True)
class Merge:
    """A claim that two tracklets are one thing, with what supports it."""

    keys: tuple[str, ...]
    similarity: float
    distance_m: float | None
    gap_s: float
    """Time between one tracklet ending and the next beginning. A long gap is
    not disqualifying -- it is exactly the case a tracker cannot handle -- but
    it is what a reader needs to judge the claim."""


@dataclass
class Report:
    """What a pass over the tracklets did, including what it refused.

    The rejection counts are the point, not decoration: a run that merges
    nothing because the constraints held and a run that merges nothing because
    no tracklet had an embedding look identical from the outside, and only this
    tells them apart.
    """

    merges: list[Merge] = field(default_factory=list)
    rejected: dict[str, int] = field(default_factory=dict)

    def note(self, reason: str) -> None:
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def summary(self) -> str:
        parts = ", ".join(f"{k}={v}" for k, v in sorted(self.rejected.items()))
        return f"{len(self.merges)} merges; rejected: {parts or 'none'}"


def group_similarity(a: NDArray[np.float64], b: NDArray[np.float64], *, top_k: int) -> float:
    """Mean of the top-k cosine similarities between two sets of views.

    Both sides are assumed L2-normalised, so the product is the cosine. The
    aggregation is the whole decision in one line, and the reason it is neither
    ``max`` nor ``mean`` is in :class:`ReidPolicy`.
    """
    sims = np.asarray(a, float) @ np.asarray(b, float).T
    flat = sims.ravel()
    k = min(top_k, flat.size)
    return float(np.mean(np.partition(flat, -k)[-k:]))


def _veto(a: Tracklet, b: Tracklet, policy: ReidPolicy) -> tuple[str | None, float | None]:
    """The cheap checks, in order of how conclusive they are."""
    if a.overlaps(b):
        # Seen at the same moment in different tracks: whatever else is true,
        # these are two things. No similarity score can outweigh it.
        return "time_overlap", None
    if policy.require_same_label and a.label != b.label:
        return "label_mismatch", None

    ca, cb = a.centroid(), b.centroid()
    if ca is None or cb is None:
        if not policy.merge_without_position:
            return "no_position", None
        return None, None

    distance = float(np.linalg.norm(ca - cb))
    if distance > policy.max_move_m:
        return "too_far", distance
    return None, distance


def find_merges(
    tracklets: Iterable[Tracklet], policy: ReidPolicy | None = None
) -> tuple[list[Merge], Report]:
    """Every pair of tracklets that survives the vetoes and clears the threshold.

    Pairs, not clusters. Transitive closure is the caller's to take, because
    "a matches b, b matches c" is a claim about a and c that this function did
    not check -- and with identical objects it is often false.
    """
    policy = policy or ReidPolicy()
    items = [t for t in tracklets]
    report = Report()

    for a, b in itertools.combinations(items, 2):
        reason, distance = _veto(a, b, policy)
        if reason is not None:
            report.note(reason)
            continue
        if a.embeddings is None or b.embeddings is None:
            report.note("no_embeddings")
            continue
        if len(a.embeddings) < policy.min_views or len(b.embeddings) < policy.min_views:
            report.note("too_few_views")
            continue

        score = group_similarity(a.embeddings, b.embeddings, top_k=policy.top_k)
        if score < policy.similarity:
            report.note("below_threshold")
            continue

        earlier, later = (a, b) if a.t_end <= b.t_start else (b, a)
        report.merges.append(
            Merge(
                keys=(a.key, b.key),
                similarity=score,
                distance_m=distance,
                gap_s=max(0.0, later.t_start - earlier.t_end),
            )
        )
    return report.merges, report


def connected(
    merges: Iterable[Merge],
    tracklets: Iterable[Tracklet] | None = None,
    policy: ReidPolicy | None = None,
) -> Iterator[frozenset[str]]:
    """Take the transitive closure of pairwise merges into groups.

    A different and weaker claim than the pairs it is built from. Chaining a-b
    and b-c asserts a-c on evidence nobody checked, and one wrong pair collapses
    two objects into one group.

    Passing ``tracklets`` checks each closed group against
    ``policy.max_spread_m`` and drops the ones that span more than a real object
    could -- the only place that error is visible, because every pair inside
    such a group passed its own distance check. Without them the closure is
    returned unchecked, which is what a caller wanting the raw chaining gets.
    """
    parent: dict[str, str] = {}

    def find(x: str) -> str:
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for merge in merges:
        roots = [find(k) for k in merge.keys]
        for other in roots[1:]:
            parent[other] = roots[0]

    groups: dict[str, set[str]] = {}
    for key in list(parent):
        groups.setdefault(find(key), set()).add(key)

    if tracklets is None:
        for members in groups.values():
            yield frozenset(members)
        return

    policy = policy or ReidPolicy()
    centroids = {t.key: t.centroid() for t in tracklets}
    for members in groups.values():
        points = [c for c in (centroids.get(k) for k in members) if c is not None]
        if len(points) > 1:
            stacked = np.stack(points)
            spread = float(np.linalg.norm(stacked.max(axis=0) - stacked.min(axis=0)))
            if spread > policy.max_spread_m:
                continue
        yield frozenset(members)
