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
from typing import TYPE_CHECKING, Any

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

    #: Metres a thing may be found from where it was even with no time to move
    #: in. Absorbs the placement error, not real motion. This is the veto that
    #: keeps two identical chairs apart; see ``max_spread_m`` for the group a
    #: chain of them lands in.
    max_move_m: float = 2.0

    #: Metres per second a thing may travel while unobserved. Without this the
    #: distance check treats a chair and a walking person alike, and a person is
    #: exactly what re-identification is most often for: measured on an
    #: 8-minute capture, 30 tracked objects moved further than ``max_move_m``
    #: and could never be rejoined however obviously they matched. Zero
    #: restores the old behaviour -- nothing may move at all.
    max_speed_mps: float = 1.5

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

    #: Fraction of the pairwise similarities averaged when comparing two
    #: tracklets. A fraction rather than a count because a count degenerates:
    #: two five-view tracklets make 25 pairs, and taking the best 20 of them is
    #: the mean -- which is what the aggregation exists to avoid. Comparing
    #: means washes out the informative views; comparing maxima trusts a single
    #: lucky pair. The best fifth is neither, at any tracklet size.
    top_frac: float = 0.2

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


def group_similarity(
    a: NDArray[np.float64], b: NDArray[np.float64], *, top_frac: float
) -> float:
    """Mean of the best ``top_frac`` of cosine similarities between two view sets.

    Both sides are assumed L2-normalised, so the product is the cosine. The
    aggregation is the whole decision in one line, and the reason it is neither
    ``max`` nor ``mean`` is in :class:`ReidPolicy`.
    """
    sims = np.asarray(a, float) @ np.asarray(b, float).T
    flat = sims.ravel()
    k = max(1, min(flat.size, round(top_frac * flat.size)))
    return float(np.mean(np.partition(flat, -k)[-k:]))


def gap_between(a: Tracklet, b: Tracklet) -> float:
    """Seconds between one tracklet ending and the other beginning."""
    earlier, later = (a, b) if a.t_end <= b.t_start else (b, a)
    return max(0.0, later.t_start - earlier.t_end)


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
    # How far it could have got: placement error, plus travel over the time
    # nobody was watching. A fixed radius would refuse every person who walked
    # away and came back, which is the case this exists for.
    reachable = policy.max_move_m + policy.max_speed_mps * gap_between(a, b)
    if distance > reachable:
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

        score = group_similarity(a.embeddings, b.embeddings, top_frac=policy.top_frac)
        if score < policy.similarity:
            report.note("below_threshold")
            continue

        report.merges.append(
            Merge(
                keys=(a.key, b.key),
                similarity=score,
                distance_m=distance,
                gap_s=gap_between(a, b),
            )
        )
    return report.merges, report


def connected(
    merges: Iterable[Merge],
    tracklets: Iterable[Tracklet] | None = None,
    policy: ReidPolicy | None = None,
) -> Iterator[frozenset[str]]:
    """Close pairwise merges into groups, strongest link first.

    A different and weaker claim than the pairs it is built from: chaining a-b
    and b-c asserts a-c on evidence nobody checked, and one wrong pair can
    collapse two objects into one group.

    Given ``tracklets``, a link is taken only if the group it would create still
    spans less than ``policy.max_spread_m``. Refusing the *link* rather than the
    finished group is what keeps one bad pair from costing every good merge
    around it -- an earlier version dropped whole groups and lost several
    hundred correct merges to a single bad chain. Taking links in descending
    similarity means the ones refused are the weakest evidence in the group,
    not whichever happened to be considered last.

    Without ``tracklets`` the closure is unchecked, which is what a caller
    wanting the raw chaining gets.
    """
    policy = policy or ReidPolicy()
    centroids: dict[str, Any] = {}
    if tracklets is not None:
        centroids = {t.key: t.centroid() for t in tracklets}

    parent: dict[str, str] = {}
    members: dict[str, set[str]] = {}

    def find(x: str) -> str:
        parent.setdefault(x, x)
        members.setdefault(x, {x})
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def spread(keys: set[str]) -> float:
        points = [c for c in (centroids.get(k) for k in keys) if c is not None]
        if len(points) < 2:
            return 0.0
        stacked = np.stack(points)
        return float(np.linalg.norm(stacked.max(axis=0) - stacked.min(axis=0)))

    for merge in sorted(merges, key=lambda m: -m.similarity):
        roots = {find(k) for k in merge.keys}
        if len(roots) < 2:
            continue
        joined: set[str] = set().union(*(members[r] for r in roots))
        if centroids and spread(joined) > policy.max_spread_m:
            continue
        keep = roots.pop()
        for other in roots:
            parent[other] = keep
        members[keep] = joined

    for root in {find(k) for k in parent}:
        yield frozenset(members[root])
