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

**What was measured, and what it ruled out**, is in
``docs/capabilities/perception/tracklet_reid.md``. The short version: pairwise
association scores 99% recall at 100% precision, and every way of chaining those
pairs into groups -- transitive closure offline, id assignment online -- lands
between 24% and 39% precision. So :func:`find_merges` returns pairs and
:func:`connected` is something a caller opts into knowing that.

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
    #: restores a fixed radius -- nothing may move at all.
    max_speed_mps: float = 1.5

    #: Ceiling on how far the speed allowance may reach, whatever the gap. A
    #: budget that grows without limit stops being a constraint: on a 16 m
    #: office, a six-second gap already reaches 11 m, and a minute reaches past
    #: every wall. Beyond the space the robot works in, "it could have got
    #: there" is true of everything and vetoes nothing.
    max_reach_m: float = 8.0

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
    reachable = min(
        policy.max_move_m + policy.max_speed_mps * gap_between(a, b), policy.max_reach_m
    )
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


@dataclass(frozen=True, slots=True)
class OnlinePolicy:
    """What an online associator needs beyond :class:`ReidPolicy`.

    Batch re-identification sees every tracklet before deciding anything. A
    robot does not: it has to answer "which thing is this" while the thing is
    still being seen, and revise nothing it has already said out loud. The two
    fields here are what that costs.
    """

    #: Views kept per track. Unbounded, a robot that runs for a day holds every
    #: crop it ever embedded; the comparison reads the best few anyway, so what
    #: is dropped is the middle of a distribution rather than its shape.
    max_views: int = 200

    #: Tracks kept before the oldest is forgotten. A long shift produces
    #: thousands, and a track last seen an hour ago is not a candidate for
    #: something visible now -- the speed veto would refuse it regardless.
    max_tracks: int = 500


class OnlineAssociator:
    """Assign long-term ids to tracks as their sightings arrive.

    The same decisions as :func:`find_merges`, made one observation at a time.
    Nothing here re-implements the policy: state accumulates into
    :class:`Tracklet` views and the same ``_veto`` and :func:`group_similarity`
    run against them, so a change to what counts as the same thing takes effect
    in both halves at once.

    **Deciding late is allowed; deciding twice is not.** :meth:`resolve` returns
    ``None`` while a track has too little evidence to be matched, rather than
    inventing an id it would have to take back. A caller that needs an answer
    now treats ``None`` as "a new thing, provisionally"; a caller writing
    identity claims simply waits, because a claim it never made needs no
    retraction.

    Memory is bounded on both axes -- views per track and tracks retained -- so
    this runs for a shift rather than for a recording.
    """

    def __init__(self, policy: ReidPolicy | None = None, online: OnlinePolicy | None = None):
        self.policy = policy or ReidPolicy()
        self.online = online or OnlinePolicy()
        self._views: dict[str, list[NDArray[np.float64]]] = {}
        self._points: dict[str, list[tuple[float, float, float]]] = {}
        self._span: dict[str, tuple[float, float]] = {}
        self._label: dict[str, str | None] = {}
        self._assigned: dict[str, str] = {}
        self._excluded: dict[str, set[str]] = {}
        self._order: list[str] = []
        self._counter = 0

    def observe(
        self,
        track: str,
        *,
        embedding: NDArray[np.float64] | None = None,
        position: tuple[float, float, float] | None = None,
        ts: float,
        label: str | None = None,
    ) -> None:
        """Add one sighting's evidence to a track."""
        if track not in self._span:
            self._order.append(track)
            self._forget_old()
        first, last = self._span.get(track, (ts, ts))
        self._span[track] = (min(first, ts), max(last, ts))
        self._label.setdefault(track, label)
        if embedding is not None:
            vector = np.asarray(embedding, float).ravel()
            norm = float(np.linalg.norm(vector))
            if norm:
                views = self._views.setdefault(track, [])
                views.append(vector / norm)
                if len(views) > self.online.max_views:
                    # Drop from the middle: the first views establish the track
                    # and the newest reflect it now, so neither end is the part
                    # to lose.
                    del views[len(views) // 2]
        if position is not None:
            x, y, z = (float(v) for v in position)
            self._points.setdefault(track, []).append((x, y, z))

    def co_occurring(self, tracks: Iterable[str]) -> None:
        """Record that these tracks were visible at one instant.

        Two things seen at once are two things, whatever they look like. The
        batch path derives this from overlapping spans; live, a caller that
        knows which tracks shared a frame can say so directly and get the veto
        before either track is long enough to compare.
        """
        keys = list(tracks)
        for a in keys:
            self._excluded.setdefault(a, set()).update(k for k in keys if k != a)

    def _tracklet(self, track: str) -> Tracklet:
        first, last = self._span[track]
        points = self._points.get(track)
        views = self._views.get(track)
        return Tracklet(
            key=track,
            t_start=first,
            t_end=last,
            positions=np.asarray(points, float) if points else None,
            embeddings=np.stack(views) if views else None,
            label=self._label.get(track),
        )

    def resolve(self, track: str) -> str | None:
        """The long-term id for ``track``, or None while the evidence is thin.

        Once a track has an id it keeps it: revising an identity a caller has
        already acted on is worse than having been slow to give one.
        """
        if track in self._assigned:
            return self._assigned[track]
        views = self._views.get(track)
        if views is None or len(views) < self.policy.min_views:
            return None

        query = self._tracklet(track)
        if query.embeddings is None:
            return None
        best_key, best_score = None, self.policy.similarity
        centroid = query.centroid()
        for other in self._order:
            if other == track or other not in self._assigned:
                continue
            if other in self._excluded.get(track, ()):
                continue
            candidate = self._tracklet(other)
            if candidate.embeddings is None or len(candidate.embeddings) < self.policy.min_views:
                continue
            if _veto(query, candidate, self.policy)[0] is not None:
                continue
            # What the group would span once this track joined it. The pairwise
            # distance cannot see this and the batch path checks it in
            # `connected`; without it here, 388 pieces of one capture collapsed
            # into a single id, each link legal and the chain across the room.
            if centroid is not None and self._spread_with(self._assigned[other], centroid) > (
                self.policy.max_spread_m
            ):
                continue
            score = group_similarity(
                query.embeddings, candidate.embeddings, top_frac=self.policy.top_frac
            )
            if score >= best_score:
                best_key, best_score = other, score

        # A single best match, not every match above the threshold: chaining is
        # what put a room of identical chairs into one entity, and online there
        # is no second pass to catch it.
        self._counter += 1
        assigned = self._assigned[best_key] if best_key else f"entity-{self._counter}"
        self._assigned[track] = assigned
        return assigned

    def _spread_with(self, entity: str, point: NDArray[np.float64]) -> float:
        """How far a group would span with ``point`` added to it."""
        points = [point]
        for track, assigned in self._assigned.items():
            if assigned != entity:
                continue
            centre = self._tracklet(track).centroid()
            if centre is not None:
                points.append(centre)
        if len(points) < 2:
            return 0.0
        stacked = np.stack(points)
        return float(np.linalg.norm(stacked.max(axis=0) - stacked.min(axis=0)))

    def _forget_old(self) -> None:
        while len(self._order) > self.online.max_tracks:
            gone = self._order.pop(0)
            for store in (self._views, self._points, self._span, self._label, self._excluded):
                store.pop(gone, None)
