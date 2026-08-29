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

from __future__ import annotations

import numpy as np

from dimos.utils.tracklet_reid import (
    Merge,
    ReidPolicy,
    Tracklet,
    connected,
    find_merges,
    group_similarity,
)


def views(direction: tuple[float, float], n: int = 10, jitter: float = 0.02) -> np.ndarray:
    """``n`` unit vectors clustered around one direction in 2-D embedding space."""
    rng = np.random.default_rng(0)
    base = np.asarray(direction, float)
    raw = base + rng.normal(0.0, jitter, size=(n, 2))
    return raw / np.linalg.norm(raw, axis=1, keepdims=True)


def tracklet(key, t0, t1, xy=(0.0, 0.0), look=(1.0, 0.0), n=10, label="chair") -> Tracklet:
    return Tracklet(
        key=key,
        t_start=t0,
        t_end=t1,
        positions=np.array([[xy[0], xy[1], 0.0]] * n, float),
        embeddings=views(look, n=n),
        label=label,
    )


class TestGeometryVetoesAppearance:
    """Two identical objects are the case appearance cannot decide.

    A similarity model given two chairs of one model returns a near-perfect
    score, correctly: they look the same. Merging them would invent an object.
    Only where and when they were can separate them, so those checks run first
    and no score overrides them.
    """

    def test_two_things_seen_at_once_never_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 5.0, 15.0, xy=(0.2, 0.0))  # identical look, overlapping time

        merges, report = find_merges([a, b])

        assert merges == []
        assert report.rejected == {"time_overlap": 1}

    def test_identical_things_far_apart_never_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(50.0, 0.0))

        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0))

        assert merges == []
        assert report.rejected == {"too_far": 1}

    def test_the_same_thing_seen_again_nearby_merges(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(0.5, 0.0))

        merges, _ = find_merges([a, b])

        assert len(merges) == 1
        assert merges[0].keys == ("a", "b")
        assert merges[0].gap_s == 10.0
        assert merges[0].distance_m is not None


class TestAThingIsAllowedToHaveMoved:
    """A fixed radius refuses every person who walked away and came back.

    Measured on an 8-minute capture, 30 tracked objects moved further than
    ``max_move_m`` -- and a walking person is the case re-identification is
    most often wanted for.
    """

    def test_a_walk_within_reach_of_the_gap_merges(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), label="person")
        b = tracklet("b", 40.0, 50.0, xy=(20.0, 0.0), label="person")

        # 30 s unobserved at 1.5 m/s reaches 45 m; 20 m is inside it.
        merges, _ = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert len(merges) == 1
        assert merges[0].gap_s == 30.0

    def test_a_walk_beyond_reach_of_the_gap_does_not(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), label="person")
        b = tracklet("b", 12.0, 22.0, xy=(20.0, 0.0), label="person")

        # 2 s reaches 5 m. Twenty metres in two seconds is not one person.
        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=1.5))

        assert merges == []
        assert report.rejected == {"too_far": 1}

    def test_zero_speed_restores_a_fixed_radius(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0))
        b = tracklet("b", 40.0, 50.0, xy=(20.0, 0.0))

        merges, report = find_merges([a, b], ReidPolicy(max_move_m=2.0, max_speed_mps=0.0))

        assert merges == []
        assert report.rejected == {"too_far": 1}


class TestAppearanceStillHasToAgree:
    def test_a_different_looking_thing_nearby_does_not_merge(self):
        a = tracklet("a", 0.0, 10.0, xy=(0.0, 0.0), look=(1.0, 0.0))
        b = tracklet("b", 20.0, 30.0, xy=(0.5, 0.0), look=(0.0, 1.0))

        merges, report = find_merges([a, b])

        assert merges == []
        assert report.rejected == {"below_threshold": 1}


class TestEvidenceHasToBeEnough:
    def test_a_tracklet_with_too_few_views_is_refused_not_guessed(self):
        a = tracklet("a", 0.0, 10.0, n=2)
        b = tracklet("b", 20.0, 30.0, n=20)

        merges, report = find_merges([a, b], ReidPolicy(min_views=5))

        assert merges == []
        assert report.rejected == {"too_few_views": 1}

    def test_a_tracklet_with_no_position_is_refused_by_default(self):
        a = Tracklet("a", 0.0, 10.0, positions=None, embeddings=views((1.0, 0.0)))
        b = tracklet("b", 20.0, 30.0)

        merges, report = find_merges([a, b])

        # Nothing vetoes a sighting that could be anywhere, so appearance would
        # decide alone -- which is the case this layer refuses by default.
        assert merges == []
        assert report.rejected == {"no_position": 1}

    def test_positionless_merging_is_available_when_asked_for(self):
        a = Tracklet("a", 0.0, 10.0, positions=None, embeddings=views((1.0, 0.0)))
        b = tracklet("b", 20.0, 30.0)

        merges, _ = find_merges([a, b], ReidPolicy(merge_without_position=True))

        assert len(merges) == 1
        assert merges[0].distance_m is None


class TestTheReportSaysWhyNothingMerged:
    """Zero merges from bad input and zero merges from good input look alike."""

    def test_every_rejection_is_counted_by_reason(self):
        far = tracklet("far", 20.0, 30.0, xy=(50.0, 0.0))
        overlapping = tracklet("over", 5.0, 15.0)
        base = tracklet("base", 0.0, 10.0)

        _, report = find_merges([base, overlapping, far])

        assert sum(report.rejected.values()) == 3
        assert "time_overlap" in report.summary()


class TestAGroupIsCheckedAsAWhole:
    """Every pair inside a chain can pass and the chain still be wrong.

    a-b within range and b-c within range puts a and c twice as far apart. On
    real data an unbounded closure produced a "chair" of 16,730 sightings spread
    over 4.28 m, where a real entity's sightings spread by centimetres.
    """

    def test_the_link_that_overruns_is_refused_not_the_whole_group(self):
        chain = [
            tracklet("a", 0.0, 10.0, xy=(0.0, 0.0)),
            tracklet("b", 20.0, 30.0, xy=(1.8, 0.0)),
            tracklet("c", 40.0, 50.0, xy=(3.6, 0.0)),
        ]
        merges = [Merge(("a", "b"), 0.99, 1.8, 10.0), Merge(("b", "c"), 0.90, 1.8, 10.0)]

        # Unchecked, the chain closes into one group spanning 3.6 m.
        assert list(connected(merges)) == [frozenset({"a", "b", "c"})]

        # Checked, the stronger link is taken and only the one that would
        # overrun is refused. Dropping the whole group instead cost several
        # hundred correct merges on real data.
        groups = sorted(connected(merges, chain, ReidPolicy(max_spread_m=2.0)), key=sorted)
        assert groups == [frozenset({"a", "b"}), frozenset({"c"})]

    def test_a_tight_chain_survives(self):
        chain = [
            tracklet("a", 0.0, 10.0, xy=(0.0, 0.0)),
            tracklet("b", 20.0, 30.0, xy=(0.3, 0.0)),
            tracklet("c", 40.0, 50.0, xy=(0.6, 0.0)),
        ]
        merges = [Merge(("a", "b"), 0.95, 0.3, 10.0), Merge(("b", "c"), 0.95, 0.3, 10.0)]

        groups = list(connected(merges, chain, ReidPolicy(max_spread_m=2.0)))

        assert groups == [frozenset({"a", "b", "c"})]


class TestGroupingIsAWeakerClaim:
    def test_chained_merges_close_into_one_group(self):
        groups = list(
            connected([Merge(("a", "b"), 0.9, 1.0, 5.0), Merge(("b", "c"), 0.9, 1.0, 5.0)])
        )

        assert groups == [frozenset({"a", "b", "c"})]

    def test_unrelated_merges_stay_separate(self):
        groups = list(
            connected([Merge(("a", "b"), 0.9, 1.0, 5.0), Merge(("c", "d"), 0.9, 1.0, 5.0)])
        )

        assert sorted(groups, key=sorted) == [frozenset({"a", "b"}), frozenset({"c", "d"})]


class TestSimilarityAggregation:
    def test_the_best_fraction_ignores_the_worst_pairs(self):
        a = np.array([[1.0, 0.0], [0.0, 1.0]])
        b = np.array([[1.0, 0.0], [0.0, 1.0]])

        # Pairs are 1, 0, 0, 1. The best half average to 1; all four average
        # to 0.5, which would hide a real match behind unhelpful views.
        assert group_similarity(a, b, top_frac=0.5) == 1.0
        assert group_similarity(a, b, top_frac=1.0) == 0.5

    def test_the_fraction_does_not_degenerate_on_small_tracklets(self):
        """A count would. Five views each is 25 pairs, and the best 20 is a mean."""
        a = np.eye(5)
        b = np.eye(5)

        # Orthogonal views: every pair scores 0 except the five matching ones.
        assert group_similarity(a, b, top_frac=0.2) == 1.0
        assert group_similarity(a, b, top_frac=0.8) < 0.3
