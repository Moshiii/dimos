# Copyright 2025-2026 Dimensional Inc.
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

import pytest

from dimos.experimental.memory_belief.api import QueryError, execute
from dimos.experimental.memory_belief.entity import ENTITY_STREAM_NAME, Entity, append_entity
from dimos.experimental.memory_belief.types import (
    SCHEMA_VERSION,
    BeliefObservation,
    Confidence,
    EvidenceRef,
)
from dimos.experimental.memory_belief.write import append_belief, belief_stream, derived_stream
from dimos.memory.store.sqlite import SqliteStore

T0 = 1000.0


def entity(eid, label, ts, *, position=(1.0, 1.0, 0.0), support=10, place="cell(0,0)", coh=1.0):
    return Entity(
        schema_version="1",
        entity_id=eid,
        label=label,
        label_agreement=1.0,
        support=support,
        first_seen_ts=ts,
        last_seen_ts=ts,
        position=position,
        extent=(0.4, 0.4, 0.4),
        dispersion_m=0.1,
        coherence=coh,
        displacement_m=0.0,
        motion="static",
        place_ref=place,
    )



@pytest.fixture
def store(tmp_path):
    s = SqliteStore(path=str(tmp_path / "views.db"))
    es = derived_stream(s, ENTITY_STREAM_NAME, Entity)
    append_entity(es, entity("e1", "chair", T0 + 10))
    append_entity(es, entity("e2", "chair", T0 + 20, position=(5.0, 5.0, 0.0)))
    append_entity(es, entity("e3", "desk", T0 + 30, support=2, coh=0.05))
    append_entity(es, entity("e4", "person", T0 + 40, place="cell(0,1)"))
    # Two sightings of one chair. Folded they are one entity; raw they are two,
    # which is what `deduplicated` exists to keep a caller from confusing.
    bs = belief_stream(s)
    for oid, ts in ((1, T0 + 10), (2, T0 + 11)):
        append_belief(
            bs,
            BeliefObservation(
                schema_version=SCHEMA_VERSION,
                target_ref=f"color_image#{oid}:0",
                label="chair",
                visibility="present",
                valid_ts=ts,
                observed_ts=ts,
                source="test",
                evidence=(EvidenceRef(stream="color_image", observation_id=oid, ts=ts),),
                confidence=Confidence(detection=0.9),
            ),
        )
    yield s
    s.stop()


class TestTheAskingTimeIsRequired:
    def test_as_of_is_not_defaulted(self, store):
        with pytest.raises(QueryError, match="as_of"):
            execute(store, {"select": "entities"})

    def test_select_is_not_defaulted(self, store):
        with pytest.raises(QueryError, match="select"):
            execute(store, {"as_of": T0})

    def test_nothing_after_the_asking_time_is_visible(self, store):
        early = execute(store, {"select": "entities", "as_of": T0 + 15, "project": "locate"})
        late = execute(store, {"select": "entities", "as_of": T0 + 100, "project": "locate"})
        assert len(early["result"]["positions"]) == 1
        assert len(late["result"]["positions"]) == 4


class TestFiltersComposeInOneCall:
    def test_two_clauses_narrow_further_than_one(self, store):
        one = execute(
            store,
            {
                "select": "entities",
                "as_of": T0 + 100,
                "where": [{"op": "label", "value": "chair"}],
                "project": "locate",
            },
        )
        two = execute(
            store,
            {
                "select": "entities",
                "as_of": T0 + 100,
                "where": [
                    {"op": "label", "value": "chair"},
                    {"op": "time_range", "t1": T0, "t2": T0 + 15},
                ],
                "project": "locate",
            },
        )
        assert len(one["result"]["positions"]) == 2
        assert len(two["result"]["positions"]) == 1

    def test_an_unknown_operator_raises_rather_than_being_skipped(self, store):
        with pytest.raises(QueryError, match="bogus"):
            execute(
                store,
                {"select": "entities", "as_of": T0, "where": [{"op": "bogus"}]},
            )


class TestEmptyResultsExplainThemselves:
    def test_no_match_reports_which_clause_emptied_it(self, store):
        out = execute(
            store,
            {
                "select": "entities",
                "as_of": T0 + 100,
                "where": [{"op": "label", "value": "unicorn"}],
                "project": "locate",
            },
        )
        assert out["status"] == "unknown"
        steps = out["diagnostic"]["per_clause"]
        assert steps[-1]["rows_after"] == 0
        assert steps[-1]["rows_before"] > 0

    def test_the_surviving_clause_count_is_reported(self, store):
        out = execute(
            store,
            {
                "select": "entities",
                "as_of": T0 + 100,
                "where": [
                    {"op": "label", "value": "chair"},
                    {"op": "time_range", "t1": T0 + 90, "t2": T0 + 99},
                ],
            },
        )
        steps = out["diagnostic"]["per_clause"]
        assert steps[0]["rows_after"] == 2
        assert steps[1]["rows_after"] == 0


class TestTheAnswerCarriesItsOwnTrustworthiness:
    def test_answers_say_they_are_about_things_not_sightings(self, store):
        out = execute(store, {"select": "entities", "as_of": T0 + 100, "project": "locate"})

        # The only select is entities, so this is always true -- and asserted
        # anyway, because the field is what stops a caller reading a result as a
        # count of sightings if raw observations are ever queryable again.
        assert out["quality"]["deduplicated"] is True

    def test_low_coherence_downgrades_the_status(self, store):
        out = execute(
            store,
            {
                "select": "entities",
                "as_of": T0 + 100,
                "where": [{"op": "label", "value": "desk"}],
                "project": "locate",
            },
        )
        # One sighting cluster too scattered to be one object: answering with
        # its centroid would be a position invented from nothing.
        assert out["status"] == "unknown"
        assert out["reason"] == "INCOHERENT"
