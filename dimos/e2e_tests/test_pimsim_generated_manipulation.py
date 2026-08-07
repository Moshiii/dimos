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

from dimos.e2e_tests.pimsim_case import PimSimCaseRun, PimSimTabletopCase

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


TABLETOP_CASES = {
    "lift-object": PimSimTabletopCase(
        case_id="lift-object/alphabet-soup/scene-296/variation-3",
        family_id="lift-object",
        scene_seed=296,
        variation_seed=3,
        semantic_roles={"object": "alphabet-soup"},
    ),
    "object-in-receptacle": PimSimTabletopCase(
        case_id="object-in-receptacle/alphabet-soup/wooden-tray/scene-296/variation-3",
        family_id="object-in-receptacle",
        scene_seed=296,
        variation_seed=3,
        semantic_roles={"object": "alphabet-soup", "target": "wooden-tray"},
    ),
    "object-on-support": PimSimTabletopCase(
        case_id="object-on-support/tomato-sauce/plate/scene-296/variation-3",
        family_id="object-on-support",
        scene_seed=296,
        variation_seed=3,
        semantic_roles={"object": "tomato-sauce", "target": "plate"},
    ),
    "collect-objects-in-receptacle": PimSimTabletopCase(
        case_id=("collect-objects-in-receptacle/soup-and-cheese/wooden-tray/scene-296/variation-3"),
        family_id="collect-objects-in-receptacle",
        scene_seed=296,
        variation_seed=3,
        semantic_roles={
            "first_object": "alphabet-soup",
            "second_object": "cream-cheese",
            "target": "wooden-tray",
        },
    ),
    "rearrange-objects": PimSimTabletopCase(
        case_id="rearrange-objects/soup-in-tray/sauce-on-plate/scene-296/variation-3",
        family_id="rearrange-objects",
        scene_seed=296,
        variation_seed=3,
        semantic_roles={
            "first_object": "alphabet-soup",
            "second_object": "tomato-sauce",
            "containment_target": "wooden-tray",
            "support_target": "plate",
        },
    ),
}

PLACE_CASES = (
    pytest.param(TABLETOP_CASES["object-in-receptacle"], 0.10, id="object-in-receptacle"),
    pytest.param(TABLETOP_CASES["object-on-support"], 0.08, id="object-on-support"),
)


@pytest.mark.parametrize(
    "pimsim_case",
    [pytest.param(TABLETOP_CASES["lift-object"], id="lift-object")],
    indirect=True,
)
def test_lift_object(pimsim_case: PimSimCaseRun) -> None:
    observation = pimsim_case.wait_for_role("object")
    assert observation.success, observation

    result = pimsim_case.pick_and_place.pick(pimsim_case.role("object"))
    evaluation = pimsim_case.wait_for_goal()

    assert result.success, f"{result}; private evaluation: {evaluation}"
    assert evaluation["passed"] is True


@pytest.mark.parametrize(
    ("pimsim_case", "z_offset"),
    PLACE_CASES,
    indirect=("pimsim_case",),
)
def test_place_object(
    pimsim_case: PimSimCaseRun,
    z_offset: float,
) -> None:
    observation = pimsim_case.wait_for_role("object")
    assert observation.success, observation
    assert pimsim_case.role("target") in observation.message, observation

    pick_result = pimsim_case.pick_and_place.pick(pimsim_case.role("object"))
    assert pick_result.success, pick_result
    place_result = pimsim_case.pick_and_place.drop_on(
        pimsim_case.role("target"),
        z_offset=z_offset,
    )
    evaluation = pimsim_case.wait_for_goal()

    assert place_result.success, f"{place_result}; private evaluation: {evaluation}"
    assert evaluation["passed"] is True


@pytest.mark.parametrize(
    "pimsim_case",
    [pytest.param(TABLETOP_CASES["collect-objects-in-receptacle"], id="collect-objects")],
    indirect=True,
)
def test_collect_objects(pimsim_case: PimSimCaseRun) -> None:
    for role_id in ("first_object", "second_object"):
        observation = pimsim_case.wait_for_role(role_id)
        assert observation.success, observation
        pick_result = pimsim_case.pick_and_place.pick(pimsim_case.role(role_id))
        assert pick_result.success, pick_result
        place_result = pimsim_case.pick_and_place.drop_on(
            pimsim_case.role("target"),
            z_offset=0.10,
        )
        assert place_result.success, place_result

    evaluation = pimsim_case.wait_for_goal()
    assert evaluation["passed"] is True, evaluation


@pytest.mark.parametrize(
    "pimsim_case",
    [pytest.param(TABLETOP_CASES["rearrange-objects"], id="rearrange-objects")],
    indirect=True,
)
def test_rearrange_objects(pimsim_case: PimSimCaseRun) -> None:
    observation = pimsim_case.wait_for_role("first_object")
    assert observation.success, observation
    first_pick = pimsim_case.pick_and_place.pick(pimsim_case.role("first_object"))
    assert first_pick.success, first_pick
    first_place = pimsim_case.pick_and_place.drop_on(
        pimsim_case.role("containment_target"),
        z_offset=0.10,
    )
    assert first_place.success, first_place

    observation = pimsim_case.wait_for_role("second_object")
    assert observation.success, observation
    second_pick = pimsim_case.pick_and_place.pick(pimsim_case.role("second_object"))
    assert second_pick.success, second_pick
    second_place = pimsim_case.pick_and_place.drop_on(
        pimsim_case.role("support_target"),
        z_offset=0.08,
    )
    evaluation = pimsim_case.wait_for_goal()

    assert second_place.success, f"{second_place}; private evaluation: {evaluation}"
    assert evaluation["passed"] is True
