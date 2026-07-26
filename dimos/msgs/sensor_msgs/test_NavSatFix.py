#!/usr/bin/env python3
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

import math

from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix


def test_lcm_encode_decode():
    original = NavSatFix(
        ts=1785068167.5,
        frame_id="gps",
        latitude=37.993847,
        longitude=23.725472,
        status=NavSatFix.STATUS_FIX,
        service=NavSatFix.SERVICE_GPS,
        position_covariance=[30.25, 0, 0, 0, 30.25, 0, 0, 0, 1e6],
        position_covariance_type=NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
    )
    decoded = NavSatFix.lcm_decode(original.lcm_encode())

    assert decoded.latitude == original.latitude
    assert decoded.longitude == original.longitude
    assert math.isnan(decoded.altitude)  # default altitude survives the round trip
    assert decoded.status == NavSatFix.STATUS_FIX
    assert decoded.service == NavSatFix.SERVICE_GPS
    assert decoded.position_covariance == original.position_covariance
    assert decoded.position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    assert decoded.frame_id == "gps"
    assert abs(decoded.ts - original.ts) < 1e-6
    assert decoded.has_fix


def test_no_fix_status_is_negative():
    # STATUS_NO_FIX must be -1 (an i8 on the wire); a searching receiver's fix
    # decodes as no-fix, distinct from SBAS_FIX=1.
    nofix = NavSatFix(status=NavSatFix.STATUS_NO_FIX)
    decoded = NavSatFix.lcm_decode(nofix.lcm_encode())
    assert decoded.status == -1
    assert not decoded.has_fix
