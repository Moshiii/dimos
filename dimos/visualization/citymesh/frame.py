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

"""Local ENU frame — the single place geodetic coordinates become metres.

Everything in citymesh (terrain, buildings, robot poses, detections) is expressed
in one local East/North/Up frame anchored at a chosen origin. Two rules keep the
scene aligned:

1. Altitudes are ambiguous until you name their datum. GNSS receivers compute
   ELLIPSOIDAL height (above the WGS84 ellipsoid) but most of them report
   ORTHOMETRIC height (above the geoid, i.e. mean sea level). The difference is
   the geoid undulation N, and around Athens it is roughly +39 m::

       h_ellipsoidal = h_orthometric + N

   Feed an MSL altitude into a pipeline expecting ellipsoidal (or vice versa) and
   the robot floats ~39 m off the buildings. This module stores the origin in
   ellipsoidal height and converts everything at the boundary, so the mistake is
   impossible to make downstream.

2. Stay near the origin. Rerun stores positions as float32, where the
   representable spacing at ECEF magnitudes (~6.4e6 m) is about 0.5 m. Local ENU
   keeps coordinates within a few km of zero, where spacing is well under a
   millimetre. Over a city-scale patch the curvature error from a tangent-plane
   approximation is also sub-millimetre, so this costs nothing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

# WGS84 defining parameters.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

AltDatum = Literal["ellipsoidal", "msl"]


class GeoidUnavailableError(RuntimeError):
    """Raised when the geoid model needed for an MSL conversion cannot be loaded."""


def geoid_undulation(lat: float, lon: float) -> float:
    """Height of the EGM2008 geoid above the WGS84 ellipsoid, in metres.

    Positive around Greece (~+39 m). Needs the EGM2008 grid, which pyproj will
    fetch from its CDN on first use if network access is enabled.
    """
    try:
        import pyproj

        pyproj.network.set_network_enabled(True)  # type: ignore[attr-defined]
        # EPSG:4979 = WGS84 3D (ellipsoidal height).
        # EPSG:4326+3855 = WGS84 2D + EGM2008 orthometric height.
        tf = pyproj.Transformer.from_crs("EPSG:4979", "EPSG:4326+3855", always_xy=True)
        _, _, h_orth = tf.transform(lon, lat, 0.0)
        if h_orth is None or not np.isfinite(h_orth):
            raise ValueError("transform returned no vertical result")
    except Exception as exc:
        raise GeoidUnavailableError(
            f"could not evaluate EGM2008 geoid at {lat:.5f},{lon:.5f}: {exc}. "
            "Pass an explicit undulation (--geoid-undulation) rather than guessing; "
            "a wrong value shifts the whole scene vertically."
        ) from exc
    # We asked for the orthometric height of the point whose ellipsoidal height is
    # 0, so N = 0 - h_orth.
    return -float(h_orth)


def geodetic_to_ecef(
    lat: np.ndarray, lon: np.ndarray, h: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """WGS84 geodetic (degrees, degrees, ellipsoidal metres) -> ECEF metres."""
    lat_r = np.radians(lat)
    lon_r = np.radians(lon)
    sin_lat = np.sin(lat_r)
    cos_lat = np.cos(lat_r)
    n = _A / np.sqrt(1.0 - _E2 * sin_lat**2)
    x = (n + h) * cos_lat * np.cos(lon_r)
    y = (n + h) * cos_lat * np.sin(lon_r)
    z = (n * (1.0 - _E2) + h) * sin_lat
    return x, y, z


def ecef_to_geodetic(
    x: np.ndarray, y: np.ndarray, z: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ECEF metres -> WGS84 geodetic (degrees, degrees, ellipsoidal metres).

    Uses Bowring's method, which converges to well under a millimetre in one
    iteration for near-surface points.
    """
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    ep2 = _E2 / (1.0 - _E2)
    b = _A * (1.0 - _F)
    theta = np.arctan2(z * _A, p * b)
    lat = np.arctan2(
        z + ep2 * b * np.sin(theta) ** 3,
        p - _E2 * _A * np.cos(theta) ** 3,
    )
    n = _A / np.sqrt(1.0 - _E2 * np.sin(lat) ** 2)
    h = p / np.cos(lat) - n
    return np.degrees(lat), np.degrees(lon), h


@dataclass(frozen=True)
class EnuFrame:
    """A tangent-plane ENU frame anchored at a geodetic origin.

    The origin altitude is always held as an ellipsoidal height. Build one with
    :meth:`at` so the datum you actually have is converted explicitly.
    """

    lat0: float
    lon0: float
    h0_ellipsoidal: float
    undulation: float  # EGM2008 geoid height at the origin, metres.

    @classmethod
    def at(
        cls,
        lat: float,
        lon: float,
        alt: float = 0.0,
        datum: AltDatum = "msl",
        undulation: float | None = None,
    ) -> EnuFrame:
        """Anchor a frame at ``lat, lon, alt``.

        ``datum`` names what ``alt`` means. ``"msl"`` (orthometric, the common
        receiver output) is the default because it is what a human quoting "60 m
        above sea level" means.
        """
        n = geoid_undulation(lat, lon) if undulation is None else float(undulation)
        h_ell = alt + n if datum == "msl" else alt
        return cls(lat0=lat, lon0=lon, h0_ellipsoidal=h_ell, undulation=n)

    @property
    def origin_msl(self) -> float:
        """Origin altitude as an orthometric (mean-sea-level) height."""
        return self.h0_ellipsoidal - self.undulation

    def to_ellipsoidal(self, alt: np.ndarray | float, datum: AltDatum) -> np.ndarray:
        """Normalise an altitude into ellipsoidal height.

        The undulation at the frame origin is reused across the patch; it varies
        by only a few centimetres over a city, far below the metre-scale error
        this exists to prevent.
        """
        alt = np.asarray(alt, dtype=float)
        return alt + self.undulation if datum == "msl" else alt

    def geodetic_to_enu(
        self,
        lat: np.ndarray | float,
        lon: np.ndarray | float,
        alt: np.ndarray | float = 0.0,
        datum: AltDatum = "msl",
    ) -> np.ndarray:
        """Geodetic -> local ENU metres. Returns an ``(N, 3)`` array of E, N, U."""
        lat = np.atleast_1d(np.asarray(lat, dtype=float))
        lon = np.atleast_1d(np.asarray(lon, dtype=float))
        h = np.broadcast_to(self.to_ellipsoidal(alt, datum), lat.shape)

        x, y, z = geodetic_to_ecef(lat, lon, h)
        x0, y0, z0 = geodetic_to_ecef(
            np.array(self.lat0), np.array(self.lon0), np.array(self.h0_ellipsoidal)
        )
        return self._ecef_delta_to_enu(x - x0, y - y0, z - z0)

    def enu_to_geodetic(
        self, enu: np.ndarray, datum: AltDatum = "msl"
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Local ENU metres -> geodetic, with the altitude in ``datum``."""
        enu = np.atleast_2d(np.asarray(enu, dtype=float))
        e, n, u = enu[:, 0], enu[:, 1], enu[:, 2]

        lat_r = np.radians(self.lat0)
        lon_r = np.radians(self.lon0)
        sin_lat, cos_lat = np.sin(lat_r), np.cos(lat_r)
        sin_lon, cos_lon = np.sin(lon_r), np.cos(lon_r)

        dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u
        dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u
        dz = cos_lat * n + sin_lat * u

        x0, y0, z0 = geodetic_to_ecef(
            np.array(self.lat0), np.array(self.lon0), np.array(self.h0_ellipsoidal)
        )
        lat, lon, h = ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)
        if datum == "msl":
            h = h - self.undulation
        return lat, lon, h

    def _ecef_delta_to_enu(self, dx: np.ndarray, dy: np.ndarray, dz: np.ndarray) -> np.ndarray:
        lat_r = np.radians(self.lat0)
        lon_r = np.radians(self.lon0)
        sin_lat, cos_lat = np.sin(lat_r), np.cos(lat_r)
        sin_lon, cos_lon = np.sin(lon_r), np.cos(lon_r)

        e = -sin_lon * dx + cos_lon * dy
        n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return np.column_stack([e, n, u])

    def bbox_deg(self, radius_m: float) -> tuple[float, float, float, float]:
        """Lon/lat bounding box covering ``radius_m`` around the origin.

        Returns ``(min_lon, min_lat, max_lon, max_lat)``.
        """
        dlat = np.degrees(radius_m / _A)
        dlon = np.degrees(radius_m / (_A * np.cos(np.radians(self.lat0))))
        return (
            self.lon0 - dlon,
            self.lat0 - dlat,
            self.lon0 + dlon,
            self.lat0 + dlat,
        )
