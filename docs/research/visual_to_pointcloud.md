# Visual Input → Point Cloud: Optimal Conversion for a Mobile Robot

**Scope:** stereo RGB/IR → depth → point cloud, targeted at an Intel RealSense D455 on a mobile robot.
**Date:** 2026-08
**Epistemic status markers used throughout:**

| Marker | Meaning |
|---|---|
| `[MEASURED]` | Verified on the D455 physically attached to this machine (serial `260922302422`, FW `5.15.1.55`), or computed from those measured values |
| `[VENDOR]` | Stated in first-party Intel/NVIDIA/OpenCV documentation |
| `[PAPER]` | The authors' own claim in their paper — self-reported, usually on their own hardware |
| `[INDEP]` | Third-party or re-benchmarked measurement |
| `[INFERRED]` | My reasoning from the above; not directly sourced |

---

## 0. Executive summary

The single highest-leverage fact in this whole document: **stereo depth error grows as the square of range**, and the only free variable in that relationship is subpixel matching precision.

```
                Z² · σ_subpixel
  σ_Z  =  ─────────────────────────
            f_pixels · Baseline
```

Everything else — algorithm choice, filtering, presets — is an argument about how to minimize `σ_subpixel` and how to *detect and discard* the pixels where it is invalid. For a D455 the `f·B` product is fixed by hardware at **40,948 px·mm at 848×480** and **61,808 px·mm at 1280×720** `[MEASURED]`. That is the budget you are spending.

The second most important fact: **the dominant visual artifact in stereo point clouds is not noise, it is flying pixels at depth discontinuities** — a ramp of phantom points connecting an obstacle to the free space in front of it. These are caused by the fronto-parallel window assumption, they are *spatially structured* (a continuous ribbon along every silhouette), and therefore statistical outlier removal does not touch them. They require a geometric test, not a statistical one.

---

## 1. Verified D455 ground truth (this specific unit)

Pulled directly from the attached device via `rs-enumerate-devices -c`. **These supersede the datasheet nominal values for any code you write.**

### 1.1 Device

| Property | Value | Note |
|---|---|---|
| Firmware | `5.15.1.55` | `[MEASURED]` |
| USB | `3.2` | `[MEASURED]` — always assert this at startup; silent USB2 fallback caps you at ~10 fps |
| IMU | **BMI085** | `[MEASURED]` — *not* the BMI055 in older docs. Later production SKUs (K83122-110/111) swapped it via PCN #118035-00. Do not hardcode. |
| Advanced Mode | YES | `[MEASURED]` — presets available |
| Product ID | `0B5C` | `[MEASURED]` |

### 1.2 Stereo geometry — the numbers that matter

| Quantity | Measured value | Common wrong value |
|---|---|---|
| **Baseline (depth → infrared2 translation)** | **94.862 mm** | 95 mm nominal; **50 mm** if code was copy-pasted from a D435 |
| Depth → Color translation | `(-59.153, 0.099, 0.311)` mm | — |
| Depth → Infrared 1 | identity | depth *is* the rectified left IR frame |

Using nominal 95 mm instead of the measured 94.862 mm is a **+0.15% systematic depth scale error** `[MEASURED]` — small, but free to eliminate, and it is a pure scale so it never averages out. Using a D435's 50 mm on a D455 is a **1.9× depth error**.

> **Cross-reference:** this repo has a known prior issue where `CameraInfo.P[3]` was 0 and the TF said 50 mm against the real 94.86 mm. The measured 94.862 mm here confirms that diagnosis exactly.

### 1.3 Depth intrinsics `[MEASURED]`

| Resolution | fx = fy | ppx | ppy | HFOV | Distortion coeffs |
|---|---|---|---|---|---|
| 1280×720 | **651.558** | 647.829 | 356.007 | 88.97° | all zero |
| 848×480 | **431.657** | 429.187 | 237.355 | 88.97° | all zero |
| 848×100 | 431.657 | 429.187 | 47.355 | 88.97° | all zero |
| 640×480 | **390.935** | 324.697 | 237.604 | **78.60°** | all zero |
| 640×360 | 325.779 | 323.914 | 178.004 | 88.97° | all zero |
| 480×270 | 244.334 | 242.936 | 133.503 | 88.97° | all zero |
| 424×240 | 215.829 | 214.593 | 118.677 | 88.97° | all zero |
| 256×144 | 651.558 | 135.829 | 68.007 | 22.23° | all zero |

**Three non-obvious things in that table:**

1. **The depth stream is distortion-free.** All five Brown-Conrady coefficients are exactly zero, because the D4 ASIC rectifies in hardware. Deprojection of D455 depth is therefore *exactly* the plain pinhole formula — no undistortion iteration, ever. `[MEASURED]`
2. **640×480 is not a downscale of 848×480.** It has fx = 390.9 and HFOV 78.6°, versus 431.7 / 88.97° for every 16:9 mode. It is a *different crop* of the 1280×800 imager, and you lose 10° of horizontal FOV. `[MEASURED]` If you switch resolutions expecting the same field of view, you will silently lose the periphery. Prefer 848×480 over 640×480 — it is both wider and higher-resolution.
3. **256×144 shares 1280×720's focal length** (651.558) — it is a centre crop, not a downscale, which is why it is the self-calibration format.

Note also: the real HFOV is **88.97°**, not the 90° the datasheet's raw-imager spec implies nor the 87° quoted as "depth FOV" (that figure is the *stereo-overlap* usable region, which is narrower than the lens FOV but does not change the focal length). Using 90° to derive `fx = Xres/2 = 424` gives a **1.8% error** against the measured 431.657. **Always read intrinsics from the device.** `[MEASURED]`

### 1.4 Color intrinsics `[MEASURED]`

848×480: fx = 425.652, fy = 424.985, ppx = 425.427, ppy = 242.077, HFOV 89.78°.
Distortion model: **Inverse Brown Conrady**, coefficients:

```
k1 = -0.05730599   k2 = 0.06331122   p1 = -0.00060748   p2 = 0.00059132   k3 = -0.02012500
```

So depth FOV (88.97°) ⊂ color FOV (89.78°) — this is the real meaning of the D455's "matched FOV" marketing, and it means **depth→color alignment loses essentially nothing**, unlike the D435 where depth 87° vs color 69° throws away a quarter of the frame.

Magnitude of ignoring color distortion, computed with the measured coefficients at 848×480 `[MEASURED]`:

| Location | radial scale | displacement | 3D error @2 m | @5 m |
|---|---|---|---|---|
| mid-right | 0.98919 | 2.32 px | 10.9 mm | 27.2 mm |
| right edge | 0.98571 | 6.02 px | 28.3 mm | 70.8 mm |
| corner | 0.98823 | 5.69 px | 26.7 mm | 66.8 mm |

Worth noting these are considerably *milder* than the textbook `k1 ≈ 0.16` example often cited for RealSense color (which would give ~27 px) — this unit's `k1` is negative and small. Still, **7 cm of lateral error at 5 m in the periphery** is enough to smear a texture-mapped cloud and to corrupt any extrinsic calibration solved across the full frame.

### 1.5 Stream modes `[MEASURED]`

- Depth 1280×720: **30/15/5 Hz only** — the "1280×720 @ 90 fps" claim in marketing copy is false for the D455.
- Depth 848×480: 90/60/30/15/5 Hz.
- Depth 848×100: **100 Hz** on this firmware. (Intel's high-speed-mode whitepaper advertises 300 fps for the D435 at this resolution; **this D455 at FW 5.15.1.55 exposes only 100 Hz** — flagging the discrepancy rather than repeating the 300 figure.) `[MEASURED]`
- Color max 1280×800 @ 30 Hz.

### 1.6 Derived hardware limits `[MEASURED]`

The D4 ASIC searches a fixed **126-disparity** window `[VENDOR]`, giving `MinZ = f·B / 126`:

| Resolution | `f·B` (px·mm) | MinZ (formula) | MinZ (datasheet) |
|---|---|---|---|
| 1280×720 | 61,808 | 491 mm | 520 mm |
| 848×480 | 40,948 | 325 mm | 350 mm |
| 640×480 | 37,085 | 294 mm | 320 mm |

Datasheet values carry ~7–8% margin over the naive formula `[INFERRED]`. **MinZ scales linearly with horizontal resolution** — this is the real cost of running at 1280×720: you cannot see anything closer than ~52 cm.

---

## 2. The depth error curve — the governing physics

### 2.1 Derivation

From `Z = f·B / d`, differentiate: `dZ/dd = -f·B/d² = -Z²/(f·B)`. Therefore

```
  |ΔZ| = (Z² / (f·B)) · Δd
```

Intel states exactly this `[VENDOR]` ([BKM tuning guide](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)):

```
  Depth RMS error (mm) = Distance(mm)² × Subpixel / (focal_length(px) × Baseline(mm))
```

Equivalently, **fractional error is linear in Z**: `err% = Z · σ_sub / (f·B) · 100`.

### 2.2 Measured D455 error tables

Computed with the **measured** `fx` and `B = 94.862 mm` `[MEASURED]`.

**848×480 (f·B = 40,948):**

| Z | σ=0.05 (best passive) | σ=0.08 (realistic) | σ=0.20 (datasheet spec) | quantization @1/32 px |
|---|---|---|---|---|
| 0.5 m | 0.3 mm (0.06%) | 0.5 mm (0.10%) | 1.2 mm (0.24%) | 0.2 mm |
| 1 m | 1.2 mm (0.12%) | 2.0 mm (0.20%) | 4.9 mm (0.49%) | 0.8 mm |
| 2 m | 4.9 mm (0.24%) | 7.8 mm (0.39%) | 19.5 mm (0.98%) | 3.1 mm |
| **4 m** | 19.5 mm (0.49%) | 31.3 mm (0.78%) | **78.1 mm (1.95%)** | 12.2 mm |
| 6 m | 44.0 mm (0.73%) | 70.3 mm (1.17%) | 175.8 mm (2.93%) | 27.5 mm |
| 8 m | 78.1 mm (0.98%) | 125.0 mm (1.56%) | 312.6 mm (3.91%) | 48.8 mm |
| 10 m | 122.1 mm (1.22%) | 195.4 mm (1.95%) | 488.4 mm (4.88%) | 76.3 mm |
| 15 m | 274.7 mm (1.83%) | 439.6 mm (2.93%) | 1099 mm (7.33%) | 171.7 mm |
| 20 m | 488.4 mm (2.44%) | 781.5 mm (3.91%) | 1954 mm (9.77%) | 305.3 mm |

**1280×720 (f·B = 61,808) — uniformly ~34% better:**

| Z | σ=0.05 | σ=0.08 | σ=0.20 | quant @1/32 px |
|---|---|---|---|---|
| 0.5 m | 0.2 mm (0.04%) | 0.3 mm (0.06%) | 0.8 mm (0.16%) | 0.1 mm |
| 1 m | 0.8 mm (0.08%) | 1.3 mm (0.13%) | 3.2 mm (0.32%) | 0.5 mm |
| 2 m | 3.2 mm (0.16%) | 5.2 mm (0.26%) | 12.9 mm (0.65%) | 2.0 mm |
| **4 m** | 12.9 mm (0.32%) | 20.7 mm (0.52%) | **51.8 mm (1.29%)** | 8.1 mm |
| 6 m | 29.1 mm (0.49%) | 46.6 mm (0.78%) | 116.5 mm (1.94%) | 18.2 mm |
| 8 m | 51.8 mm (0.65%) | 82.8 mm (1.04%) | 207.1 mm (2.59%) | 32.4 mm |
| 10 m | 80.9 mm (0.81%) | 129.4 mm (1.29%) | 323.6 mm (3.24%) | 50.6 mm |
| 15 m | 182.0 mm (1.21%) | 291.2 mm (1.94%) | 728.1 mm (4.85%) | 113.8 mm |
| 20 m | 323.6 mm (1.62%) | 517.7 mm (2.59%) | 1294 mm (6.47%) | 202.2 mm |

### 2.3 Reading these tables

- **Intel's "<2% RMS at 4 m" spec** `[VENDOR]` inverts to **σ_subpixel ≈ 0.20 px at 848×480** `[MEASURED, derived]`. That is a *worst-case acceptance budget*, not typical performance. A healthy, well-calibrated D455 on textured scenes should hit 0.05–0.08 px, i.e. **2.5–4× better than spec**.
- **σ_subpixel is the only variable you control.** Intel's own figures `[VENDOR]`: well-textured passive target with projector off gives **0.03–0.05**; with the laser projector on it degrades to **0.07–0.11** — *laser speckle costs you ~30% of your depth precision*. Recalibrate if you measure > 0.2 px over the central ROI.
- **Quantization is not the limiting factor.** The 1/32-px column is 2.5–4× below even the best-case noise column at every range. Do not spend effort on finer subpixel encoding; spend it on matching quality.
- **Max useful range.** Intel's ideal envelope is **0.6–6 m** `[VENDOR]`; the ≤2% claim is only made to 4 m. "Up to 20 m" appears on retail listings and is technically true but at 848×480 you have 0.5–0.8 m of 1σ noise there. **Practical rule: trust ≤6 m, treat 6–10 m as coarse/low-weight, discard >10 m.** `[INFERRED]`

### 2.4 Anisotropy — the error ellipsoid is a needle

At range the uncertainty is *not* isotropic. With `σ_d = 0.1 px` at 5 m on 848×480 `[MEASURED, derived]`:

```
  σ_Z = Z²·σ_d/(f·B) = 5000²·0.1/40948 = 61.1 mm
  σ_X = Z·σ_pix/f    = 5000·0.1/431.66 =  1.16 mm
```

A **53:1 ratio.** Isotropic voxel fusion throws this entirely away. The correct reference is still Matthies & Shafer, *Error Modeling in Stereo Navigation*, IEEE J. Robotics & Automation 3(3):239–248, 1987 ([PDF](https://www.ri.cmu.edu/pub_files/pub3/matthies_l_1987_1/matthies_l_1987_1.pdf)) — including its warning that even the 3D Gaussian "fails to represent the longer tails of the true error distribution... the skew becomes more pronounced the more distant the points."

---

## 3. Depth estimation from stereo

### 3.1 What the D4 ASIC actually does

The D455's depth is computed entirely on the **Intel RealSense Vision Processor D4** (28 nm), zero host CPU. It is a hardware **SGM variant**. This is not marketing inference — the advanced-mode parameter set exposes the internals directly (`src/ds/advanced_mode/presets.cpp` in librealsense):

| Exposed parameter | What it reveals |
|---|---|
| `censusUDiameter/vDiameter = 9×9` | 9×9 **census transform** matching cost |
| `lambdaCensus`, `lambdaAD` | cost = weighted blend of **Census + Absolute Difference** |
| `sloK1Penalty`, `sloK2Penalty` | classic SGM **P1/P2 scanline penalties** |
| `disableSLOLeftColor/RightColor` | per-direction scanline optimization toggles |
| `lrAgreeThreshold` | left-right consistency check |
| `deepSeaSecondPeakThreshold` | best-vs-second-best cost margin (uniqueness/ratio test) |
| `rsm*`, `rsvc*` | robust subpixel refinement; region validity (N/S/E/W minima) |

Corroborated at a high level by Keselman et al., *Intel RealSense Stereoscopic Depth Cameras*, CVPRW 2017 ([ar5iv](https://ar5iv.labs.arxiv.org/html/1705.05548)) `[PAPER]`.

**Hard limits of the ASIC:**

- **126 disparity levels**, not configurable. Sets MinZ (§1.6). Only the *disparity shift* (0–128) moves the window; it does not widen it.
- **1/32 px subpixel** `[VENDOR]`. All filter `smooth_delta` params are in these units.
- **No confidence map.** Critically: *"There is no confidence map provided, data is either returned or marked zero to indicate that it is invalid"* `[PAPER, Keselman]`. You get a **binary** valid/invalid mask, not graded confidence. This is a genuine limitation versus e.g. NVIDIA ESS or Luxonis DepthAI (0–255 graded), and it constrains your quality-control options (§5.3).
- **Default depth units 1 mm** (`param-zunits: 1000`), max representable 65.5 m.

### 3.2 Classical stereo on the host CPU

Only relevant if you bypass the ASIC (e.g. to run your own matcher on the raw IR pair).

**OpenCV `StereoSGBM` is not Hirschmüller SGM.** Documented deviations `[VENDOR]`: no mutual information (uses Birchfield–Tomasi), block-based not pixel-based, and *"single-pass, which means that you consider only 5 directions instead of 8."* Reading the source (`stereosgbm.cpp`) shows the docs are stale — the default aggregates **4 causal paths per pass**; `MODE_HH` runs 2 passes → 8 paths; `MODE_SGBM_3WAY` is literally 3 paths; `MODE_HH4` is 4 (no diagonals).

Measured CPU timings (Intel i7-13620H, OpenCV 4.13, blockSize=5) `[INDEP]`, 16-thread / 1-thread:

| Res | D | MODE_SGBM | MODE_HH | MODE_SGBM_3WAY | MODE_HH4 |
|---|---|---|---|---|---|
| 848×480 | 96 | 61.8 / 58.7 ms | 116.7 / 117.2 | **16.3** / 45.0 | 42.4 / 100.7 |
| 848×480 | 128 | 71.9 / 70.6 | 141.2 / 143.6 | **21.9** / 53.8 | 52.1 / 126.7 |
| 1280×720 | 128 | 177.8 / 174.9 | 376.4 / 347.8 | **51.8** / 128.0 | 128.6 / 301.9 |
| 1280×720 | 256 | 294.2 / 268.7 | 604.9 / 593.6 | **66.3** / 207.9 | 217.6 / 525.3 |

**Key finding: `MODE_SGBM` and `MODE_HH` do not scale with threads at all** (identical 1- vs 16-thread). Only `MODE_SGBM_3WAY` has `parallel_for_`. If you need CPU SGBM at 30 Hz, `SGBM_3WAY` is the only viable mode.

**ELAS** (Geiger et al., ACCV 2010, [libelas](http://www.cvlibs.net/software/libelas/)): sparse support points → Delaunay triangulation → disparity prior → dense MAP. `[PAPER]` "<1 second on a single i7 core" for a 1-megapixel L+R pair. Degrades more gracefully than SGBM at high resolution / large disparity because the prior prunes the search.

Accuracy anchors on KITTI 2015 (D1-all) `[INDEP]`: ELAS 9.72%, OpenCV-SGBM 10.86%, versus current leaders at **1.28–1.35%**. Zero-shot on Middlebury (bad2.0): SGM **25.2%** vs RAFT-Stereo 8.7% `[PAPER, IGEV Tab.7]`. **Classical stereo is roughly 3× worse than a 2021 learned net on unseen indoor data.**

### 3.3 Learned stereo — accuracy tier

All KITTI figures are official leaderboard submissions; runtimes are authors' own on their own hardware `[PAPER]`.

| Method | Venue | KITTI15 D1-all | Middlebury bad2.0 | ETH3D bad1.0 | SceneFlow EPE | Params | Runtime / GPU |
|---|---|---|---|---|---|---|---|
| RAFT-Stereo | 3DV 2021 | 1.82 | 4.74 | 2.44 | 0.53 | 11.1 M | 0.38 s |
| CREStereo | CVPR 2022 | 1.69 | 3.71 | 0.98 | — | — | 0.41 s |
| IGEV-Stereo | CVPR 2023 | 1.59 | 4.83 | 1.12 | 0.47 | 12.6 M | **0.18 s** (16 it.) |
| Selective-IGEV | CVPR 2024 | 1.55 | **2.51** | 1.23 | 0.44 | 13.1 M | 0.24 s (16 it.) |
| IGEV++ | TPAMI 2025 | 1.51 | 3.23 | 1.14 | 0.67 | — | 280 ms |
| RT-IGEV++ | TPAMI 2025 | 1.79 | — | — | 0.52 | — | **48 ms** (6 it.) |
| FoundationStereo | CVPR 2025 | 1st MB+ETH3D | 1st | 0.26 (FT) | **0.34** | 60.6 M | 2.97 s @MB-half, RTX 3090 |

Sources: [RAFT-Stereo](https://arxiv.org/abs/2109.07547) · [CREStereo](https://arxiv.org/abs/2203.11483) · [IGEV](https://arxiv.org/abs/2303.06615) · [Selective-Stereo](https://arxiv.org/abs/2403.00486) · [IGEV++](https://arxiv.org/abs/2409.00638) · [FoundationStereo](https://arxiv.org/abs/2501.09898)

**The iteration knob — the cheapest latency control you have.** SceneFlow EPE vs inference iterations `[PAPER, IGEV Tab.2 / Selective Tab.3]`:

| Model | 1 it. | 2 | 3 | 4 | 8 | 32 |
|---|---|---|---|---|---|---|
| RAFT-Stereo | 2.16 | 1.21 | 0.95 | 0.82 | 0.66 | 0.61 |
| IGEV-Stereo | **0.66** | 0.62 | 0.58 | 0.55 | 0.50 | 0.47 |
| Selective-IGEV | 0.65 | 0.60 | 0.56 | 0.53 | 0.48 | 0.44 |

**RAFT-family collapses below ~8 iterations; IGEV-family barely moves.** IGEV at *1* iteration (0.66) equals RAFT at *8*. Cutting IGEV from 32→4 iterations costs ~17% EPE; cutting RAFT costs 40%+. If you deploy learned stereo with a latency budget, **use an IGEV-family model and cut iterations** — the cost is roughly linear in time.

### 3.4 Learned stereo — real-time / embedded tier

Re-benchmarked on one GPU (RTX 3090) by the LightStereo authors `[INDEP for baselines]` ([arXiv:2406.19833](https://arxiv.org/abs/2406.19833)):

| Method | Params | SceneFlow EPE | KITTI15 D1 | Time (RTX 3090) |
|---|---|---|---|---|
| StereoNet | 0.40 M | 1.10 | 4.83 | 20 ms |
| HITNet | 0.42 M | 0.55 | 1.98 | 36 ms |
| CoEx | 2.72 M | 0.67 | 2.13 | 36 ms |
| Fast-ACVNet+ | 3.20 M | 0.59 | 2.01 | 27 ms |
| LightStereo-S | 3.44 M | 0.73 | 2.30 | **17 ms** |
| LightStereo-L | 24.3 M | 0.59 | 1.93 | 37 ms |

Note the re-benchmark disagrees with self-reported numbers: HITNet claims 15 ms on a Titan V `[PAPER]` but measures 36 ms on a 3090 `[INDEP]`. **Prefer the re-benchmarked column.**

Independent in-the-wild comparison (Burrus, RTX 3090, FP32) `[INDEP]` ([source](https://nicolas.burrus.name/stereo-comparison/)) — peak GPU memory at 640×480 is the disqualifying metric on an 8 GB Jetson: OpenCV-SGBM 6 MB, RealtimeStereo 18 MB, RAFT-fast 172 MB, CREStereo 458 MB, **HITNet-accurate 2179 MB (6973 MB at 720p)**. Also: RAFT-Stereo-accurate takes **32.5 seconds** per VGA frame on a single CPU core — learned stereo on CPU is a non-starter.

### 3.5 Jetson-class deployment

| Option | Resolution | Precision | Throughput | Source |
|---|---|---|---|---|
| **Isaac ROS ESS** (17 M, + confidence output) | 576×960 | TRT FP16 | ~78.8 fps AGX Orin | `[INDEP, secondhand]` |
| Isaac ROS ESS Light | 288×480 | TRT FP16 | ~288 fps AGX Orin | `[INDEP, secondhand]` |
| **FoundationStereo** | 320×736 | TRT FP16 | **1.0 fps AGX Orin** | `[VENDOR, NVIDIA TAO card]` |
| FoundationStereo | — | — | **OOM on Orin 8 GB** | `[INDEP]` |
| **Fast-FoundationStereo** (12.4 M) | low-res | TRT | up to **26 fps AGX Orin** | `[PAPER]` |
| ESMStereo-S | 960×540 | TRT FP16 | **91 fps AGX Orin 64 GB** | `[PAPER]` |
| ESMStereo-M | 960×540 | TRT FP16 | 29 fps | `[PAPER]` |
| Lite Any Stereo V2-M | — | — | 101 ms on Orin 8 G | `[PAPER]` |
| DEFOM-Stereo ViT-S | 720×1280 | TRT FP16 | 450 ms (2.2 fps) | `[PAPER]` |
| libSGM (CUDA classical) | 1024×440, D=128 | — | 110.7 fps Xavier MaxN; 35 fps TX2 | `[VENDOR, repo]` |

**The single most important tradeoff in this section:** FoundationStereo is the best zero-shot stereo model in existence and it is **~1 Hz on an AGX Orin**, and OOMs on the 8 GB variant. It is not deployable for real-time navigation. Use **Fast-FoundationStereo** (~26 fps, 12.4 M params) or **ESS** (has a confidence output, ROS 2 integrated) instead.

TensorRT FP16 buys a consistent **~2.5–2.8×** over PyTorch FP32 on Orin `[PAPER, DEFOM table]`. No credible INT8 stereo numbers exist — INT8 for cost-volume networks is not a solved deployment path.

### 3.6 Zero-shot generalization — why this matters most for a robot

A robot meets scenes its model was never trained on. Trained on SceneFlow only, tested zero-shot `[PAPER, various]`:

| Method | Middlebury bad2.0 | ETH3D bad1.0 | KITTI-15 D1 |
|---|---|---|---|
| SGM (classical) | 25.2 | 12.9 | — |
| RAFT-Stereo | 8.7 | 3.2 | 5.5 |
| IGEV-Stereo | 7.1 | 3.6 | 5.7 |
| **FoundationStereo** | **5.5** | **1.8** | **4.9** |

With mixed-dataset training (targets excluded), FoundationStereo reaches Middlebury **1.1** / ETH3D **0.5** — a ~7× reduction over the previous best zero-shot method, **with no target-domain finetuning** `[PAPER]`. Independently reproduced at MB 1.12 / ETH3D 0.49 by the Lite Any Stereo authors `[INDEP]`. This is the strongest evidence in the field that a robot can drop in a stereo net without collecting data.

**But** note the MACs: FoundationStereo **12,824 G** vs Lite Any Stereo **33 G** — a ~390× gap. Zero-shot robustness is currently extremely expensive.

---

## 4. Monocular and metric depth — when it beats stereo, when it doesn't

### 4.1 Why monocular depth is scale-ambiguous (rigorously)

Perspective projection gives `u = f·X/Z + cx`. Two consequences:

1. **Scale ambiguity is exact and unremovable.** For any `s > 0`, a scene scaled by `s` viewed by a camera with focal length `s·f` produces a **pixel-identical** image. For a fixed camera, `{X, Z}` and `{sX, sZ}` are indistinguishable. A single view fixes the projective ray per pixel and *nothing along it*. The only escape is a prior on absolute size ("that is a car; cars are ~4.5 m") or a known focal length combined with known object scale. Monocular geometry is determined only up to the similarity group.
2. **Shift ambiguity is an artifact of training data.** MiDaS-style training mixes stereo-derived data with unknown baselines and unknown principal-point offsets, so supervision is only valid up to `d ≈ a/Z + b`. Networks therefore predict inverse depth up to **scale *and* shift** (2-DoF affine) and are trained with scale-and-shift-invariant losses. This is why relative-depth outputs must be least-squares-fitted against ground truth before any error metric is meaningful — **and why they cannot be used on a robot at all without an external metric anchor.**

**How metric models resolve it — three families:**

| Family | Mechanism | Requirement | Failure mode |
|---|---|---|---|
| Canonical camera / intrinsics conditioning | Metric3D v1/v2: rescale GT by `f_c/f` at train, multiply by `f` at test. Depth Anything 3: literally `metric = focal · net_out / 300` | You must **know** your intrinsics | 20% focal error → 20% depth error, linearly |
| FOV / focal embedding | DMD conditions on vertical FOV; ZoeDepth's domain bins are a coarse version | Known or estimated FOV | coarse |
| Learned camera prediction | UniDepth dense ray prompt; Depth Pro focal head; MoGe-2 scale head | none | *substitutes an estimation error for a measurement* |

Depth Pro's focal estimation achieves δ₂₅% of only **64.6–84.2%** `[PAPER]` — i.e. **~1/3 of images carry >25% focal error**, hence >25% scale error. "Intrinsics-free" is convenient, not free.

### 4.2 The models

| Model | Venue | Key numbers | Latency |
|---|---|---|---|
| **Depth Anything V2** | NeurIPS 2024 | Zero-shot relative AbsRel: NYU 0.045, KITTI 0.074. Metric finetunes: NYU δ₁ 0.984, KITTI δ₁ 0.983 `[PAPER]` | ViT-S 21 ms (A100); **ViT-S 50 ms / ~20 fps on Orin AGX @480×300** `[INDEP]`; 20 fps with TensorRT `[INDEP]` |
| **Depth Anything 3** | ICLR 2026 | KITTI δ₁ **97.1** vs DA2 94.6; ETH3D **98.8** vs 86.5 `[PAPER]` | A100 @504×336: Small 160 fps, Large 78 fps `[PAPER]` |
| **Metric3D v2** | TPAMI 2024 | NYU AbsRel 0.063/δ₁ 0.975; KITTI 0.052/0.974; trained on 16 M images `[PAPER]` | **no published latency** — a real gap |
| **UniDepth V2** | TPAMI 2025 | Aggregate zero-shot δ₁ **60.0**; iBims-1 94.5; best *stability* under perturbation `[PAPER/INDEP]` | **ViT-S 23.0 ms, ViT-B 35.1 ms, ViT-L 65.4 ms** @0.5 MP on A6000 `[PAPER]` |
| **Depth Pro** | ICLR 2025 | Boundary F1 Sintel **0.409** (Metric3Dv2 0.321, Marigold 0.068) — 2–6× better boundaries. But zero-shot metric δ₁ on ETH3D only **41.5** vs Metric3Dv2's 87.7 `[PAPER]` | 0.3 s/2.25 MP on V100 `[PAPER]`; **<15 s on an RTX 4050** `[INDEP]` |
| Marigold (diffusion) | CVPR 2024 | best detail at the time | **19–378 s per image** `[INDEP]` — non-starter on a robot |

**The most useful single result for a SLAM stack:** Metric3D as a depth prior in DROID-SLAM cuts KITTI translational RMS drift from **7.00–33.90% → 0.60–2.73%** `[PAPER]`. A good metric prior essentially eliminates classic monocular scale drift — *on in-distribution driving data*.

### 4.3 When mono beats stereo

All of these are corroborated by the fact that the mono-prior-fusion literature exists at all:

- **Textureless / repetitive surfaces** — stereo has no correspondence signal whatsoever; mono's contextual prior fills it.
- **Transparent and mirrored surfaces** — stereo systematically returns the scene *behind* the glass or the *reflected* scene. Adding a mono prior on Booster (mirrors) takes bad>2 from **17.84% → 9.96%** `[PAPER, Stereo Anywhere]`.
- **Occluded regions** — no right-image match exists. Middlebury occluded-region bad>2: **29.06% → 20.77%** with mono priors.
- **Thin structures and boundaries** — stereo cost volumes over-smooth and *fatten* poles and wires (§6). Depth Pro's boundary recall is 2–6× the field. This is currently mono's strongest genuine advantage.
- **Beyond stereo range** — stereo error is quadratic in Z; mono has no such range law (though also no guarantee).
- **Dense output everywhere**, no holes, no extrinsic calibration to maintain, no rectification drift.

### 4.4 When mono loses — the numbers that settle it

| Failure | Evidence |
|---|---|
| **Safety-critical obstacle distance** | MonoMPC measured **48.3% collision rate at 1.0 m/s** and 25% at 0.5 m/s for a standard ROS nav stack fed mono-depth costmaps. Cause: *"significant and stochastic offset between estimated and ground-truth point clouds, dependent on obstacle placement and camera FOV"* `[INDEP]` ([arXiv:2508.07387](https://arxiv.org/html/2508.07387v1)) |
| **Out-of-distribution scenes** | DA-V2-B AbsRel: industrial hall 0.194 → **agricultural field 1.632** → **Rhône glacier 2.669** (depth off by 160–270%) `[INDEP]` |
| **Unusual camera height/pitch** | AerialMetric (UAV): zero-shot **ZoeDepth AbsRel 97.1%, δ₁ 0.0%**; MoGe-2 δ₁ collapsed 43.1% → 10.1% just by raising altitude to 120 m `[INDEP]` |
| **Domain shift (underwater)** | Full ranking flip: UniDepthV2-L AbsRel 0.1156 (best) vs **Depth Pro 0.9858**, **Metric3D V2-S 1.5331** — same models, ~30× spread `[INDEP]` |
| **Flat-wall hallucination** | On OOD geometry, *"DepthPro attempted reconstructing subtle details while UniDepthV2 predicted flat surfaces."* Depth "interpolates between the foreground and background object," producing physically impossible geometry — a *prior-driven, confident* error `[INDEP]` |
| **Temporal flicker** | Temporal Alignment Error on ScanNet: DA-V2 **1.140** vs Video Depth Anything 0.570. Per-frame mono is ~2× less temporally stable — free space appears and disappears between frames, poisoning occupancy fusion `[PAPER]` |
| **Scale set by a config constant** | DA-V2 metric outdoor: `max_depth=80` vs an empirically better 65 against LiDAR — a **~19% scale error** governed by a hyperparameter with no principled way to choose it `[INDEP]` |
| **Accuracy vs stability are different models** | Accuracy rank: DepthPro 1.24 < UniDepthV2 1.40. Stability rank: **UniDepthV2 0.27 < ... < DepthPro 0.30**. The most accurate model is not the most stable one `[INDEP]` |

**Bottom line: nobody should use monocular metric depth as the sole source of obstacle distance.** The 48.3% collision rate and the δ₁ = 0.0% aerial result are the two facts that settle it.

### 4.5 Hybrid — the actually-correct architecture

- **Stereo Anywhere** (CVPR 2025, [arXiv:2412.04472](https://arxiv.org/html/2412.04472v1)) — dual correlation volumes, one from stereo features and one from surface normals derived from Depth Anything V2, fused with learned weighting. Middlebury bad>2 **11.15 → 7.07** (−37%); KITTI-15 bad>3 5.44 → 3.98 `[PAPER]`.
  **The killer ablation** is their MonoTrap optical-illusion set: DA-V2 alone scores 27.62% while RAFT-Stereo hits 4.62% and Stereo Anywhere 4.59%. **The fused net correctly ignores mono hallucinations when stereo disagrees.** That is precisely the property a robot needs.
- **OmniDepth / BridgeDepth** ([arXiv:2508.04611](https://arxiv.org/html/2508.04611v1)) — bidirectional latent alignment. Zero-shot error reduction vs NMRF: Middlebury **−42.7%**, ETH3D **−65.8%** `[PAPER]`.
- **Sparse-anchor rescaling** — the cheap robotics recipe: take relative or mis-scaled mono depth and fit scale (or a *monotonic spline*) against sparse metric anchors from VIO features, LiDAR, radar, or **stereo disparity in regions where stereo is confident**. A drone team achieved weighted-average AbsRel **0.185** this way at 20 fps on Orin AGX `[PAPER]`.
- **Warning on naive rescaling:** blind *global* rescaling with a bad anchor is worse than nothing — naive radar rescaling of DA-V2 made one dataset go from AbsRel 0.194 → **2.705** `[PAPER]`.

**Verdict for a D455 robot:** mono depth is a *prior* and a *gap-filler*, never the primary metric source. The D455 already gives you metric stereo; the correct use of mono is (a) to fill the textureless/transparent regions stereo abandons, tagged at lower confidence, and (b) as an arbiter to detect when stereo has latched onto a repetitive-texture alias.

---

## 5. Depth → point cloud: deprojection done correctly

### 5.1 The math

For an **undistorted / rectified** stream:

```
  X = (u - cx) · Z / fx
  Y = (v - cy) · Z / fy
  Z = Z          ← PLANAR depth along the optical axis, NOT radial range
```

**Z-depth vs radial range.** They relate by `Z = R·cosθ` where `cosθ = 1/sqrt(1 + ((u-cx)/fx)² + ((v-cy)/fy)²)`. Some ToF and fisheye pipelines emit range. Mixing them makes a flat wall bulge by `R(1−cosθ)`.

**On the D455 this error is enormous, because the FOV is enormous.** Computed with the measured 848×480 intrinsics `[MEASURED]`:

| Location | cos θ | error | at 3 m |
|---|---|---|---|
| right edge | 0.7185 | **28.1%** | 844 mm |
| corner | 0.6666 | **33.3%** | **1000 mm** |

A full metre of error at 3 m in the corners. Textbook treatments of this pitfall usually quote ~6% because they assume a narrow-FOV lens (f≈615 at 640×480); at the D455's 89° HFOV the effect is **5× worse**. ROS ships a separate `point_cloud_xyz_radial` nodelet precisely for this case. **RealSense Z16 is planar Z** `[VENDOR]` — use the formula above, and never feed a range image through it.

**The half-pixel convention.** Two live conventions differ by exactly 0.5 px:

| Convention | Centre of top-left pixel | Users |
|---|---|---|
| Pixel-centre-at-integer | `(0, 0)` | **OpenCV, Kalibr, ROS `image_geometry`, librealsense, Open3D** |
| Pixel-corner-at-integer | `(0.5, 0.5)` | COLMAP, OpenGL/CUDA texture space |

COLMAP documents the conversion: `cx_colmap = cx_opencv + 0.5`. **Rule: deproject in whatever convention your calibration was solved in.** Do not "helpfully" add 0.5 to OpenCV/librealsense intrinsics.

Evidence librealsense is on the OpenCV convention `[VENDOR, source]`: `src/proc/pointcloud.cpp` feeds raw integer coordinates (`const float pixel[] = { (float)x, (float)y }`), and `rs2_fov` computes `atan2f(intrin->ppx + 0.5f, intrin->fx)` — i.e. the sensor's left edge sits at pixel coordinate −0.5.

Magnitude of getting it wrong: a half-pixel principal-point error is a pure lateral shear of `0.5·Z/fx` — at the D455's fx=431.66 that is **3.5 mm at 3 m, 11.6 mm at 10 m** `[MEASURED, derived]`. Small, but systematic, range-proportional, and it does not average out.

### 5.2 `rs2_deproject_pixel_to_point` — what it actually does

From `src/rs.cpp` `[VENDOR, source]`:

```c
assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY);  // no closed form
float x = (pixel[0] - intrin->ppx) / intrin->fx;
float y = (pixel[1] - intrin->ppy) / intrin->fy;
if (!is_intrinsics_distortion_zero(intrin)) { /* 10-iteration fixed-point undistort */ }
point[0] = depth * x;  point[1] = depth * y;  point[2] = depth;
```

- The assert blocks only `MODIFIED_BROWN_CONRADY` (defined as distortion applied *on projection*, so no closed-form inverse).
- `is_intrinsics_distortion_zero()` short-circuits the iteration when all coefficients are `< FLT_EPSILON`. **On the D455 depth stream this always fires** (§1.3) — so deprojecting D455 depth is exactly the plain pinhole formula, no iteration.
- **`INVERSE_BROWN_CONRADY`** is what the *color* stream uses: calibration stores the coefficients of the *undistort* map, so deprojection undistorts and projection applies them forward. This is why you must not hand-roll `(u-cx)/fx` on the raw color stream.

### 5.3 Which intrinsics, and what `rs2::align` really does

**Use the intrinsics of the stream the depth image is expressed in.** On a D400 the depth image is registered to the rectified left IR imager (depth→infrared1 extrinsics are identity `[MEASURED]`). Deprojecting depth with *color* intrinsics is a common silent bug — at 848×480 that is fx 431.66 vs 425.65, only 1.4%, but the principal points differ too, and on the D435 the equivalent error is ~45%.

Formally, using `K'` when truth is `K`: `X' = (fx/fx')·X + ((cx-cx')/fx')·Z` — **linear in (X, Z)**. So wrong-intrinsics errors *shear and scale* the cloud; they do **not** curve it. **Curvature always comes from a nonlinear term** (unmodelled distortion, or Z-vs-R confusion). Diagnostically: flat-but-tilted wall → suspect intrinsics/extrinsics; *bowed* wall → suspect distortion or range-vs-depth.

**`rs2::align` is a forward scatter with a z-buffer, not a resample** `[VENDOR, source]`:

```cpp
memset(aligned_data, 0, ...);                          // holes stay 0
for each depth pixel:
    if (depth == 0) continue;
    map corners (x±0.5, y±0.5) → deproject → transform → project
    if any corner outside target: continue;            // whole pixel dropped
    for each covered target pixel:
        out_z = out_z ? min(out_z, z) : z;             // nearest-surface z-buffer
```

Consequences to plan for:
1. **The output profile becomes the target stream's.** After `align_to_color` you must deproject with **color** intrinsics, and the cloud lives in the **color** optical frame. This is the one case where color intrinsics on a depth image is correct.
2. **Holes** from parallax and occlusion. Buffer is memset to 0, so unfilled = invalid.
3. **Blocky quantization** — the footprint rectangle is snapped to integers and the same depth is written to every covered pixel. Deliberately nearest-neighbour-like, to avoid inventing depths that correspond to no surface.
4. **`align_to_depth` (color→depth) is cheaper and lossless in depth**; `align_to_color` punches holes in depth. **Prefer aligning color to depth** and building the cloud in the depth frame where you can.
5. Do not combine `pc.map_to()` (which projects texcoords internally) with `align_to` — you will double-transform.

### 5.4 Disparity → depth, rectification, and the Q matrix

`Z = fx·B/d` assumes identical focal lengths, coplanar image planes, and optical centres collinear with the x-axis. Only rectification manufactures that. `cv::stereoRectify` returns `R1,R2` (virtual rotations) and `P1,P2` (new rectified intrinsics), plus:

```
        [ 1  0   0        -cx1      ]
  Q  =  [ 0  1   0        -cy       ]
        [ 0  0   0          f       ]
        [ 0  0 -1/Tx  (cx1-cx2)/Tx  ]
```

**The off-by-16 trap** `[VENDOR]`: *"If the disparity is 16-bit signed format, as computed by StereoBM or StereoSGBM... it should be divided by 16 before being used here."* Forgetting this makes everything **16× too close**. And the fixed-point scale is **not** universal:

| Source | Fixed-point divisor |
|---|---|
| OpenCV StereoBM/StereoSGBM | **16** |
| libSGM / `cv::cuda::StereoSGM` | **16** |
| NVIDIA VPI (Q10.5) | **32** |
| librealsense `DISPARITY32` | **32** |

Every hop between these needs an explicit rescale.

**What unrectified images actually do to you.** Two distinct failures:
- *(a) Matching failure* — a horizontal-search matcher matches wrong pixels or nothing.
- *(b) Systematic bowing.* Even with correct correspondences, residual radial distortion gives `d_meas = d·(1 + k1(r² + 2x²))`, so `Z_apparent ≈ Z·(1 − k1(r² + 2x²))` — a **quadratic-in-radius pull toward the camera at the periphery**. A flat wall becomes a bowl. This is the photogrammetry "banana/bowl" family of errors. On the D455 depth stream this is a non-issue (ASIC rectifies, coeffs are zero `[MEASURED]`); it bites you the moment you run your own matcher on the raw IR pair without rectifying.

**`alpha` in `stereoRectify`:** `alpha=0` crops to only-valid pixels (and **changes f and cx**); `alpha=1` keeps every source pixel but leaves black no-data borders the matcher will produce garbage in. Either way, **deproject with the resulting `P`, not the original `K`**.

### 5.5 Disparity shift and depth units

**Disparity shift** `s ∈ [0,128]` moves the ASIC's search window from `[0,126]` to `[s, s+126]`:

```
  newMinZ = f·B / (126 + s)          newMaxZ = f·B / s   (infinite at s=0)
```

Verified against Intel's own worked example `[VENDOR]`: shift 0 → MinZ 45 cm, MaxZ ∞; shift 50 → MinZ 30 cm, MaxZ 110 cm. Check: `f·B = 0.45·126 = 56.7`; `56.7/176 = 32 cm` ✓, `56.7/50 = 113 cm` ✓.

**MaxZ collapses far faster than MinZ improves.** Disparity shift is a fixed-rig near-field tool. **For a mobile robot, leave it at 0.** And if any layer of your stack recomputes depth from disparity, it must know `s` — otherwise every depth is wrong by `f·B·s/(d(d+s))`.

**Depth units.** `Z_m = raw_uint16 · depth_units`, so quantum = `depth_units` and max range = `65535 · depth_units`:

| depth_units | quantum | max range | verdict for a robot |
|---|---|---|---|
| 100 µm | 0.1 mm | **6.55 m** | too short — you lose everything past 6.5 m |
| **1000 µm (default)** | 1 mm | **65.5 m** | **correct choice** |
| 5000 µm | 5 mm | 327 m | pointless |

Compare the quantum against the physics: at 848×480 the D455's own noise is 4.9 mm at 2 m and 44 mm at 6 m `[MEASURED]`, so a 1 mm quantum is comfortably below the noise floor across the whole useful range. **Do not reduce depth units on a mobile robot** — you would gain precision you cannot use and lose range you need.

**Caution:** ROS `depth_image_proc` hard-codes `DepthTraits<uint16_t>::toMeters = 0.001`. A RealSense running non-default depth units **must** be republished as float32 metres or the ROS cloud is silently scaled wrong.

### 5.6 Invalid depth — never deproject a zero

REP-118 is normative `[VENDOR]`: for `16UC1`, **0 = invalid**; for `32FC1`, **NaN = invalid**.

**A zero depth deprojects to the camera optical centre**, producing a dense blob of points at the origin. Downstream this becomes (a) an obstacle *on top of the robot*, (b) a RANSAC inlier set that destroys ground-plane fitting, (c) a centroid dragged toward the origin. Symmetrically, an unmasked saturated `65535` becomes a point at 65 m. **Mask before the multiply, always.**

Note the `depth_image_proc` `invalid_depth` parameter, which maps invalid pixels to a specified value (typically max range). **This is the wrong choice for stereo** — it converts "I don't know" into "definitely free out to max range," which raytrace-clears real obstacles. It is only correct for sensors where invalid genuinely means "nothing within range."

**Holes have three distinct causes with three distinct correct responses:**
1. *Occlusion* (only one camera sees it) — geometrically unknowable, must stay **unknown**.
2. *Low texture* — unknown, but plausibly interpolatable from the surrounding surface.
3. *Filtered-out artifact* — must stay **unknown**.

Hole-fillers that do not distinguish these invent geometry. **In a costmap the safe mapping is hole → unknown, never → free and never → occupied.**

### 5.7 Frames

REP-103 `[VENDOR]`: body frames are **x forward, y left, z up**; camera `_optical` frames are **z forward, x right, y down**. Every formula in this section produces optical-frame coordinates. Publishing an optical-convention cloud under a body-convention `frame_id` is the single most common "why is my cloud rotated 90°" bug.

Also: ROS `image_geometry`'s `model.fx()` returns `P(0,0)`, **not** `K(0,0)`. `depth_image_proc` therefore assumes a rectified depth image and rectified intrinsics. A driver that fills `K` correctly but leaves `P` zeroed produces an all-NaN cloud. And **baseline is recovered as `B = -P_right[3] / P_right[0]`** — that is where a ROS stereo cloud gets its scale, and it is exactly the field that was zero in this repo's prior D455 bug (§1.2).

---


*Continued in [visual_to_pointcloud_part2.md](/docs/research/visual_to_pointcloud_part2.md) — depth quality control, D455 specifics, and the recommended pipeline.*
