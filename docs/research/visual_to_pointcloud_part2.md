# Visual input to point cloud on a D455 — part 2: quality control, hardware, recommendation

*Part 1 — verified hardware ground truth, the error curve, stereo/mono depth estimation and deprojection — is in [visual_to_pointcloud.md](/docs/research/visual_to_pointcloud.md).*

## 6. Depth quality control and artifact removal

> This is the section that determines whether your point cloud is usable. Everything above produces depth; this section decides which depths you are allowed to believe.

### 6.1 Flying pixels / edge fattening — the #1 artifact

**Mechanism.** The canonical reference is Hirschmüller, Innocent & Garibaldi, *Real-Time Correlation-Based Stereo Vision with Reduced Border Errors*, IJCV 47(1-3):229–246, 2002.

A correlation window embodies an implicit **constant-disparity (fronto-parallel) assumption**. When the window straddles a depth edge, that assumption is violated: part of the window belongs to the foreground disparity and part to the background. The aggregated cost is dominated by whichever sub-region carries the **stronger horizontal texture** — and at an object boundary the strongest horizontal gradient *is the boundary itself*, which moves with the **foreground** disparity. Hence a systematic bias toward foreground disparity even when the window centre sits on the background. This is **foreground fattening**.

In the point cloud these mis-assigned border pixels deproject to positions **between** foreground and background — "flying pixels", "mixed pixels", "veiling points", "split pixels", "ghost points".

Two distinct physical causes produce the same 3D artifact:
- **Passive stereo:** cost aggregation averaging across an edge, *plus* half-occlusion (background pixels visible in only one view have no correct match at all).
- **ToF / lidar:** a single pixel integrates returns from foreground and background; the demodulated phase is a *linear mixture*, giving "confident yet incorrect depth estimation in space — floating between two objects."

**Why this is the worst possible failure location for a robot:** flying pixels manufacture *a ramp of phantom points connecting an obstacle to the free space in front of it*. They concentrate on:
- Left/right silhouettes of foreground objects (band width = foreground−background disparity difference)
- Thin structures (poles, cables, chair legs, table edges) — often *deleted* rather than fattened
- Any region adjacent to a large disparity jump

**Classic remedies (why the D4 ASIC looks the way it does):**

| Method | Mechanism |
|---|---|
| Kanade & Okutomi, TPAMI 1994 — adaptive window | Model disparity uncertainty as a function of intensity variance *and* disparity variance in the window; search over window size **and shape** to minimize predicted uncertainty. Purely local, no global optimization, yet keeps sharp edges. |
| Yoon & Kweon, TPAMI 2006 — adaptive support weights | Keep a large fixed window, weight each pixel by `w = exp(−Δcolor/γ_c)·exp(−Δdist/γ_p)`. Pixels across the depth edge usually differ in colour → near-zero weight → they stop contributing foreground evidence. Typical γ_c ≈ 5–15 (Lab), window 33×33. **Fails exactly where fore/background share colour**, and costs O(W²) per hypothesis. |
| Hirschmüller 2002 — multiple supporting windows + border correction | Aggregate only the *best-scoring subset* of sub-windows, then run a dedicated border-correction pass at detected discontinuities. |
| Census / rank costs | Rank-order statistics rather than averages — reduces (does not remove) fattening, and is maximally robust to radiometric differences (Hirschmüller & Scharstein, TPAMI 2009). **This is why the D4 uses a 9×9 census.** |
| SGM | Replaces window aggregation with 1-D path aggregation + P1/P2. Removes *window* fattening but introduces **streaking** and still smears edges via P2. |

### 6.2 Detecting and removing flying pixels

**(a) The angle / incidence test — the correct geometric tool.**

A flying pixel lies on a surface patch whose local normal is nearly perpendicular to the viewing ray — a sliver stretched *along* the ray. Test the angle between viewing ray `r̂` and local normal `n̂`; discard when `|n̂·r̂|` is small.

`pcl::ShadowPoints` is exactly this filter — *"removes the ghost points appearing on edge discontinuities."* Implementation computes `val = |n·p|` and keeps if `val >= threshold_`, **default `threshold_ = 0.1`**.

> **Gotcha:** PCL uses the *unnormalized* point vector, so the effective test is range-dependent unless you normalize `p` yourself. If you normalize, `threshold = 0.10` ⇔ incidence angle **84.3°**; `0.17` ⇔ 80°; `0.26` ⇔ 75°.

Independent corroboration of the 80–85° band comes from terrestrial laser scanning, where points with incidence angle > 80° are given zero weight and 85° is used as a hard cutoff `[INDEP]`.

The ROS 2D analogue is `laser_filters::ScanShadowsFilter`, which computes the angle at P1 in triangle (Origin, P1, P2) via `atan2(r2·sin Δθ, r1 − r2·cos Δθ)`. Typical config: `min_angle: 10°`, `max_angle: 170°`, `window: 1`, `neighbors: 1`.

**Requires a normal field** — so run `IntegralImageNormalEstimation` on the *organized* cloud first, with a **small** normal radius (3–5 px), since normal estimation is itself window-based and will smear at edges.

**(b) Disparity-space discontinuity masking — cheapest, do it in the image.**

Operate on the organized disparity image *before* deprojection:
- Drop pixel `p` if `max over 4/8-neighbourhood |d(p) − d(q)| > τ_d`.
- Because disparity noise is roughly **constant in disparity space**, a constant `τ_d` (1–2 px) is range-appropriate. The equivalent depth-space threshold would have to scale as `Z²`. **This is the whole argument for working in disparity space.**
- Then **dilate the invalid mask by the aggregation-window radius** (3–5 px for a 9×9 census window). This removes exactly the band where the constant-disparity assumption was violated.

**(c) What does NOT work: statistical outlier removal.**

`pcl::StatisticalOutlierRemoval` (canonical `setMeanK(50)`, `setStddevMulThresh(1.0)`) and `RadiusOutlierRemoval` are the tools everyone reaches for, and they are **the wrong tool for flying pixels**:

1. **Flying pixels are not isolated.** They form a *continuous ribbon* along every silhouette with locally normal density, so they pass SOR/ROR trivially. SOR removes salt-and-pepper speckle, not edge artifacts.
2. **Density is range-dependent.** Points-per-solid-angle is constant, so metric neighbour spacing grows linearly with `Z` (and quantization spacing as `Z²`). A single global μ+kσ threshold therefore **deletes far-field valid geometry while keeping near-field noise.** This is exactly the DSOR critique ([arXiv:2109.07078](https://ar5iv.labs.arxiv.org/html/2109.07078)), whose fix is a range-scaled threshold `T_d = T_g · r · range`.
3. **Cost:** O(n log n) with a kd-tree rebuild per frame — often the dominant cost in the pipeline at 848×480×30 Hz.
4. **Non-idempotent:** μ and σ are computed over the current cloud, so behaviour changes as scene content changes.

Use SOR/ROR **only as a mop-up after** the geometric filters, and prefer ROR (fixed metric threshold, reproducible) over SOR.

**(d) Bilateral / guided filtering — with a critical caveat.**

A bilateral filter on depth with range kernel `σ_r` will happily average across a depth edge whenever the jump is `< ~2σ_r` — **creating flying pixels rather than removing them.** Two mitigations: run it in **disparity space** (noise-linear kernel), and **mask out edge pixels first** and only filter the interior.

OpenCV's `cv::ximgproc::DisparityWLSFilter` is the practical version: `lambda = 8000`, `sigmaColor ≈ 0.8–2.0` (demo uses 1.5). It supports a confidence map from a left-right consistency check — *"filtering with confidence requires two disparity maps and is approximately two times slower, however quality is typically significantly better."*

### 6.3 Confidence and uncertainty

**Left-right consistency (LRC).** Invalidate `p` if `|d_L(x) − d_R(x − d_L(x))| > τ`, with **τ = 1 px** near-universal.

- OpenCV exposes it as `disp12MaxDiff`, and **the default is disabled (−1)** `[VENDOR]`. Set it to 1 for robotics.
- **Catches:** half-occlusions, gross mismatches, most repeated-texture failures where the two views latch onto different repeats.
- **Misses:**
  - **Correlated errors** — if both directions latch onto the *same* wrong match (periodic structure at exactly the aliasing disparity; low-texture regions where SGM propagates the same wrong plane both ways), LRC passes with zero error. **LRC is a consistency check, not a correctness check.**
  - **Small systematic errors** — 1 px fattening, subpixel bias, rectification error are all self-consistent.
  - It cannot *fix* what it detects: *"the main cause of errors in occluded regions is the absence of information."*
- **Cost:** a second full disparity pass (~2×), unless approximated.
- **LRD (Left-Right Difference)** is the stronger cousin, combining margin with consistency, and consistently outranks plain LRC.

**The confidence-measure taxonomy.** Hu & Mordohai, *A Quantitative Evaluation of Confidence Measures for Stereo Vision*, TPAMI 34(11), 2012 evaluated 17 measures in 6 categories. **Headline finding: LRD, DSM and PKRN outperform the naive matching-score measure** — margin/ratio and consistency cues beat absolute cost.

The current reference survey is Poggi et al., *On the Confidence of Stereo Matching in a Deep-Learning Era*, TPAMI 44(9):5293–5313, 2022 ([arXiv:2101.00431](https://arxiv.org/abs/2101.00431)). Key measures (with `c_d1` = min cost, `c_d2` = second min, `c_d2m` = second *local* min):

| Measure | Definition | Where you already have it |
|---|---|---|
| MSM | `−c_d1` | — |
| MM / MMN (max margin) | `c_d2m − c_d1` / `c_d2 − c_d1` | — |
| **PKR / PKRN** (peak ratio) | `c_d2m / c_d1` / `c_d2 / c_d1` | **OpenCV `uniquenessRatio`** (percent form), typical **5–15**; ROS default **15.0** |
| CUR (curvature) | `−2c_d1 + c_{d1−1} + c_{d1+1}` | — |
| NEM (negative entropy) | `−Σ p_i log p_i`, softmin of costs | — |
| **LRC / LRD** | see above | **OpenCV `disp12MaxDiff`**; **D4 `lrAgreeThreshold`** |
| DMV | `|∇d|` | trivially computable post-hoc |

**The D455 gives you a binary mask, not graded confidence.** Intel's own words: *"There is no confidence map provided, data is either returned or marked zero to indicate that it is invalid."* The ASIC internally applies LR threshold, texture threshold, neighbour threshold, second-peak threshold, and median threshold, and zeroes any pixel failing any of them — but you only see the union. Compare Luxonis DepthAI, which exposes a graded 0–255 confidence, and NVIDIA ESS, which outputs a confidence channel.

> **Practical consequence for a D455:** since you cannot get graded confidence out of the ASIC, your confidence signal must be **reconstructed post-hoc** from the disparity map alone: local disparity variance, distance-to-discontinuity, distance-to-invalid, and temporal stability. This is exactly the input space of **CCNN** (Poggi & Mattoccia, BMVC 2016), which regresses confidence **from the disparity map only**, *"making it suitable for depth devices that don't expose cost volume cues."* That is the correct learned option here if you want graded confidence on a D455.

**Learned confidence and uncertainty.**
- **CCNN** — disparity-map-only CNN, 9×9 patches, trains in ~15 min on 20 KITTI images.
- **LGC-Net** (ECCV 2018) — fuses a local patch net with a global U-Net; the global branch is what lets it reason about large low-texture regions that local patches cannot.
- **SEDNet** (CVPR 2023) — adds a KL term forcing predicted uncertainty to match the *empirical disparity-error distribution* via a differentiable soft-histogram. **This is the right choice if you need calibrated metric uncertainty** (e.g. to set a physical threshold like "σ_z < 5 cm").
- **Poggi et al., CVPR 2020** taxonomy for depth uncertainty: `Post` (flip-consistency, free, no retraining), `Drop` (MC-dropout, consistently the *weakest*), `Boot`/`Snap` (ensembles, strongest but N× cost), `Self` (self-teaching, most of the benefit at 1× inference cost). Metrics: **AUSE** (area under sparsification error, lower better) and **AURG**.
- **The open problem is cross-domain generalization** — a confidence net trained on KITTI can be *worse than PKRN* on an indoor robot.

**Turning confidence into a mask — the density/accuracy ROC.**

Sort pixels by descending confidence, sweep a threshold, and at each density level compute the error rate. The optimal achievable AUC, given a dense-map error rate ε, is:

```
  AUC_opt = ε + (1−ε)·ln(1−ε)
```

Pick your operating point **from this curve**, not from a threshold that "looks nice". Two things to internalize:
- The ROC is **not spatially uniform**. Dropping the least-confident 30% removes almost exclusively (a) occlusion bands, (b) low-texture regions, (c) sky/far field. You lose *whole regions*, not a uniform thinning. Verify the surviving density still gives you several hits per voxel.
- Prefer a **two-tier** output over a binary keep/drop: high-confidence → occupied evidence; low-confidence → **unknown** (neither occupied nor cleared). For navigation this is almost always better.

### 6.4 Speckle filtering

`cv::filterSpeckles(img, newVal, maxSpeckleSize, maxDiff, buf)` does connected-component labelling where 4-neighbours join iff `|d1−d2| ≤ maxDiff`, then zeroes any component smaller than `maxSpeckleSize`. **Both parameters must be scaled by 16 for OpenCV fixed-point disparity.**

| Parameter | Typical | Source |
|---|---|---|
| `speckleWindowSize` | 50–200 | `[VENDOR]` OpenCV |
| `speckleRange` | 1–2 (implicitly ×16) | `[VENDOR]` OpenCV |
| ROS `stereo_image_proc` | `speckle_size = 100`, `speckle_range = 4` | `[VENDOR]` |

**Caution — speckle filtering is a size test, so it deletes genuine small/thin obstacles exactly as readily as noise.** A 30 cm pole at 5 m subtends only a handful of pixels. At 848×480, `speckleWindowSize` above ~200 is already dangerous.

Nav2's `DenoiseLayer` (`minimal_group_size`, `group_connectivity_type`) is the same connected-component idea one representation later — useful as a second line of defence when you cannot retune the stereo box.

### 6.5 Temporal filtering — and why it is dangerous on a moving robot

**Filter in disparity space, not depth space.** Intel's whitepaper states it directly `[VENDOR]`: stereo depth noise increases as the square of distance, so processing in disparity space (1/distance) *normalizes* the relationship and allows consistent α/δ across range.

Three independent reasons this is *correct*, not merely convenient:
1. The measurement **is** disparity, and its error is stationary and roughly Gaussian in disparity (σ_d ≈ 0.1 px for a well-calibrated system).
2. Depth error is the *nonlinear pushforward* `|ε_Z| = (Z²/(f·B))·|ε_d|`, so filtering depth with fixed α under-filters the far field and over-filters the near field.
3. Depth is a **convex** function of disparity (1/d), so `E[Z] ≠ 1/E[d]`: naively averaging depth is **biased outward** by Jensen's inequality, while averaging disparity and then inverting is unbiased to first order.

**Intel's default parameters** `[VENDOR]`:

| Filter | Parameters |
|---|---|
| Temporal | α = 0.4–0.5, δ = 20 (in **1/32 disparity** units), persistency index 0–8 |
| Spatial (domain-transform, edge-preserving) | α = 0.5–0.6, δ = 8–20, magnitude (iterations) 2, `holes_fill` 0–5 |
| Decimation | magnitude 2 → 4× less downstream compute; 4 → 16×. **Non-zero median** for factor <3, **non-zero mean** for ≥4 |

The temporal δ is an **edge-stopping** term: if the new sample differs from the history by more than δ, the filter **resets** rather than blends. So it is an IIR *bilateral-in-time* filter, not a plain EMA — that is what stops a moving edge from being smeared into an average.

**The motion hazard.** Intel's own documentation notes that the persistence filter has a **directional bias**: when objects move, persistence *"persists the object into occlusion regions"* for one direction of motion and *"persists the background"* for the other. Concretely, with the camera panning:
- On one side it smears the **foreground obstacle into newly-revealed space the robot is about to enter** (false positive — robot freezes).
- On the other side it leaves **stale free space where an obstacle now is** (false negative — collision). **This is the one that kills you.**

Without ego-motion warping, a fixed-α EMA on a robot translating at `v` introduces a depth lag of roughly `v·Δt·α/(1−α)` — at α=0.5, 30 Hz, 1 m/s that is ~3 cm of systematic *late* obstacle reporting per time constant, and far worse under rotation where pixel flow is large and range-independent.

**Guidance:**
- The correct fix is **ego-motion compensation**: forward-warp the previous depth map into the current frame using the platform's ego-motion estimate, then fuse.
- If you cannot warp: keep α **high** (≥0.7, i.e. weak filtering), keep δ tight so real motion resets history, and set **persistency 0–1, never 8**, whenever the base is moving.
- **Gate on motion:** disable temporal filtering when `|v|` or `|ω|` exceeds a threshold; re-enable when stationary (where it is genuinely valuable).
- **Prefer pushing temporal integration downstream into the occupancy map**, where ego-motion is already handled and raytrace-clearing gives principled decay.
- **Beware double filtering:** a temporally filtered depth image feeding a costmap with its own decay produces an effective time constant that is the *product*, and stale-obstacle behaviour nobody tuned.

### 6.6 Subpixel interpolation and pixel-locking

**Estimators.** Parabola fit `d̂ = d1 + (c_{d1−1} − c_{d1+1}) / (2(c_{d1−1} − 2c_{d1} + c_{d1+1}))` is ubiquitous because it is 3 loads and a divide. Equiangular/triangular fitting is more appropriate for SAD costs. Shimizu & Okutomi (IJCV 63(3), 2005) showed the bias has a **known functional form in the fractional part of the true disparity** and can be largely cancelled by a second fit at half-pixel-shifted locations (the two biases have opposite sign).

**The artifact: pixel locking.** The subpixel estimate is biased toward integer (and half-integer) disparities, giving artificially-peaked histograms of fractional disparity.

- **Diagnostic:** histogram the fractional part of your disparity map over a smoothly slanted surface. Flat histogram = correct; sharp spikes at 0.0 (and 0.5) = locked.
- **3D consequence:** *"erroneous ripples or waves in 3D reconstruction of flat surfaces"* — Stein, Huertas & Matthies, *Attenuating Stereo Pixel-locking via Affine Window Adaptation*, ICRA 2006. This is a JPL robotics paper *specifically because* the ripples corrupt ground-plane fitting and traversability estimation.
- **Root cause:** a rectangular window on a slanted surface samples a *range* of true disparities; the aggregated cost curve is asymmetric unless the surface is fronto-parallel; fitting a symmetric parabola to an asymmetric curve pulls the estimate toward the integer sample. **Pixel-locking and foreground fattening are two symptoms of the same fronto-parallel window assumption.**
- **Ranking:** *"the parabola-based interpolation formula shows significantly worse performance regarding the pixel-locking effect, indicating that parabola cost interpolation should be avoided"*; equiangular or Shimizu-Okutomi cancellation *"can always replace their parabola counterpart with minimal performance loss and no overhead."*

**Quantization shells ("onion rings").** Disparity is quantized to `2^−b` px, so reconstructed depths lie on discrete shells spaced `ΔZ = Z²/(f·B·2^b)` — spacing growing **quadratically** with range. A continuous slanted floor reconstructs as a staircase of concentric range arcs, and normals computed on such a cloud are garbage.

For the D455 (b=5, i.e. 1/32 px) the shell spacing is 3.1 mm at 2 m, 27.5 mm at 6 m, 305 mm at 20 m `[MEASURED]` — always **2.5–4× below** the matching noise floor, so **the D455 does not suffer from visible quantization shells within its useful range.** (Contrast a 0-subpixel device: Luxonis documents 0–95 integer disparity giving only **96 unique depth values** and visible layering.)

Rule of thumb: set `b` so `ΔZ` at max working range is *comparable to*, not much smaller than, `σ_Z`. With σ_d = 0.1 px you gain nothing beyond b = 4. Beyond that you are resolving noise.

### 6.7 The false-positive / false-negative asymmetry

The costs are **not** symmetric:
- **False negative** (missed obstacle) → collision. Unbounded, possibly unrecoverable.
- **False positive** (phantom obstacle) → detour, or if it lands in a doorway, the robot **freezes**. Bounded per event — but frequent false positives destroy the mission *and* train operators to raise clearing aggressiveness, which reintroduces false negatives.

**But flying pixels break the naive "just bias toward FP" reasoning**, and this is the key insight:

1. Flying pixels at a silhouette produce points **in front of** the true obstacle. As occupied evidence they are *conservative* — harmless.
2. But they also occur at **concave edges and the ground/obstacle junction**, producing points *below* the ground plane or floating above it. These become spurious negative obstacles or phantom obstacles in otherwise-clear floor — the classic "stereo robot refuses to cross a floor seam."
3. **Aggressive flying-pixel removal by dilating the discontinuity mask deletes the entire silhouette of every obstacle** — including thin obstacles whose *entire extent* is silhouette. A 2 cm cable, a chair leg, or a table edge can be 100% edge pixels. **Blanket edge dilation converts thin obstacles into false negatives.**

> **This argues decisively for the angle/normal test (§6.2a) over blanket edge dilation**, because the angle test keeps a thin obstacle's *front face* (normal pointing toward the camera) while removing only the ray-aligned sliver.

Similarly, speckle filtering with large windows and SOR with aggressive `std_ratio` both preferentially delete **small, distant** objects — buying cosmetic cloud quality with false negatives at exactly the range where you need early detection.

**Evaluation practice:**
- Do **not** report a single "bad-pixel %". Report the **ROC/sparsification curve** so density and accuracy are visible together, and separately report **error rate restricted to depth-discontinuity neighbourhoods** (Middlebury's "disc" region metric is the precedent) — that is where the navigation-relevant errors live.
- Score at the **costmap level, not the point level**: per-cell FP rate and per-obstacle detection rate as a function of obstacle size and range. A cloud with 2% bad points can be perfectly safe or lethal depending on *where* the 2% is.
- Include a **thin-obstacle recall test** (pole / cable / chair-leg at 1, 3, 5 m) in every filter-tuning sweep, and re-run it after any change to speckle size, mask dilation, or SOR aggressiveness.
- Keep a **three-state map** (occupied / free / unknown) so filtered-out pixels degrade to *unknown* rather than *free*.
- Model point uncertainty **anisotropically** when fusing — 53:1 ratio at 5 m for the D455 (§2.4).

---

## 7. RealSense D455 specifics

### 7.1 IR emitter — when on, when off

The projector is a VCSEL emitting a **static semi-random dot pattern**, 850 nm ±10 nm, **360 mW max average optical power**, Class 1, field of projection 90°×63° `[VENDOR]`. Default `controls-laserpower: 150` mW.

**Turn it ON when:** indoors, textureless surfaces (white walls, blank floors, glossy tabletops), dim light. The entire purpose is to add artificial texture where stereo has no signal.

**Turn it OFF when:**

| Situation | Why |
|---|---|
| **Outdoors / bright sunlight** | 360 mW against ~100,000 lux is nil contrast. Intel: *"both the D415 and D435 tend to perform even better in bright light… Sunlight reduces the sensor noise and tends to bring out the texture."* The emitter is irrelevant-to-useless outdoors. |
| **Well-textured scenes** | Laser **speckle actively hurts**: subpixel goes 0.03–0.05 (off) → 0.07–0.11 (on). Intel: *"~30% smaller Subpixel with the laser turned off."* Referring to §2.2's tables, that is the difference between 44 mm and 70 mm RMS at 6 m. |
| **Specular surfaces at close range** | Saturated dot blooms. Reduce laser power if you see localized saturation. |
| **You need clean left-IR for VO/SLAM** | The dots pollute the image. See below. |

**Effective range of the projector is the key limitation.** Intel's own framing: the question "how do I increase the range?" is *always* about seeing a textureless white wall indoors beyond ~5 m. And the physics is unkind: **doubling power (150 → 360 mW) buys only √2 = 1.41× range**. Adding an **IR bandpass filter** in front of the L/R imagers (i.e. the D455f variant) raises white-wall range from **~3 m to ~10 m** — far more effective than cranking power `[VENDOR]`.

> **Practical envelope: the emitter meaningfully contributes to roughly 3 m stock, ~5 m at full 360 mW, and is essentially a no-op beyond that or outdoors.** `[INFERRED from VENDOR]`

**The "dots pollute my IR for SLAM" problem.** Intel notes the left IR image is *pixel-perfect aligned with the depth map, perfectly time-synced, needs zero alignment compute, and creates no occlusion artifacts* — the only downside is the dots. Options:

1. **`RS2_OPTION_EMITTER_ON_OFF`** — the camera alternates emitter state every frame; tag frames via `RS2_FRAME_METADATA_FRAME_EMITTER_MODE`. Depth from the on-frames, VO from the off-frames, at half effective rate. **Known problems:** not deterministic and can fail to toggle ([#9450](https://github.com/IntelRealSense/librealsense/issues/9450), [#3407](https://github.com/IntelRealSense/librealsense/issues/3407)); **conflicts with auto-exposure** — setting it while AE is on can freeze exposure ([#11908](https://github.com/IntelRealSense/librealsense/issues/11908)), so use manual exposure with it; the metadata polarity has been reported **inverted** on Jetson/UVC ([#6191](https://github.com/IntelRealSense/librealsense/issues/6191)) — verify on your platform.
2. **Use the RGB stream for VO instead.** ← **Best answer for a D455 specifically.** Unlike the D435, the D455's colour sensor is a global-shutter OV9782 **with an IR-cut filter**, so it never sees the dots, and its FOV (89.78°) matches depth (88.97°) `[MEASURED]`. This sidesteps the whole `emitter_on_off` minefield.
3. Emitter fully off if the scene has natural texture (typical outdoors / cluttered indoors).
4. An *external* continuous projector placed off-axis, or one flickering >50 kHz.

Note: the D415-only "Remove IR Pattern" preset does **not** apply — it requires the D415's colour depth imagers; the D455's L/R imagers are mono.

### 7.2 Visual presets

Intel's official position is that they will not document individual parameters (*"we currently use machine learning to globally optimize for different usages"*), but the values are in `src/ds/advanced_mode/presets.cpp`. The ones that matter:

| Parameter | Default | **HighAccuracy** | **HighDensity** | Effect |
|---|---|---|---|---|
| `deepSeaSecondPeakThreshold` | 325 | **647** | 222 | best-vs-second-best margin — rejects ambiguous/repetitive matches |
| `lrAgreeThreshold` | 24 | **10** | 18 | left-right consistency tightness |
| `deepSeaNeighborThreshold` | 7 | **108** | 12 | how many valid neighbours required |
| `deepSeaMedianThreshold` | 500 | 796 | 789 | median deviation |
| `textureDifferenceThreshold` | 0 | 1722 | 2466 | minimum local texture to accept |
| `sloK1Penalty / sloK2Penalty` | 60 / 342 | 155 / 190 | 55 / 235 | SGM P1/P2 |

Reading it: **HighAccuracy tightens every gate — second-peak margin 325→647, LR consistency 24→10, neighbour requirement 7→108.** Every knob trades fill rate for confidence.

**Use `RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY` for a mobile robot.** Intel says so explicitly and memorably `[VENDOR]`:

> *"very good for autonomous robots where false depth, aka. Hallucinations, are much worse than no depth… imagine that you are making a robot that needs to walk across lava."*

The wiki also lists HighAccuracy under "collision avoidance". The **stated counter-risk** is that it can be conservative enough to miss a real obstacle — so pair it with a costmap that treats unknown as **unknown**, not as free (§6.7), and with the thin-obstacle recall test.

Also settable from the same JSON: **`aux-param-depthclampmin/max`** — clamp depth to your real operating envelope *in the ASIC*. Free, and kills far-field garbage before it ever becomes a point.

### 7.3 Post-processing filter chain and its order

Intel's stated order `[VENDOR]`:

```
Depth Frame → Decimation → Depth2Disparity → Spatial → Temporal → Disparity2Depth → Hole Filling → Filtered Depth
```

Two nuances people get wrong: **decimation goes *before* the disparity transform**, and **hole filling goes *after* disparity→depth**. And **alignment goes last** — filtering an already-aligned depth map introduces aliasing/jaggies.

**Which filters fabricate geometry — and must be avoided for robot safety:**

| Filter | Verdict |
|---|---|
| **Hole Filling filter** (discrete block) | **Do not use.** Pure invention. Note the **default mode is 1 = "farthest from sensor"**, which fills a hole in front of an obstacle with *free space*. If you must fill, mode 2 ("nearest") at least errs conservatively. |
| **Spatial filter `holes_fill` > 0** | **Set to 0.** Same problem, hidden inside a filter you wanted only for smoothing. |
| **Temporal `persistency` 7–8** | **Never on a moving robot.** Resurrects removed obstacles and smears moving ones (§6.5). 1–3 is a defensible compromise; 8 holds the last valid value *indefinitely*. |
| **Decimation with mean** (magnitude ≥4) | Averages across depth discontinuities — **actively manufactures flying pixels.** Prefer magnitude 2 (which uses non-zero *median*). |

**Safe chain for a robot:**
```
decimation(2) → depth2disparity → spatial(mag 2, α 0.5, δ 20, holes_fill=0)
              → temporal(α 0.4, δ 20, persist ≤3) → disparity2depth
```
and stop. **Holes stay holes — that is the entire point of HighAccuracy.**

### 7.4 Calibration

| | On-Chip / Tare Self-Calibration | OEM Calibration |
|---|---|---|
| Runs on | D4 ASIC (zero host CPU) | Host PC |
| Target | On-chip: **none**. Tare: ground truth distance | V-shape OEM target (purchased) |
| Time | <30 s (on-chip as fast as **0.6 s**) | ~30 s |
| Fixes | Intrinsics, **depth only** | Intrinsics + extrinsics, depth **and** RGB |

- **On-Chip Calibration** minimizes depth *noise* (fixes rectification/extrinsic drift). Corrects the two real degradation modes: microscopic bending of the sensor stiffener, and small lens shifts, from thermal cycling / shock / vibration. Works on any textured scene, projector on or off, indoors or in sunlight; best with texture over the central 20% of FOV.
- **Tare Calibration** fixes absolute depth *accuracy* (scale/offset). Requires the true perpendicular distance **from the left camera's origin** — remember the **−4.55 mm** depth start point offset behind the front cover glass when measuring.
- **Health check thresholds** `[VENDOR]`: `|health| ≤ 0.25` → do nothing; `0.25–0.75` → accept changes then re-run; `≥ 0.75` → needs full OEM calibration.
- **When to recalibrate:** point at a flat textured target with the projector **off** and measure plane-fit RMS with the Depth Quality Tool. Healthy: **subpixel RMS < 0.1**. **> ~0.2 px over the central ROI → recalibrate.** Cross-check §2.2: 0.2 px is exactly the datasheet spec limit, so a camera at 0.2 is performing at the bottom of its acceptance band.
- **D455-specific:** run on-chip calibration with the **thermal loop off** (the Viewer and DQT do this automatically).
- For a robot that vibrates and thermal-cycles daily, budget a periodic on-chip self-cal — it's ~0.6 s and needs no target, so it can be triggered opportunistically.

### 7.5 Other D455 gotchas

- **Exposure is the #1 cause of bad depth** `[VENDOR]`. Keep **GAIN = 16 (minimum)** and tune **exposure** instead. Raising gain "may look better" in the IR image but degrades depth. Over-exposure is as bad as under-exposure.
- **Outdoor AE:** shrink the auto-exposure **ROI** (e.g. to the lower half of the frame) so RGB AE chasing the sky doesn't destroy your texture.
- **Left invalid depth band:** the left imager is the reference, so there is a strip of no-depth at the left edge of width `B·f/Z` px. At 848×480, Z=1 m: `94.862·431.657/1000 ≈ 41 px`; at 0.5 m ≈ **82 px** `[MEASURED, derived]`. Crop it or your cloud has a systematic left-edge void.
- **Repetitive structures** (fences, wire grids, tiled floors, brick) alias badly and can defeat LR consistency (§6.3). Mitigations: raise `DSSecondPeakThreshold`, and/or **tilt the camera 20–30° from horizontal**.
- **Thermal drift is real and measurable:** a D455 pointed at a fixed floor drifted **1700 mm → 1715 mm over 12 h** (~0.9%) `[INDEP]`. Depth quality starts degrading above ~42 °C ASIC temperature. The D45x family has **`RS2_OPTION_THERMAL_COMPENSATION`** — leave it **enabled** for steady-state operation, disable only during on-chip calibration. Monitor `RS2_OPTION_ASIC_TEMPERATURE`; gate any absolute-accuracy work on it plateauing.
- **Fixed focus** at 50 cm to infinity — goes out of focus well below 20 cm (irrelevant given MinZ ≈ 33 cm).
- **USB:** real-world USB3 throughput is ~3200–3600 Mbps against a 5000 theoretical. Bulk transfer has **no QoS guarantee** — a CPU spike elsewhere fills the buffer and drops frames. Expect ≤5% drops on a healthy single-camera setup. **Always assert `USB_TYPE_DESCRIPTOR == "3.2"` at startup** `[MEASURED — this unit reports 3.2]`; silent USB2 fallback from a bad cable caps you around 10 fps.
- **RGB↔depth hardware sync only works when all streams run at the same frame rate.** Mixing depth@30 with RGB@15 loses HW sync.
- **HDR:** `RS2_OPTION_HDR_ENABLED` alternates exposures per frame — worth trying for scenes with both sunlit and shadowed regions.

### 7.6 Timestamps and multi-camera sync

**Timestamp domains** — always call `rs2_get_frame_timestamp_domain()` and assert; do not assume:

| Domain | Meaning |
|---|---|
| `HARDWARE_CLOCK` | raw camera clock — monotonic, sub-ms, but drifts against host time |
| `SYSTEM_TIME` | host clock at arrival — the silent fallback when metadata is unavailable (**common on Linux without kernel patches**) |
| **`GLOBAL_TIME`** | hardware clock continuously regressed onto host clock — **this is what you want** for fusing with wheel odometry / lidar. Enable via `RS2_OPTION_GLOBAL_TIME_ENABLED` |

Also: `RS2_FRAME_METADATA_FRAME_TIMESTAMP` is the device clock at **readout start**, while **`RS2_FRAME_METADATA_SENSOR_TIMESTAMP` is the middle of exposure**. For a moving robot, mid-exposure is the physically correct stamp for motion compensation; the difference is exposure/2 and can be tens of ms indoors.

**Hardware sync** via `RS2_OPTION_INTER_CAM_SYNC_MODE` and the 9-pin JST header (pin 5 = SYNC, pin 9 = GND). Firmware-gated: FW ≥5.9.15.1 gives Default/Master/Slave; FW ≥5.12.4.0 adds Full Slave and genlock modes 4–258. Two D455-specific notes:
- **On the D455 both depth AND RGB can be triggered externally** (on the D435 only depth can) — a genuine advantage for RGB-D rigs.
- **D400 cameras do not cross-talk** — overlapping projector patterns just add texture. You sync for *temporal* alignment, not to avoid interference.
- Multi-camera clouds: treat each camera independently with its own capture/processing thread and combine **after** computing their point clouds.

---

## 8. Summary table — depth source options for a mobile robot

| Approach | Metric? | Accuracy | Latency (target HW) | Compute | Zero-shot robustness | Verdict for a D455 robot |
|---|---|---|---|---|---|---|
| **D4 ASIC on-chip stereo** | ✅ | 0.5–2% of Z to 6 m `[MEASURED]` | **0 ms host**, 30–90 fps | **zero CPU/GPU** | good (census+SGM, well-tuned) | **Primary source.** Nothing else is free. |
| OpenCV SGBM on host (raw IR) | ✅ | KITTI D1-all ~10.9% `[INDEP]` | 22 ms @848×480 D=128, `SGBM_3WAY` only `[INDEP]` | 1 CPU core+ | poor | Only if you need cost-volume access the ASIC won't give you |
| ELAS | ✅ | KITTI D1-all 9.7% | ~0.4 s/MP single core | CPU | poor | Scales better at high res; still too slow |
| libSGM / VPI SGM (CUDA/OFA) | ✅ | ≈SGM | 110 fps Xavier MaxN @1024×440 `[VENDOR]` | GPU / OFA block | poor | Good if you want disparity on GPU with cost volume; VPI OFA is free of GPU load on Orin |
| **Isaac ROS ESS** | ✅ | Middlebury BP2 8.27, MAE 1.06 px | **~79 fps AGX Orin** @576×960 `[INDEP]` | GPU (TRT) | moderate | **Best real-time learned option**; *outputs confidence*, which the D455 does not |
| Fast-FoundationStereo | ✅ | MB BP2 2.20 zero-shot | **~26 fps AGX Orin** `[PAPER]` | GPU (TRT), 12.4 M | **excellent** | Best accuracy/latency point if you have GPU budget |
| FoundationStereo | ✅ | MB BP2 1.1 — best in field | **1.0 fps AGX Orin**; **OOM on 8 GB** `[VENDOR]` | 12,824 GMACs | **best** | **Not deployable** for navigation. Offline/ground-truth use only |
| IGEV / Selective-IGEV | ✅ | KITTI D1 1.55–1.59 | 0.18–0.24 s on desktop GPU | GPU | good | Cut iterations 32→4 for ~17% EPE cost |
| RAFT-Stereo | ✅ | KITTI D1 1.82 | 0.38 s | GPU, 530 MB | good | Degrades badly below 8 iterations — prefer IGEV family |
| **Depth Anything V2/V3** | ❌ relative | best-in-class *structure* | ~20 fps Orin AGX (ViT-S, TRT) `[INDEP]` | GPU | excellent for structure | **Prior / gap-filler only.** Needs a metric anchor |
| Metric3D v2 | ⚠ needs intrinsics | NYU AbsRel 0.063 | **unpublished** | GPU | good in-distribution | Fine if intrinsics fixed and domain stable |
| UniDepth V2 | ✅ intrinsics-free | agg. zero-shot δ₁ 60.0; **best stability** | ViT-S 23 ms @0.5 MP (A6000) | GPU | best of the mono family | Best single mono model if you must pick one |
| Depth Pro | ✅ | **2–6× better boundaries**; but ETH3D δ₁ only 41.5 | 0.3 s/2.25 MP V100 `[PAPER]`; **<15 s on RTX 4050** `[INDEP]` | heavy (504 M) | inconsistent | Use only for thin-structure recovery, offline |
| Marigold / diffusion | ✅ | excellent detail | **19–378 s/image** | absurd | — | Non-starter |
| **Mono alone as obstacle source** | — | — | — | — | — | **48.3% collision rate at 1 m/s** `[INDEP]`. Never. |
| **Stereo + mono fusion** (Stereo Anywhere / BridgeDepth) | ✅ | MB bad>2 11.15 → 7.07 (−37%) | GPU-heavy | GPU | best | **Architecturally correct** direction; not yet real-time on Orin |

---

## 9. Recommended pipeline for a D455 on a mobile robot

### 9.1 The pipeline

```
┌── CAPTURE ──────────────────────────────────────────────────────────────┐
│ 848×480 @ 30–60 Hz  (reactive obstacle avoidance)                       │
│ 1280×720 @ 30 Hz    (mapping / higher-accuracy mode)                    │
│ Preset: HIGH_ACCURACY                                                   │
│ ASIC depth clamp:   min = MinZ+margin (400 mm), max = 8000 mm           │
│ Gain = 16 (pinned), exposure tuned; AE ROI restricted outdoors          │
│ Emitter: 150 mW indoors / OFF outdoors; disparity shift = 0             │
│ depth_units = 1000 µm (default);  THERMAL_COMPENSATION = on             │
│ GLOBAL_TIME_ENABLED = on; use SENSOR_TIMESTAMP (mid-exposure)           │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌── FILTER (in disparity space) ──────────────────────────────────────────┐
│ decimation(magnitude=2, non-zero median)                                │
│ depth → disparity                                                       │
│ spatial(magnitude=2, α=0.5, δ=20, holes_fill=0)     ← holes_fill MUST be 0│
│ temporal(α=0.4, δ=20, persistency ≤ 3, or DISABLED while moving)        │
│ disparity → depth                                                       │
│ NO hole-filling filter                                                  │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌── VALIDITY MASK (before deprojection, in image space) ──────────────────┐
│ 1. drop depth == 0                                                      │
│ 2. range gate: 0.40 m ≤ Z ≤ 6 m (hard) / ≤10 m (low-confidence tier)    │
│ 3. crop left invalid band (~41 px at 848×480)                           │
│ 4. disparity discontinuity mask: |Δd| > 1–2 px over 8-neighbourhood     │
│ 5. speckle filter: window 50–100 px, range 1–2                          │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌── DEPROJECT ────────────────────────────────────────────────────────────┐
│ X = (u−ppx)·Z/fx,  Y = (v−ppy)·Z/fy,  Z = Z                             │
│ intrinsics READ FROM DEVICE (fx=431.657, ppx=429.187 @848×480)          │
│ integer u,v (no +0.5) — librealsense/OpenCV convention                  │
│ no undistortion (depth coeffs are exactly zero)                         │
│ output = ORGANIZED cloud, is_dense=false, NaN for invalid               │
│ frame_id = *_depth_optical_frame  (z fwd, x right, y down)              │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌── CLOUD-SPACE CLEANUP ──────────────────────────────────────────────────┐
│ 6. IntegralImageNormalEstimation (small radius, 3–5 px, organized)      │
│ 7. incidence/shadow test: drop if |n̂·r̂| < cos(80–85°)                   │
│    (pcl::ShadowPoints, NORMALIZED p, threshold 0.10–0.17)               │
│ 8. RadiusOutlierRemoval mop-up (fixed metric radius) — NOT global SOR   │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌── DOWNSTREAM ───────────────────────────────────────────────────────────┐
│ Anisotropic σ: σ_Z = Z²·σ_d/(f·B), σ_XY = Z·σ_d/f  (53:1 at 5 m)        │
│ Three-state map: occupied / free / UNKNOWN  (never map holes to free)   │
│ Temporal integration HERE (ego-motion aware), not in the image          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 9.2 Justification of each non-obvious choice

**Use the ASIC, don't replace it.** The D4 computes census+SGM with LR consistency, second-peak, texture, neighbour, and median gates at **zero host cost**, 30–90 fps. On a Jetson, the only learned alternative that beats it *and* runs in real time is Isaac ROS ESS (~79 fps AGX Orin), and it costs your entire GPU. FoundationStereo — the accuracy winner — is **1 fps on AGX Orin and OOMs on 8 GB**. The ASIC is not a compromise; it is the correct default, and it frees the GPU for perception that only a GPU can do.

**848×480, not 640×480.** Same frame rate options, **10° more horizontal FOV** (88.97° vs 78.60°), higher resolution, and lower MinZ per unit resolution `[MEASURED]`. There is no reason to use 640×480 on a D455.

**848×480 for reactive, 1280×720 for mapping.** 1280×720 gives **34% lower depth noise** (`f·B` 61,808 vs 40,948) but caps at 30 Hz and pushes MinZ from ~33 cm to ~49 cm `[MEASURED]`. For obstacle avoidance, latency and near-field coverage beat precision. For mapping, the reverse.

**Never capture low-res to save CPU — decimate afterward.** MinZ and the noise floor are set at capture time and cannot be recovered by upsampling.

**HIGH_ACCURACY preset.** Intel designed it for exactly this case and says so in the lava metaphor. It raises the second-peak margin 325→647 and the neighbour requirement 7→108, trading fill for confidence. Combined with a three-state costmap, the resulting holes are honest "unknown" rather than dangerous "free."

**Filter in disparity space.** Three independent reasons (§6.5): disparity noise is stationary, depth noise is quadratic, and depth is a convex function of disparity so averaging depth is *biased outward* by Jensen's inequality. Intel's ordering exists for this reason, not for convenience.

**`holes_fill = 0` and no hole-filling filter.** These fabricate geometry, and the *default* hole-filling mode ("farthest from sensor") fills holes in front of obstacles with free space. For a robot this is a false negative generator.

**Temporal persistency ≤ 3, or off while moving.** Persistence has a documented **directional bias** that on one side smears an obstacle into space the robot is about to enter, and on the other leaves stale free space where an obstacle now is. Without ego-motion warping, an α=0.5 EMA at 30 Hz and 1 m/s reports obstacles ~3 cm late per time constant. Push temporal integration into the occupancy map instead, where ego-motion is already handled.

**Discontinuity mask + incidence test, in that order, both of them.** This is the core of flying-pixel removal:
- The **disparity discontinuity mask** is cheap and catches the aggregation-window band directly, but blanket dilation deletes thin obstacles entirely (a cable can be 100% edge pixels).
- The **incidence/normal test** is the complement: it keeps a thin obstacle's front face (normal toward camera) while removing the ray-aligned sliver. **This is why both are needed and why the angle test must not be replaced by more dilation.**
- **Statistical outlier removal does neither** — flying pixels form a continuous ribbon with normal local density and pass SOR trivially, while SOR's global threshold preferentially deletes valid far-field geometry. Use ROR as a mop-up only.

**Read intrinsics from the device, always.** Deriving `fx = Xres/2` from a nominal 90° FOV gives 424 against the measured 431.657 — a **1.8% error**. Using nominal 95 mm baseline instead of the measured 94.862 mm adds another 0.15%. Both are pure scale errors that never average out `[MEASURED]`.

**Use RGB for visual odometry, not left-IR.** The D455's colour sensor is global-shutter with an IR-cut filter and a FOV that matches depth `[MEASURED]` — so it never sees the projector dots. This avoids `emitter_on_off` entirely, which is non-deterministic, conflicts with auto-exposure, and has inverted metadata on some Jetson/UVC stacks.

**Organized cloud, `is_dense = false`, NaN for invalid.** Preserves O(1) neighbour lookup for normal estimation and the incidence test, and keeps invalid pixels honestly invalid. Zeros must never reach the deprojection multiply, or you get a dense blob of phantom points at the camera origin.

**Trust ≤6 m.** At 848×480 with a realistic σ=0.08 px, RMS error is 70 mm at 6 m, 195 mm at 10 m, and 781 mm at 20 m `[MEASURED]`. Feeding 20 m data into a 5 cm-voxel map is pure noise injection.

### 9.3 Where to add a learned model, if you add one

In priority order:

1. **Confidence estimation first, not depth.** The D455's biggest architectural gap is that it emits a *binary* validity mask with no graded confidence. **CCNN** (disparity-map-only, trains in ~15 min on 20 images) is designed for exactly this — *"suitable for depth devices that don't expose cost volume cues."* This gives you the two-tier occupied/unknown map that makes HIGH_ACCURACY's holes safe, at negligible cost.
2. **Mono depth as a gap-filler, tagged low-confidence.** Depth Anything V2 ViT-S runs ~20 fps on Orin AGX with TensorRT. Use it *only* where stereo returned invalid, scale-anchored against the surrounding valid stereo depths (a monotonic, spatially-local fit — **never a global rescale**, which made one benchmark 14× worse). Never let mono-derived points clear free space.
3. **Replace the ASIC only if you need it.** Isaac ROS ESS at ~79 fps on AGX Orin, with a native confidence output, is the credible option. Fast-FoundationStereo (~26 fps) if you need zero-shot robustness in genuinely unusual environments.

### 9.4 Open questions / things to verify on the actual robot

- **Measured σ_subpixel for this unit.** All the error tables span 0.05–0.20 px. Run the Depth Quality Tool against a flat textured target to find where this camera actually sits. This single number determines every accuracy claim downstream.
- **Thermal warm-up curve.** The 0.9%-over-12h drift figure is from a third party. Log `ASIC_TEMPERATURE` against a fixed-target depth on the actual robot to find the settling time and whether `THERMAL_COMPENSATION` is sufficient.
- **`emitter_on_off` metadata polarity** on this Jetson/host stack, if that path is ever used.
- **Whether Isaac ROS ESS's confidence output beats a CCNN-on-ASIC-disparity** — this is the key architectural fork and I found no published comparison.
- **Thin-obstacle recall** at 1/3/5 m for the recommended filter chain. The FP/FN asymmetry analysis (§6.7) predicts the discontinuity mask is the risky parameter; this needs empirical confirmation, not reasoning.

---

## 10. Key sources

**RealSense (first-party)**
- [Tuning depth cameras for best performance (BKM)](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/) — MinZ formula, error formula, subpixel targets, emitter guidance
- [Depth Post-Processing for D400](https://dev.realsenseai.com/docs/depth-post-processing-for-intel-realsense-depth-camera-d400-series/) — filter chain, disparity-domain argument
- [D400 Series Visual Presets](https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets)
- [Projectors for D400 Series](https://www.realsenseai.com/wp-content/uploads/2019/03/WhitePaper_on_Projectors_for_RealSense_D4xx_1.0.pdf)
- [Subpixel Linearity Improvement whitepaper](https://dev.realsenseai.com/docs/white-paper-subpixel-linearity-improvement-for-intel-realsense-depth-cameras/)
- [Projection, Texture-Mapping and Occlusion](https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/)
- [D400 Series Datasheet](https://www.realsenseai.com/wp-content/uploads/2023/10/Intel-RealSense-D400-Series-Datasheet-September-2023.pdf)
- Keselman et al., *Intel RealSense Stereoscopic Depth Cameras*, CVPRW 2017 — [ar5iv](https://ar5iv.labs.arxiv.org/html/1705.05548)

**Stereo algorithms**
- Hirschmüller, *Stereo Processing by Semiglobal Matching and Mutual Information*, TPAMI 2008
- Hirschmüller, Innocent & Garibaldi, *Real-Time Correlation-Based Stereo Vision with Reduced Border Errors*, IJCV 2002 — the foreground-fattening reference
- [RAFT-Stereo](https://arxiv.org/abs/2109.07547) · [CREStereo](https://arxiv.org/abs/2203.11483) · [IGEV](https://arxiv.org/abs/2303.06615) · [Selective-Stereo](https://arxiv.org/abs/2403.00486) · [FoundationStereo](https://arxiv.org/abs/2501.09898) · [Fast-FoundationStereo](https://arxiv.org/abs/2512.11130)
- [LightStereo re-benchmark](https://arxiv.org/abs/2406.19833) · [Burrus independent comparison](https://nicolas.burrus.name/stereo-comparison/)
- [Isaac ROS DNN Stereo Depth (ESS)](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html) · [libSGM](https://github.com/fixstars/libSGM) · [NVIDIA VPI Stereo Disparity](https://archive.docs.nvidia.com/vpi/algo_stereo_disparity.html)

**Monocular / metric depth**
- [Depth Anything V2](https://arxiv.org/html/2406.09414v2) · [Depth Anything 3](https://arxiv.org/html/2511.10647v1) · [Metric3D v2](https://arxiv.org/html/2404.15506v2) · [UniDepthV2](https://arxiv.org/html/2502.20110v2) · [Depth Pro](https://arxiv.org/html/2410.02073v1)
- [Stereo Anywhere (mono+stereo fusion)](https://arxiv.org/html/2412.04472v1) · [BridgeDepth/OmniDepth](https://arxiv.org/html/2508.04611v1)
- [MonoMPC — the 48.3% collision result](https://arxiv.org/html/2508.07387v1) · [Mono robustness study](https://arxiv.org/html/2507.00981v1) · [Underwater cross-domain benchmark](https://arxiv.org/html/2507.02148v2)

**Confidence, uncertainty, artifacts**
- Hu & Mordohai, *A Quantitative Evaluation of Confidence Measures for Stereo Vision*, TPAMI 2012 — [preprint](https://mordohai.github.io/public/Hu_EvalutionOfConfidence10.pdf)
- Poggi et al., *On the Confidence of Stereo Matching in a Deep-Learning Era*, TPAMI 2022 — [arXiv:2101.00431](https://arxiv.org/abs/2101.00431)
- Poggi & Mattoccia, *Learning from scratch a confidence measure* (CCNN), BMVC 2016 — [code](https://github.com/fabiotosi92/CCNN-Tensorflow)
- Poggi et al., *On the Uncertainty of Self-Supervised Monocular Depth Estimation*, CVPR 2020 — [code](https://github.com/mattpoggi/mono-uncertainty)
- Chen, Wang & Mordohai, *SEDNet*, CVPR 2023 — [code](https://github.com/lly00412/SEDNet)
- Kanade & Okutomi, *A Stereo Matching Algorithm with an Adaptive Window*, TPAMI 1994
- Yoon & Kweon, *Adaptive Support-Weight Approach for Correspondence Search*, TPAMI 2006
- Stein, Huertas & Matthies, *Attenuating Stereo Pixel-locking via Affine Window Adaptation*, ICRA 2006
- Shimizu & Okutomi, *Sub-Pixel Estimation Error Cancellation on Area-Based Matching*, IJCV 2005
- Matthies & Shafer, *Error Modeling in Stereo Navigation*, 1987 — [PDF](https://www.ri.cmu.edu/pub_files/pub3/matthies_l_1987_1/matthies_l_1987_1.pdf)
- Sabov & Krüger, *Identification and correction of flying pixels in range camera data*, SCCG 2008
- Reynolds et al., *Capturing Time-of-Flight Data with Confidence*, CVPR 2011
- [DSOR — range-scaled statistical outlier removal](https://ar5iv.labs.arxiv.org/html/2109.07078)
- [pcl::ShadowPoints](https://pointclouds.org/documentation/classpcl_1_1_shadow_points.html) · [REP-118 Depth Images](https://ros.org/reps/rep-0118.html) · [REP-103 Coordinate Conventions](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
