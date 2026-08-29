# Tracklet re-identification: what was measured

`dimos/utils/tracklet_reid.py` merges tracklets a tracker lost. This records what
it was tested against and what the results ruled out, because the headline is
negative: **pairwise association works and automatic grouping does not**, and
nobody should spend a week rediscovering that.

## Ground truth without labelling

Re-identification's job is "the tracker lost this; find it again", so the
benchmark makes the tracker lose it on purpose.

| | How | Why it is exact |
|---|---|---|
| **same-object pairs** | cut one tracklet into pieces, discard 30% between them | the tracker held it continuously, so the pieces are one object |
| **different-object pairs** | two tracklets overlapping in time | both visible at one instant in different tracks |

Neither needs a human. The discarded span is a dial, so a method can be scored
on how far it bridges rather than only on whether it merges anything.

Data: one 7.8-minute go2 capture of an SF office (`go2_SF_office_8mins_moshi`),
6,628 frames, 36,189 sightings, 64% placed in 3D. Crops embedded once with CLIP;
every method saw identical input, differing only in the association rule.

**388 pieces from 194 tracks — 194 same-object pairs, 1,246 different-object
pairs.**

## Results

| method | recall | precision | merged same | merged different |
|---|---|---|---|---|
| ours, pairs only, tuned | **99.0%** | **100.0%** | 192 | **0** |
| upstream `EmbeddingIDSystem` rule | 99.0% | 100.0% | 192 | 0 |
| ours, pairs only, shipped defaults | 83.5% | 100.0% | 162 | 0 |
| geometry only, no appearance | 96.9% | 41.2% | 188 | 268 |
| ours, with transitive closure | 96.4% | 38.5% | 187 | 299 |
| ours, online (ids, so groups) | 96.9% | 23.9% | 188 | 597 |
| appearance only @0.63 | 100.0% | 13.5% | 194 | 1,238 |

## What this rules out

**Grouping.** Pairwise decisions are perfect — 192 right, 0 wrong. Every failure
appears the moment pairs are chained into groups, whether offline by transitive
closure (38.5%) or online by assigning ids (23.9%). Two of every three merges in
a group are wrong. An office is full of identical furniture, so one legal link
joins two groups, and each link passed its distance, speed and spread check on
its own.

Adding constraints moved this but did not fix it. A group-spread ceiling took the
online path from 1 id to 23, against a true 194 — twenty times better and still
an order of magnitude out.

**So `find_merges` returns pairs and `connected` is opt-in.** Whether eight
fragments are one chair or three is a judgement; looking at eight crops answers
it and tuning a threshold does not.

**Online.** An id *is* a group, so the online path inherits every grouping
failure and cannot revise what it already said. It is in the module because the
shape is right for a robot, not because these numbers justify deploying it.

## What the geometry veto is worth

Upstream vetoes only co-occurrence — things visible in the same frame. Adding
position and speed refused **48 pairs upstream would have merged, all 48
correctly**: things that look alike and were never seen together, like two
chairs of one model at opposite ends of a room.

    similarity 0.914, 5.3 m apart, reachable 5.3 m  -> different objects
    similarity 0.890, 8.0 m apart, reachable 4.1 m  -> different objects

The benchmark cannot score this. Its negatives are defined as *overlapping in
time*, which is exactly what co-occurrence already catches, so the 48 fall
outside the labelled set. The benchmark understates this method's advantage, and
that limitation is the reason to keep the number here rather than in a score.

## Thresholds are the model's, not the code's

Measured on this capture with CLIP: same-object pairs at median 0.967,
different-object pairs at 0.862, separating at 0.925 with 93.7% balanced
accuracy. The repo's person-re-identification default of 0.63 would have merged
nearly every different-object pair — under *that* aggregation.

Under `top_frac`, 0.63 is right and 0.925 costs 15 points of recall. **The
threshold and the aggregation are one setting, not two**, and changing either
without re-measuring the other is how the shipped defaults ended up 15 points
low. Re-measure by cutting tracklets in half; it takes one GPU pass.

## Known limits of these numbers

- One capture, one room, one model, one detector.
- Positives are two halves of a tracklet with 30% discarded: temporally adjacent,
  small viewpoint change. **Easier than the real problem**, so every recall
  figure here is optimistic. Widening the cut is the next measurement.
- `torchreid`'s OSNet was never run — its weights need LFS credentials this
  machine lacks — so "is a person-re-identification model better than CLIP for
  office furniture" is open.
- 36% of detections carry no 3D position and are refused outright. On this data
  that was 340,000 of 673,000 refusals. The limit on re-identification here is
  placement, not association.
