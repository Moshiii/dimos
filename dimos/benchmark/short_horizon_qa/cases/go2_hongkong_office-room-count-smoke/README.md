# Hong Kong office room-count CLI smoke case

This case exercises the real `go2_hongkong_office` recording at progress `1.0`
through the standalone evaluation CLI.

Its expected count is the synthetic sentinel `0`. It validates CLI and runtime
plumbing only and is not the benchmark room-count oracle. Do not interpret a
failed task score as an agent or mapping regression.

The authoritative case remains incomplete until a human-authored room inventory,
counting policy, and independent review establish the expected count.

The credentialed API-key smoke was exercised on 2026-08-04 after adding live
progress. The final operational attempt completed in 312.9 seconds with 40
successful `python_exec` broker calls and a parsed `ANSWER: 8`. Its task score
was expectedly failed because this fixture's synthetic oracle is `0`; the result
must not be used as the authoritative room count.
