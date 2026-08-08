# CodePolicy runtime responsibilities are fixed across evaluations

The initial evaluation system requires the CodePolicy agent runtime: one `python_exec` MCP tool backed by a persistent Python environment. Evaluations own session boundaries and public evaluation/task prompt components; the versioned runtime profile owns system instructions, tool descriptions, session-control messages, and deterministic prompt assembly. This keeps evaluation-specific semantics visible while allowing both in-house and third-party Evaluations to use the same agent interaction contract.
