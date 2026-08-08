# Evaluation

This context describes executable evaluations, requests to run them, and immutable records of their execution.

## Language

**Evaluation**:
An executable definition that owns its protocol and result semantics.
_Avoid_: Evaluator, benchmark integration

**Evaluation Run Specification**:
A user-authored request that binds an evaluation and its configuration to a CodePolicy agent configuration. Output paths, credentials, infrastructure timeouts, and concurrency are operational settings outside it.
_Avoid_: Run, case

**Evaluation Run**:
An immutable record of one resolved execution of an evaluation run specification.
_Avoid_: Run configuration, mutable run

**Evaluation Case**:
An optional atomic input owned by an evaluation. It is not part of the universal evaluation run contract.
_Avoid_: Evaluation target

**Evaluation Attempt**:
One execution of an evaluation case or another evaluation-owned unit.
_Avoid_: Evaluation run

**CodePolicy Agent Runtime**:
The required agent interaction contract for an evaluation: exactly one `python_exec` MCP tool backed by a persistent Python environment.
_Avoid_: Evaluation subject, generic agent runtime

**CodePolicy Session**:
A lifecycle-bounded CodePolicy interaction whose Python namespace and agent conversation persist until the evaluation closes it. The evaluation owns session boundaries.
_Avoid_: Evaluation run, global agent session

**CodePolicy Runtime Profile**:
A versioned definition of the runtime-owned system instructions, `python_exec` tool surface, and session behavior used by a CodePolicy agent runtime. Evaluations cannot override its prompt responsibilities.
_Avoid_: Evaluation prompt, benchmark prompt

**Prompt Component**:
An immutable, separately recorded part of an agent prompt owned by either the runtime profile or the evaluation. Evaluation protocol and task input remain distinct components even when transported in one message.
_Avoid_: Prompt fragment, appended prompt

**Prompt Assembly**:
The deterministic runtime-owned combination of prompt components for one CodePolicy session.
_Avoid_: Prompt concatenation, prompt settings
