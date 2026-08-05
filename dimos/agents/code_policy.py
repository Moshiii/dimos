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

"""Compatibility DimOS module over the standalone CodePolicy session core."""

from __future__ import annotations

from typing import Any

from pydantic import model_validator

from dimos.agents.annotation import skill
from dimos.agents.code_policy_core import (
    DEFAULT_INTERRUPT_GRACE_S,
    DEFAULT_OUTPUT_LIMIT,
    DEFAULT_STARTUP_TIMEOUT_S,
    MAX_EXECUTION_TIMEOUT_S,
    CodePolicyExecutionRecord,
    CodePolicyObserverProbeReceipt,
    CodePolicyObserverState,
    CodePolicySession,
    CodePolicySessionConfig,
    CodePolicySessionReceipt,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
    _bootstrap_source as _bootstrap_source,
    _BoundedTextOutput as _BoundedTextOutput,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig


class CodePolicyConfig(ModuleConfig):
    recording_path: str
    derived_recording_path: str | None = None
    memory_cutoff_timestamp: float | None = None
    connect_app: bool = True
    output_limit: int = DEFAULT_OUTPUT_LIMIT
    startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S
    interrupt_grace_s: float = DEFAULT_INTERRUPT_GRACE_S

    @model_validator(mode="after")
    def validate_environment(self) -> CodePolicyConfig:
        frozen_values = (self.derived_recording_path, self.memory_cutoff_timestamp)
        if (frozen_values[0] is None) != (frozen_values[1] is None):
            raise ValueError(
                "derived_recording_path and memory_cutoff_timestamp must be set together"
            )
        if frozen_values[0] is not None and self.connect_app:
            raise ValueError("connect_app must be false for frozen memory")
        if frozen_values[0] is None and not self.connect_app:
            raise ValueError("connect_app=false requires a frozen memory environment")
        return self

    def session_config(self) -> CodePolicySessionConfig:
        if self.derived_recording_path is not None:
            assert self.memory_cutoff_timestamp is not None
            environment = FrozenMemoryEnvironment(
                recording_path=self.recording_path,
                derived_recording_path=self.derived_recording_path,
                memory_cutoff_timestamp=self.memory_cutoff_timestamp,
            )
        else:
            environment = LiveDimosEnvironment(recording_path=self.recording_path)
        return CodePolicySessionConfig(
            environment=environment,
            output_limit=self.output_limit,
            startup_timeout_s=self.startup_timeout_s,
            interrupt_grace_s=self.interrupt_grace_s,
        )


class CodePolicyModule(Module):
    """Temporary module wrapper; new evaluation paths use standalone CodePolicy."""

    config: CodePolicyConfig  # type: ignore[assignment]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session = CodePolicySession(self.config.session_config())

    @rpc
    def start(self) -> None:
        super().start()
        self._session.start()

    @skill
    def python_exec(self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
        """Execute one synchronous Python program in the persistent policy session.

        The trusted, unsandboxed session preloads `memory` for observations and,
        in the live environment, `app` for deployed DimOS RPCs. Imports, functions,
        variables, and mutations persist across calls until the host resets the
        session. Use this for observation processing, control flow, retries, and
        coordinated multi-RPC behavior.

        Args:
            code: Complete Python source for one task attempt.
            timeout_s: Execution deadline in seconds, greater than 0 and at most 110.
        """
        return self._session.python_exec(code, timeout_s)

    @rpc
    def reset_session(self) -> CodePolicySessionReceipt:
        return self._session.reset_session()

    @rpc
    def get_session_receipt(self) -> CodePolicySessionReceipt:
        return self._session.get_session_receipt()

    @rpc
    def get_execution_records(
        self, session_id: str | None = None
    ) -> tuple[CodePolicyExecutionRecord, ...]:
        return self._session.get_execution_records(session_id)

    @rpc
    def prepare_observer(self) -> CodePolicyObserverState:
        return self._session.prepare_observer()

    @rpc
    def get_observer_state(self, known_generation: int | None = None) -> CodePolicyObserverState:
        return self._session.get_observer_state(known_generation)

    @rpc
    def issue_observer_probe(self, kernel_generation: int) -> CodePolicyObserverProbeReceipt:
        return self._session.issue_observer_probe(kernel_generation)

    @rpc
    def interrupt_active(self) -> bool:
        return self._session.interrupt_active()

    @rpc
    def stop(self) -> None:
        self._session.stop()
        super().stop()

    @property
    def _execution_lock(self) -> Any:
        return self._session._execution_lock

    @_execution_lock.setter
    def _execution_lock(self, value: Any) -> None:
        self._session._execution_lock = value

    @property
    def _kernel_manager(self) -> Any:
        return self._session._kernel_manager

    @_kernel_manager.setter
    def _kernel_manager(self, value: Any) -> None:
        self._session._kernel_manager = value

    @property
    def _kernel_client(self) -> Any:
        return self._session._kernel_client

    @_kernel_client.setter
    def _kernel_client(self, value: Any) -> None:
        self._session._kernel_client = value

    @property
    def _kernel_generation(self) -> int:
        return self._session._kernel_generation

    @_kernel_generation.setter
    def _kernel_generation(self, value: int) -> None:
        self._session._kernel_generation = value


code_policy_module = CodePolicyModule.blueprint
