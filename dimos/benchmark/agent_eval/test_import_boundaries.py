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

import ast
from pathlib import Path

FORBIDDEN_PREFIXES = (
    "dimos.benchmark.dimsim",
    "dimos.benchmark.spatial",
)


def test_focused_evaluation_slice_has_no_live_benchmark_imports() -> None:
    package = Path(__file__).parent
    violations: list[str] = []

    for path in sorted(package.glob("*.py")):
        if path.name.startswith("test_"):
            continue
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                imports = [node.module]
            else:
                continue
            for imported in imports:
                if imported.startswith(FORBIDDEN_PREFIXES):
                    violations.append(f"{path.name}: {imported}")

    assert violations == []
