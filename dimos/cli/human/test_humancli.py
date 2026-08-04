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

from dimos.cli.human.humancli import _content_text


def test_content_text_extracts_responses_api_text_blocks() -> None:
    content = [
        {"type": "reasoning", "content": []},
        {"type": "text", "text": "The block is blue."},
    ]

    assert _content_text(content) == "The block is blue."
