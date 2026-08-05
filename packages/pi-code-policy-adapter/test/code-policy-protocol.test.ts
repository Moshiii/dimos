import assert from "node:assert/strict";
import test from "node:test";

import {
  CODE_POLICY_MAX_LINE_BYTES,
  encodeCodePolicyFrame,
  parseCodePolicyFrame,
} from "../src/code-policy-protocol.js";

test("accepts a valid prompt and emits one newline-delimited frame", () => {
  assert.deepEqual(
    parseCodePolicyFrame(JSON.stringify({ version: 1, type: "prompt", id: "turn-1", text: "Count" })),
    { version: 1, type: "prompt", id: "turn-1", text: "Count" },
  );
  assert.equal(
    encodeCodePolicyFrame({ version: 1, type: "session_started", id: "session-1", tools: ["python_exec"] }),
    '{"version":1,"type":"session_started","id":"session-1","tools":["python_exec"]}\n',
  );
});

test("rejects malformed, unknown, and oversized inbound frames", () => {
  assert.throws(() => parseCodePolicyFrame("{"), /invalid code-policy JSON/);
  assert.throws(
    () => parseCodePolicyFrame(JSON.stringify({ version: 1, type: "unknown" })),
    /invalid code-policy frame/,
  );
  assert.throws(
    () => parseCodePolicyFrame("x".repeat(CODE_POLICY_MAX_LINE_BYTES + 1)),
    /exceeds limit/,
  );
});
