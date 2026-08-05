import assert from "node:assert/strict";
import { Readable, Writable } from "node:stream";
import test from "node:test";
import {
  progressFrame,
  runCodePolicyAdapter,
  type CodePolicySessionFactory,
} from "../src/code-policy-main.js";

function sink(): { stream: Writable; frames: () => Array<Record<string, unknown>> } {
  const chunks: string[] = [];
  return {
    stream: new Writable({
      write(chunk, _encoding, callback) {
        chunks.push(String(chunk));
        callback();
      },
    }),
    frames: () =>
      chunks
        .join("")
        .trim()
        .split("\n")
        .filter(Boolean)
        .map((line) => JSON.parse(line) as Record<string, unknown>),
  };
}

test("normalizes visible progress and discards thinking and raw events", () => {
  assert.deepEqual(progressFrame({ type: "agent_start" }), {
    version: 1,
    type: "transcript",
    event: "agent_start",
  });
  assert.deepEqual(
    progressFrame({
      type: "message_update",
      assistantMessageEvent: { type: "text_delta", delta: "Looking around" },
    }),
    {
      version: 1,
      type: "transcript",
      event: "assistant_text_delta",
      delta: "Looking around",
    },
  );
  assert.equal(
    progressFrame({
      type: "message_update",
      assistantMessageEvent: { type: "thinking_delta", delta: "private reasoning" },
    }),
    undefined,
  );
  assert.equal(progressFrame({ type: "before_provider_request", payload: "private" }), undefined);
});

test("streams concise progress during a code-policy turn", async () => {
  const output = sink();
  const previousMode = process.env.PI_SPATIAL_AUTH_MODE;
  const previousKey = process.env.OPENAI_API_KEY;
  process.env.PI_SPATIAL_AUTH_MODE = "openai-api-key";
  process.env.OPENAI_API_KEY = "test-key";
  let listener: ((event: unknown) => void) | undefined;
  const factory: CodePolicySessionFactory = async () => ({
    subscribe: (next) => {
      listener = next;
    },
    prompt: async () => {
      listener?.({ type: "agent_start" });
      listener?.({ type: "turn_start" });
      listener?.({
        type: "message_update",
        assistantMessageEvent: { type: "thinking_delta", delta: "do not emit" },
      });
      listener?.({
        type: "message_update",
        assistantMessageEvent: { type: "text_delta", delta: "Visible text" },
      });
      listener?.({
        type: "message_update",
        assistantMessageEvent: { type: "text_delta", delta: "\nANSWER: 2" },
      });
      listener?.({ type: "agent_end" });
      return undefined;
    },
    abort: async () => undefined,
    dispose: () => undefined,
    sessionEvidence: () => ({ state: "complete", persisted: false }),
  });
  const input = Readable.from(
    [
      {
        version: 1,
        type: "session_start",
        id: "session-1",
        initial_prompt: "Count rooms",
        thinking_level: "medium",
      },
      { version: 1, type: "prompt", id: "turn-1", text: "Count rooms" },
      { version: 1, type: "dispose" },
    ].map((frame) => `${JSON.stringify(frame)}\n`),
  );

  try {
    await runCodePolicyAdapter(input, output.stream, new Writable({
      write(_chunk, _encoding, callback) {
        callback();
      },
    }), factory);
  } finally {
    if (previousMode === undefined) delete process.env.PI_SPATIAL_AUTH_MODE;
    else process.env.PI_SPATIAL_AUTH_MODE = previousMode;
    if (previousKey === undefined) delete process.env.OPENAI_API_KEY;
    else process.env.OPENAI_API_KEY = previousKey;
  }

  const frames = output.frames();
  assert.equal(frames.some((frame) => frame.event === "assistant_text_delta"), true);
  assert.equal(JSON.stringify(frames).includes("Visible text"), true);
  assert.equal(JSON.stringify(frames).includes("do not emit"), false);
  const complete = frames.find((frame) => frame.type === "turn_complete");
  assert.equal(complete?.final_text, "Visible text\nANSWER: 2");
});
