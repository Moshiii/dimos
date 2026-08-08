# Copyright 2026 Dimensional Inc.
"""Local browser viewer for VQA evaluation progress and grounding artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
from threading import Thread
from typing import Any
from urllib.parse import unquote, urlparse
import webbrowser

_VISUALIZER_DIR = Path(__file__).with_name("visualizer")


@dataclass
class VqaViewer:
    """Running local VQA viewer server."""

    url: str
    _server: ThreadingHTTPServer
    _thread: Thread

    def stop(self) -> None:
        """Stop the local server, primarily for callers that own its lifetime."""
        self._server.shutdown()
        self._server.server_close()
        self._thread.join()


def start_viewer(dataset: Path, run_directory: Path) -> VqaViewer:
    """Start a local VQA review viewer and return its URL."""
    root = dataset.expanduser().resolve()

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self._send_static("index.html", "text/html; charset=utf-8")
                return
            if path == "/static/viewer.js":
                self._send_static("viewer.js", "text/javascript; charset=utf-8")
                return
            if path == "/api/state":
                self._send_bytes(
                    json.dumps(_viewer_state(root, run_directory)).encode(), "application/json"
                )
                return
            if path.startswith("/media/"):
                self._send_media(root, unquote(path.removeprefix("/media/")))
                return
            self.send_error(404)

        def _send_media(self, root: Path, relative: str) -> None:
            path = (root / relative).resolve()
            if path not in _allowed_media(root) or not path.is_file():
                self.send_error(404)
                return
            self._send_bytes(path.read_bytes(), mimetypes.guess_type(path.name)[0] or "image/jpeg")

        def _send_static(self, name: str, content_type: str) -> None:
            self._send_bytes((_VISUALIZER_DIR / name).read_bytes(), content_type)

        def _send_bytes(self, content: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, _format: str, *args: object) -> None:
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    webbrowser.open(url)
    return VqaViewer(url, server, thread)


def _viewer_state(dataset: Path, run_directory: Path) -> dict[str, Any]:
    manifest = _load_json(run_directory / "run-manifest.json")
    results = _load_run_results(run_directory, manifest)
    cases = []
    for case_path in sorted(dataset.glob("frame-*/cases/*/case.json")):
        case = json.loads(case_path.read_text())
        case_dir = case_path.parent
        frame_dir = case_dir.parent.parent
        result = results.get(case["case_id"])
        cases.append(
            {
                "case_id": case["case_id"],
                "frame": frame_dir.name,
                "question": case["question"],
                "answer_policy": (
                    f"choices: {', '.join(case['allowed_answers'])}"
                    if case.get("answer_kind", "choice") == "choice"
                    else f"numeric: {case['unit']} +/- {case['tolerance']}"
                ),
                "image": f"/media/{case_dir.relative_to(dataset)}/{case['image_path']}",
                "overlay": f"/media/{frame_dir.relative_to(dataset)}/grounding_overlay.jpg",
                "result": result,
            }
        )
    completed = list(results.values())
    return {
        "cases": cases,
        "total": len(cases),
        "completed": len(completed),
        "passed": sum(item.get("passed") is True for item in completed),
        "failed": sum(item.get("passed") is False for item in completed),
        "infra_errors": sum(item.get("infra_error") is not None for item in completed),
        "status": manifest.get("status", "running"),
    }


def _load_run_results(run_directory: Path, manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if manifest.get("status") == "completed":
        return _load_results(run_directory / "results.json")
    checkpoints = run_directory / "checkpoints"
    if not checkpoints.is_dir():
        return {}
    return {
        item["result"]["case_id"]: item["result"]
        for path in sorted(checkpoints.glob("*.json"))
        if (item := _load_json(path))
    }


def _load_results(path: Path) -> dict[str, dict[str, Any]]:
    return {item["case_id"]: item for item in _load_json(path, [])}


def _load_json(path: Path, default: Any = None) -> Any:
    if not path.is_file():
        return {} if default is None else default
    return json.loads(path.read_text())


def _allowed_media(dataset: Path) -> set[Path]:
    allowed: set[Path] = set()
    for case_path in dataset.glob("frame-*/cases/*/case.json"):
        case = json.loads(case_path.read_text())
        case_dir = case_path.parent
        image = (case_dir / case["image_path"]).resolve()
        overlay = (case_dir.parent.parent / "grounding_overlay.jpg").resolve()
        if dataset in image.parents and "private" not in image.parts:
            allowed.add(image)
        if dataset in overlay.parents and "private" not in overlay.parts:
            allowed.add(overlay)
    return allowed
