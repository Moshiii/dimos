# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import json

import requests

from dimos.benchmark.vqa.evaluation import viewer


def test_viewer_state_exposes_public_image_and_grounding_overlay(tmp_path) -> None:
    case_dir = tmp_path / "frame-000001" / "cases" / "case-1"
    case_dir.mkdir(parents=True)
    (case_dir / "case.json").write_text(
        json.dumps(
            {
                "case_id": "case-1",
                "image_path": "image.jpg",
                "question": "Is the chair left?",
                "allowed_answers": ["yes", "no"],
                "answer_marker": "ANSWER:",
            }
        )
    )
    run = tmp_path / "run"
    (run / "checkpoints").mkdir(parents=True)
    (run / "run-manifest.json").write_text(json.dumps({"status": "running"}))
    (run / "checkpoints" / "00000000.json").write_text(
        json.dumps(
            {"index": 0, "case_id": "case-1", "result": {"case_id": "case-1", "passed": True}}
        )
    )

    state = viewer._viewer_state(tmp_path, run)

    assert state["total"] == 1
    assert state["passed"] == 1
    assert state["cases"][0]["image"] == "/media/frame-000001/cases/case-1/image.jpg"
    assert state["cases"][0]["overlay"] == "/media/frame-000001/grounding_overlay.jpg"


def test_viewer_can_be_stopped(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(viewer.webbrowser, "open", lambda _url: True)

    running = viewer.start_viewer(tmp_path, tmp_path / "run")
    running.stop()

    assert not running._thread.is_alive()


def test_viewer_serves_external_html_and_javascript(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(viewer.webbrowser, "open", lambda _url: True)
    running = viewer.start_viewer(tmp_path, tmp_path / "run")
    try:
        page = requests.get(running.url, timeout=5)
        script = requests.get(f"{running.url}/static/viewer.js", timeout=5)
    finally:
        running.stop()

    assert page.status_code == 200
    assert 'src="/static/viewer.js"' in page.text
    assert script.status_code == 200
    assert "function render" in script.text
