"""빈손 프리뷰 녹화의 실행별 증거를 보존한다.

Isaac Sim에 의존하지 않는 상태 관리 모듈이다. 녹화를 시작하는 즉시 미완료
결과를 기록하고, 완료 또는 중단 사유로 같은 결과를 종결한다.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class PreviewRecording:
    """한 번의 빈손 프리뷰 녹화를 독립 디렉터리에서 관리한다."""

    def __init__(self, output_dir: Path, frame_rate: int = 10) -> None:
        if frame_rate <= 0:
            raise ValueError("frame_rate는 양수여야 합니다")
        root = Path(output_dir) / "demo_recordings"
        root.mkdir(exist_ok=True)
        index = 0
        while (root / f"run_{index:04d}").exists():
            index += 1
        self.run_dir = root / f"run_{index:04d}"
        self.frames_dir = self.run_dir / "frames"
        self.frames_dir.mkdir(parents=True)
        self.result_path = self.run_dir / "manifest.json"
        self.frame_rate = frame_rate
        self.frames: list[dict[str, Any]] = []
        self.active = True
        self._write(completed=False, reason="recording_in_progress")

    def frame_path(self) -> Path:
        if not self.active:
            raise RuntimeError("종결된 녹화에는 프레임을 추가할 수 없습니다")
        return self.frames_dir / f"frame_{len(self.frames):04d}.png"

    def add_frame(self, evidence: dict[str, Any]) -> None:
        if not self.active:
            raise RuntimeError("종결된 녹화에는 프레임을 추가할 수 없습니다")
        self.frames.append(dict(evidence))
        self._write(completed=False, reason="recording_in_progress")

    def finish(self, *, completed: bool, reason: str,
               final_reset_error_rad: float | None = None) -> None:
        if not self.active:
            return
        if not reason:
            raise ValueError("녹화 종결 사유가 필요합니다")
        self._write(completed=completed, reason=reason,
                    final_reset_error_rad=final_reset_error_rad)
        self.active = False

    def _write(self, *, completed: bool, reason: str,
               final_reset_error_rad: float | None = None) -> None:
        payload = {
            "completed": completed,
            "reason": reason,
            "kind": "empty_hand_orientation_not_grasp",
            "frame_rate": self.frame_rate,
            "frames": self.frames,
            "final_reset_error_rad": final_reset_error_rad,
            "hardware_accessed": False,
        }
        temporary = self.result_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2,
                                        allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(self.result_path)
