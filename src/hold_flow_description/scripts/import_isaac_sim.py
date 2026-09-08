#!/usr/bin/env python3
"""검증된 HOLD THE FLOW URDF를 Isaac Sim 6.0 USD로 변환한다.

일반 Python에서는 ``--check-only``만 실행한다. 실제 변환은 Isaac Sim의
``python.sh`` 또는 Isaac Sim Python 환경에서 실행해야 한다.
"""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/isaac_sim_6.yaml"
VALIDATOR = Path(__file__).with_name("validate_isaac_contract.py")
BOOLEAN_IMPORTER_OPTIONS = (
    "collision_from_visuals",
    "merge_mesh",
    "merge_fixed_joints",
    "allow_self_collision",
    "fix_base",
)


def run_static_validation() -> None:
    subprocess.run([sys.executable, str(VALIDATOR)], cwd=REPO_ROOT, check=True)


def materialize_isaac_urdf(source: Path, output_dir: Path) -> Path:
    """package URI를 절대 메시 경로로 바꾼 Isaac 전용 중간 URDF를 만든다."""
    output_dir.mkdir(parents=True, exist_ok=True)
    text = source.read_text(encoding="utf-8")
    text = text.replace(
        "package://hold_flow_description/",
        str(PACKAGE_ROOT.resolve()) + "/",
    )
    output = output_dir / "hold_flow.isaac.urdf"
    output.write_text(text, encoding="utf-8")
    return output


def load_import_settings(config_path: Path) -> tuple[dict[str, bool], str, Path, Path]:
    """YAML을 검사하고 importer 옵션과 기본 입출력 경로를 반환한다."""
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"Isaac 설정 최상위는 객체여야 합니다: {config_path}")
    if document.get("target") != "Isaac_Sim_6_0":
        raise ValueError(f"지원하지 않는 Isaac target: {document.get('target')!r}")
    importer = document.get("importer")
    if not isinstance(importer, dict):
        raise ValueError("Isaac 설정에 importer 객체가 없습니다")
    settings: dict[str, bool] = {}
    for name in BOOLEAN_IMPORTER_OPTIONS:
        value = importer.get(name)
        if not isinstance(value, bool):
            raise ValueError(f"importer.{name}은 bool이어야 합니다: {value!r}")
        settings[name] = value
    robot_type = importer.get("robot_type")
    if robot_type != "Mobile Manipulators":
        raise ValueError(f"robot_type은 'Mobile Manipulators'여야 합니다: {robot_type!r}")
    source_urdf = REPO_ROOT / str(document["source_urdf"])
    output_usd = REPO_ROOT / str(document["output_usd_default"])
    return settings, robot_type, source_urdf, output_usd


def create_import_config(
    config_class: type,
    urdf_path: Path,
    usd_dir: Path,
    settings: dict[str, bool],
    robot_type: str,
):
    """Isaac Sim 6.0 계약의 모든 필드를 생략 없이 생성자에 전달한다."""
    return config_class(
        urdf_path=str(urdf_path),
        usd_path=str(usd_dir),
        collision_from_visuals=settings["collision_from_visuals"],
        merge_mesh=settings["merge_mesh"],
        merge_fixed_joints=settings["merge_fixed_joints"],
        allow_self_collision=settings["allow_self_collision"],
        fix_base=settings["fix_base"],
        robot_type=robot_type,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--usd-dir", type=Path)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    run_static_validation()
    importer_settings, robot_type, default_urdf, default_usd = load_import_settings(
        args.config.resolve()
    )
    urdf_path = (args.urdf or default_urdf).resolve()
    usd_dir = (args.usd_dir or default_usd.parent).resolve()
    if args.check_only:
        print(f"Isaac Sim 정적 가져오기 계약: PASS ({robot_type}, {importer_settings})")
        return

    if importlib.util.find_spec("isaacsim") is None:
        raise SystemExit(
            "Isaac Sim Python 모듈을 찾지 못했습니다. 이 호스트에서는 --check-only만 가능하며, "
            "Isaac Sim 6.0 설치 폴더의 python.sh로 이 스크립트를 실행해야 합니다."
        )

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    try:
        import omni.kit.app

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        import_urdf = materialize_isaac_urdf(urdf_path, usd_dir)
        config = create_import_config(
            URDFImporterConfig,
            import_urdf,
            usd_dir,
            importer_settings,
            robot_type,
        )

        output_path = URDFImporter(config).import_urdf()
        if not output_path or not Path(output_path).exists():
            raise RuntimeError(f"URDF importer가 유효한 USD 경로를 반환하지 않았습니다: {output_path}")
        print(f"Isaac Sim USD 생성: {output_path}")
        print(f"프로필: {robot_type}")
        print(f"적용된 가져오기 옵션: {importer_settings}")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
