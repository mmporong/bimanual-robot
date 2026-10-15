"""Resolve detector defaults without importing camera or inference runtimes."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/perception/water_detection"


def default_path(source: Path, name: str) -> Path:
    tree = ast.parse(source.read_text())
    assignment = next(
        node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)
    )
    return eval(compile(ast.Expression(assignment.value), str(source), "eval"),
                {"Path": Path, "__file__": str(source)})


def test_detector_defaults_find_the_bundled_model_directory():
    liquid = default_path(PACKAGE / "water_detection/detect_gui.py", "MODEL_PATH")
    cup = default_path(PACKAGE / "perception/yoloe_detector.py", "_LOCAL_MODEL")
    assert liquid == PACKAGE / "models/water_yoloe.pt"
    assert liquid.is_file() and liquid.stat().st_size > 20_000_000
    assert cup == liquid.with_name("yoloe-26s-seg.pt")
