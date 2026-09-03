#!/usr/bin/env python3
"""HOLD THE FLOW 차체 부품을 K1 Max + PETG 인계 문서 스펙으로 슬라이싱한다.

인계 문서 `docs/20260901_설계팀_제작인계패키지_P0.md` 5.2절의 시작 프로파일을
OrcaSlicer 시스템 프리셋 위에 그대로 얹는다. 시스템 프리셋을 평탄화하지 않고
`inherits`를 남긴 사본에 키만 덮어써서 CLI 호환 실패(-17)를 피한다.

OrcaSlicer CLI는 필라멘트 프리셋의 상속 체인을 스스로 풀지 못해
"Creality Generic PETG"를 넘겨도 G-code가 PLA 200도로 나온다. 그래서 상속을
파이썬에서 먼저 해석해 실제 값(PETG 255도 등)을 사본에 명시한다.

사용법:
  python3 design/cad/slice_k1max_petg.py --out ~/gcode STL...
  python3 design/cad/slice_k1max_petg.py --out ~/gcode --all      # exports/stl 전체
  python3 design/cad/slice_k1max_petg.py --emit-presets           # GUI 임포트용 프리셋만 생성
출력: stdout JSON
"""
import argparse, glob, importlib.util, json, os, subprocess, sys

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(HOME, ".cache", "orca-3d-holdflow")
SKILL = os.path.join(HOME, ".claude", "skills", "3d", "scripts", "slice.py")

MACHINE = "Creality K1 Max (0.4 nozzle)"
PROCESS = "0.20mm Standard @Creality K1Max (0.4 nozzle)"
FILAMENT = "Creality Generic PETG"
FILAMENT_TPU = "Creality Generic TPU @K1-all"
BED_TEMP_TPU = "45"

# 인계 문서 5.2절 시작 프로파일. G-code 승인값이 아니라 시작값이다.
PROCESS_SPEC = {
    "layer_height": "0.2",
    "initial_layer_print_height": "0.25",
    "wall_loops": "6",
    "top_shell_layers": "6",
    "bottom_shell_layers": "6",
    "sparse_infill_density": "45%",
    "sparse_infill_pattern": "gyroid",
    "enable_support": "0",
    "detect_thin_wall": "1",
    "seam_position": "aligned",
}
BRIM_LARGE = {"brim_type": "outer_only", "brim_width": "8"}   # 250 mm 판
BRIM_SMALL = {"brim_type": "auto_brim", "brim_width": "5"}
LARGE_PARTS = {"chassis_bottom", "chassis_middle", "chassis_top"}

# 상속 체인에서 실제 값을 읽어올 필라멘트 키
FILAMENT_KEYS = [
    "filament_type", "nozzle_temperature", "nozzle_temperature_initial_layer",
    "filament_flow_ratio", "filament_max_volumetric_speed", "fan_max_speed",
    "fan_min_speed", "overhang_fan_speed", "slow_down_layer_time",
    "filament_density", "filament_cost", "temperature_vitrification",
]
PLATE_TEMP_KEYS = ["cool_plate_temp", "eng_plate_temp", "hot_plate_temp",
                   "textured_plate_temp", "supertack_plate_temp"]
BED_TEMP = "80"


def load_skill():
    spec = importlib.util.spec_from_file_location("orca_slice", SKILL)
    if spec is None or not os.path.exists(SKILL):
        raise SystemExit(f"3d 스킬 스크립트를 찾지 못했습니다: {SKILL}")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def build_presets(sl, idx, brim, filament_name=None, bed_temp=None):
    os.makedirs(WORK, exist_ok=True)
    src = {k: idx.get((t, n)) for k, (t, n) in
           {"machine": ("machine", MACHINE), "process": ("process", PROCESS),
            "filament": ("filament", filament_name or FILAMENT)}.items()}
    missing = [k for k, v in src.items() if not v]
    if missing:
        raise SystemExit(f"시스템 프리셋을 찾지 못했습니다: {missing}. OrcaSlicer에서 Creality 벤더를 설치하세요.")

    # 상대 압출 검증(-51) 통과용 G92 E0
    plate_temp = bed_temp or BED_TEMP
    lcg = sl.resolved(idx, "machine", MACHINE, "layer_change_gcode", "") or ""
    m_patch = {}
    if "G92" not in lcg:
        m_patch["layer_change_gcode"] = (lcg.rstrip("\n") + "\n" if lcg else "") + \
            ";AFTER_LAYER_CHANGE\n;[layer_z]\nG92 E0\n"

    # 필라멘트: 상속을 미리 풀어 값을 명시한다 (CLI가 못 푸는 부분)
    f_patch = {}
    for k in FILAMENT_KEYS:
        v = sl.resolved(idx, "filament", filament_name or FILAMENT, k)
        if v is not None:
            f_patch[k] = v
    for k in PLATE_TEMP_KEYS:
        f_patch[k] = [plate_temp]
        f_patch[k + "_initial_layer"] = [plate_temp]

    p_patch = dict(PROCESS_SPEC)
    p_patch.update(brim)

    return (sl.override(src["machine"], os.path.join(WORK, "machine.json"), m_patch) if m_patch else src["machine"],
            sl.override(src["process"], os.path.join(WORK, "process.json"), p_patch),
            sl.override(src["filament"], os.path.join(WORK, "filament.json"), f_patch))


def emit_presets(sl, idx):
    """OrcaSlicer GUI로 가져갈 수 있게 저장소에도 사본을 남긴다."""
    out = os.path.join(HERE, "slicer")
    os.makedirs(out, exist_ok=True)
    written = []
    for brim_name, brim in (("large", BRIM_LARGE), ("small", BRIM_SMALL)):
        m, p, f = build_presets(sl, idx, brim)
        for label, path in (("process", p), ("filament", f)):
            if label == "filament" and brim_name == "small":
                continue
            name = f"HOLDFLOW_{label}_K1Max_PETG" + (f"_{brim_name}brim" if label == "process" else "")
            d = json.load(open(path))
            d["name"] = name
            dst = os.path.join(out, name + ".json")
            json.dump(d, open(dst, "w"), indent=1, ensure_ascii=False)
            written.append(dst)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl", nargs="*")
    ap.add_argument("--out", default=None, help="G-code 출력 디렉터리 (홈 아래)")
    ap.add_argument("--all", action="store_true", help="design/cad/exports/stl 전체")
    ap.add_argument("--emit-presets", action="store_true", help="GUI 임포트용 프리셋만 생성")
    ap.add_argument("--tpu", action="store_true",
                    help="TPU 95A 로 슬라이싱. 인서트용")
    ap.add_argument("--support", choices=["off", "tree", "normal"], default="off",
                    help="서포트. ggao50 그리퍼 부품은 tree 가 필요하다")
    ap.add_argument("--plate-only", action="store_true",
                    help="서포트를 베드에서만 세운다. 모델 위 자국을 줄인다")
    ap.add_argument("--threshold", type=float, default=30.0, help="서포트 임계각(도)")
    ap.add_argument("--infill", default=None,
                    help="인필 밀도 덮어쓰기 (예: 20%%). 쿠폰처럼 하중을 받지 않는 부품용")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    sl = load_skill()
    cmd, conf = sl.find_orca()
    idx = sl.preset_index(conf)

    if a.emit_presets:
        json.dump({"ok": True, "presets": emit_presets(sl, idx)}, sys.stdout, ensure_ascii=False, indent=1)
        print(); return

    stls = [os.path.abspath(s) for s in a.stl]
    if a.all:
        stls = sorted(glob.glob(os.path.join(HERE, "exports", "stl", "*.stl")))
    if not stls:
        raise SystemExit("슬라이싱할 STL이 없습니다. 경로를 주거나 --all 을 쓰세요.")
    if not a.out:
        raise SystemExit("--out 이 필요합니다.")

    out = os.path.abspath(os.path.expanduser(a.out))
    if not out.startswith(HOME):
        raise SystemExit(f"출력 경로는 홈 아래여야 합니다 (flatpak 샌드박스): {out}")
    os.makedirs(out, exist_ok=True)

    results, failed = [], 0
    for stl in stls:
        stem = os.path.splitext(os.path.basename(stl))[0]
        brim = dict(BRIM_LARGE if stem in LARGE_PARTS else BRIM_SMALL)
        if a.infill:
            brim["sparse_infill_density"] = a.infill
        if a.support == "off":
            brim["enable_support"] = "0"
        else:
            brim.update({
                "enable_support": "1",
                "support_type": "tree(auto)" if a.support == "tree" else "normal(auto)",
                "support_style": "organic" if a.support == "tree" else "snug",
                "support_threshold_angle": str(int(a.threshold)),
                "support_on_build_plate_only": "1" if a.plate_only else "0",
            })
        m, p, f = build_presets(
            sl, idx, brim,
            filament_name=FILAMENT_TPU if a.tpu else None,
            bed_temp=BED_TEMP_TPU if a.tpu else None,
        )
        argv = list(cmd) + ["--load-settings", f"{m};{p}", "--load-filaments", f,
                            "--ensure-on-bed", "--slice", "0", "--outputdir", out, stl]
        if a.dry_run:
            results.append({"part": stem, "command": argv}); continue

        for stale in glob.glob(os.path.join(out, "plate_*.gcode")) + [os.path.join(out, "result.json")]:
            try: os.remove(stale)
            except OSError: pass
        r = subprocess.run(argv, capture_output=True, text=True, timeout=2400)
        res = os.path.join(out, "result.json")
        info = json.load(open(res)) if os.path.exists(res) else {}
        plates = sorted(glob.glob(os.path.join(out, "plate_*.gcode")))
        if r.returncode != 0 or not plates:
            failed += 1
            results.append({"part": stem, "ok": False, "returncode": r.returncode,
                            "error": info.get("error_string") or (r.stdout or r.stderr)[-400:]})
            continue
        tag = "TPU" if a.tpu else "PETG"
        final = os.path.join(out, f"{stem}_K1Max_{tag}_0.2mm.gcode")
        os.replace(plates[0], final)
        for extra in plates[1:]:
            os.remove(extra)
        try: os.remove(res)
        except OSError: pass
        results.append({"part": stem, "ok": True, "gcode": final, "verify": sl.verify(final)})

    json.dump({"ok": failed == 0, "machine": MACHINE, "process": PROCESS, "filament": FILAMENT,
               "spec": PROCESS_SPEC, "bed_temp": BED_TEMP, "failed": failed, "results": results},
              sys.stdout, ensure_ascii=False, indent=1)
    print()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
