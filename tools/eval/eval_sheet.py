"""실물 롤아웃 평가 기록 시트(md): 만들기 · 회차별 실행 명령 · 1등 모델 채우기 · 체크포인트 받기 · 집계(표+그래프).

평가 설계는 docs/20261028_IL_실물평가_일정표.md 를 따른다(π0 제외, GR00T·SmolVLA·ACT).
    A   : 데이터 80개 모델 3종 × 2과제 × 10회 (모델 순서를 라운드마다 돌린다)
    B   : A 1등 모델(과제별)의 데이터 20·40·60개 × 2과제 × 5회 (80개는 A 결과를 쓴다)
    OOD : 물따르기, 컵·병을 함께 좌우로 ±2·±5·±10cm 옮겨 1등 모델과 비교 모델 × 3회
회차 번호: A-P01(A 물따르기 1번) · A-C01(A 컵배치) · B-P01 · B-C01 · OOD-01

    python tools/eval/eval_sheet.py init                 # 시트 만들기(이미 있으면 거부)
    python tools/eval/eval_sheet.py cmd A-P01            # 그 회차의 롤아웃 명령 출력
    python tools/eval/eval_sheet.py fill --pour groot --place smolvla --ood groot,act
    python tools/eval/eval_sheet.py downloads            # 아직 없는 체크포인트의 hf 명령 출력
    python tools/eval/eval_sheet.py summary              # docs/eval/summary/ 에 요약.md + 그래프

시트는 VS Code 등에서 md 표 칸을 직접 채운다. 단계 칸은 O(통과)/X(실패), 앞 단계가 X면 뒤도 X.
성공 여부·진행 점수는 집계가 계산하므로 적지 않는다.
"""
from __future__ import annotations

import argparse
import re
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SHEET = REPO / "docs/eval/20261028_실물평가_기록시트.md"
OUT = REPO / "docs/eval/summary"
OUTPUTS = r"C:\Users\USER\lerobot-training\outputs"
PYTHON = r"C:\Users\USER\miniconda3\envs\so101_env\python.exe"
HF_EXE = r"C:\Users\USER\miniconda3\envs\so101_env\Scripts\hf.exe"

TASKS = {"P": "pour", "C": "place"}
TASK_KO = {"pour": "물따르기", "place": "컵배치"}
TASK_HF = {"pour": "water_pour", "place": "cup_place"}
STAGES = {
    "pour": ["1 컵 파지", "2 병 파지", "3 붓기", "4 병 내려놓기", "5 컵 적재·유지"],
    "place": ["1 테이블로 이동", "2 컵 내려놓기", "3 팔 빼기"],
}
MODELS = ["groot", "smolvla", "act"]
LABEL = {"groot": "GR00T", "smolvla": "SmolVLA", "act": "ACT"}
KEY = {v.lower(): k for k, v in LABEL.items()}
COLOR = {"groot": "#2a78d6", "smolvla": "#eb6834", "act": "#1baf7a"}  # dataviz 기본 범주색 1~3번(전 쌍 검증)
# 최적 step: docs/experiments/20261028_차체결합_32개모델_학습결과_종합.md
BEST_STEP = {
    ("groot", "pour"): {80: 17500, 60: 12000, 40: 8500, 20: 4000},
    ("groot", "place"): {80: 13000, 60: 6000, 40: 4500, 20: 1500},
    ("smolvla", "pour"): {80: 16000, 60: 12500, 40: 7000, 20: 4000},
    ("smolvla", "place"): {80: 7000, 60: 7000, 40: 3000, 20: 500},
    ("act", "pour"): {80: 8500, 60: 10000, 40: 7500, 20: 4000},
    ("act", "place"): {80: 3500, 60: 5500, 40: 7000, 20: 2500},
}
OOD_OFFSETS = [2, -2, 5, -5, 10, -10]  # cm, 로봇에서 테이블을 볼 때 + = 오른쪽
UNSET_B = "A 1등(미정)"
UNSET_OOD = ("비교1(미정)", "비교2(미정)")
TAIL = ["일시", "흘림", "시간(초)", "개입", "비상정지", "실패 증상", "영상", "메모"]
SPILL = ("없음", "조금", "많음")

ARM = (" --robot.type=bi_so_follower --robot.id=arms --robot.calibration_dir=calibration\\bi_follower"
       " --robot.left_arm_config.port=COM15 --robot.left_arm_config.gripper_protection=true"
       " --robot.left_arm_config.disable_torque_on_disconnect=false --robot.left_arm_config.max_relative_target=15"
       ' --robot.left_arm_config.cameras="{ wrist: {type: opencv, index_or_path: {left}, width: 640, height: 480, fps: 30} }"'
       " --robot.right_arm_config.port=COM16 --robot.right_arm_config.disable_torque_on_disconnect=false"
       " --robot.right_arm_config.max_relative_target=15"
       ' --robot.right_arm_config.cameras="{ wrist: {type: opencv, index_or_path: {right}, width: 640, height: 480, fps: 30} }"'
       ' --robot.cameras="{ top: {type: opencv, index_or_path: {depth}, width: 640, height: 480, fps: 30, mirror: true} }"')
TASK_FLAGS = {
    "pour": ' --task="Pick up the cup with the left arm, pick up the water bottle with the right arm, pour water from the'
            " bottle into the cup, set the bottle down to the right of where it was picked up, move the cup onto the robot"
            ' chassis with the left arm, and keep holding the cup." --duration=100 --display_data=true --start.pose=pour'
            " --end.pose=final --end.auto=pour",
    "place": ' --task="While holding the cup already loaded on the robot chassis with the left arm, move it to the table'
             ' and place it down." --duration=50 --display_data=true --end.pose=place_end --end.auto=place',
}


def checkpoint(model: str, task: str, n: int) -> str:
    step = BEST_STEP[(model, task)][n]
    return rf"{OUTPUTS}\{TASK_HF[task]}_chassis_{model}_{n}\checkpoints\{step:06d}\pretrained_model"


def hf_repo(model: str, task: str, n: int) -> str:
    return f"jun-seo/{TASK_HF[task]}_chassis_{model}_{n}_{BEST_STEP[(model, task)][n]}S"


def rotate(items: list, k: int) -> list:
    k %= len(items)
    return items[k:] + items[:k]


# ---------------------------------------------------------------- 시트 만들기
def header(phase: str, task: str) -> list[str]:
    cols = ["회차", "순서", "모델"]
    if phase == "B":
        cols.append("데이터 수")
    if phase == "OOD":
        cols.append("이동(cm)")
    cols += ["반복", TAIL[0], *STAGES[task]]
    cols += TAIL[1:] if task == "pour" else TAIL[2:]  # 흘림은 물따르기만
    return cols


def md_row(cells: list) -> str:
    return "| " + " | ".join(str(c) for c in cells) + " |"


def table(phase: str, task: str, rows: list[dict]) -> list[str]:
    cols = header(phase, task)
    out = [md_row(cols), "|" + "---|" * len(cols)]
    for r in rows:
        cells = {"회차": r["id"], "순서": r["order"], "모델": r["model"], "데이터 수": r.get("n", ""),
                 "이동(cm)": r.get("offset", ""), "반복": r["rep"]}
        out.append(md_row([cells.get(c, "") for c in cols]))
    return out


def build() -> str:
    order = 0
    sec = []
    for code, task in TASKS.items():  # A: 라운드마다 모델 순서를 한 칸씩 돌린다
        rows = []
        for rep in range(1, 11):
            for m in rotate(MODELS, rep - 1):
                order += 1
                rows.append({"id": f"A-{code}{len(rows) + 1:02d}", "order": order, "model": LABEL[m], "rep": rep})
        sec += [f"## A. {TASK_KO[task]} (데이터 80개, 10회씩)", "", *table("A", task, rows), ""]
    for code, task in TASKS.items():  # B: 1등 모델의 20·40·60개
        rows = []
        for rep in range(1, 6):
            for n in rotate([20, 40, 60], rep - 1):
                order += 1
                rows.append({"id": f"B-{code}{len(rows) + 1:02d}", "order": order, "model": UNSET_B, "n": n, "rep": rep})
        sec += [f"## B. {TASK_KO[task]} (A 1등 모델, 데이터 20·40·60개, 5회씩)", "", *table("B", task, rows), ""]
    rows = []  # OOD: 가까운 거리부터(먼 거리 생략 규칙을 쓰려고)
    for off in OOD_OFFSETS:
        for rep in range(1, 4):
            for slot in rotate(list(UNSET_OOD), rep - 1):
                order += 1
                rows.append({"id": f"OOD-{len(rows) + 1:02d}", "order": order, "model": slot, "offset": f"{off:+d}", "rep": rep})
    sec += ["## OOD. 물따르기 컵·병 좌우 이동 (데이터 80개, 3회씩)", "",
            "한 방향의 어떤 거리에서 0/3이면 그 모델은 같은 방향의 더 먼 거리를 생략한다. 생략한 줄은 단계 칸을 비우고 메모에 `생략(+5cm 0/3)`처럼 적는다.", "",
            *table("OOD", "pour", rows), ""]
    ckpt = ["| 모델 | 물따르기 | 컵배치 |", "|---|---|---|"]
    ckpt += [f"| {LABEL[m]} 80 | `{checkpoint(m, 'pour', 80)}` | `{checkpoint(m, 'place', 80)}` |" for m in MODELS]
    intro = [
        "# 실물 평가 기록 시트", "",
        "작성 방법은 [실물 평가 일정표](../20261028_IL_실물평가_일정표.md) 4절. 회차 하나가 끝날 때마다 그 줄의 빈칸을 채운다.", "",
        "| 칸 | 적는 값 |", "|---|---|",
        "| 일시 | `10-29 14:05` |",
        "| 단계 칸(1~5) | `O` 통과 / `X` 실패. 앞 단계가 X면 뒤 단계도 X |",
        "| 흘림 | `없음` / `조금` / `많음` (물따르기만, 눈으로 판정. 붓기 전에 실패했으면 비움) |",
        "| 시간(초) | 시작부터 끝까지 초. 자동 종료 시각이나 영상 기준 |",
        "| 개입 · 비상정지 | 있었으면 `예`, 없었으면 비움 |",
        "| 실패 증상 | 본 것만 짧게. 예: `병 미끄러짐`, `컵 앞에서 멈춤`, `붓다가 흘림`. 원인 추정은 메모에 |",
        "| 영상 | 휴대폰 영상 파일 이름. 촬영 시작 때 회차 번호 종이를 비춘다 |", "",
        "회차 번호의 P는 물따르기, C는 컵배치다. 실행 명령은 `python tools/eval/eval_sheet.py cmd A-P01`처럼 회차 번호로 출력한다.", "",
        "A단계 체크포인트(B·OOD는 1등을 정한 뒤 `fill`로 모델 칸이 채워지고, `cmd`가 경로를 알려 준다):", "",
        *ckpt, "",
    ]
    return "\n".join(intro + sec)


# ---------------------------------------------------------------- 시트 읽기·고치기
RUN_ID = re.compile(r"^(A|B)-([PC])\d{2}$|^OOD-\d{2}$")


def cells_of(line: str) -> list[str]:
    return [c.strip() for c in line.strip().strip("|").split("|")]


def read(path: Path) -> tuple[list[str], list[dict]]:
    """시트의 모든 회차 줄을 dict로 읽는다. 각 dict에 줄 번호(_line)와 그 표의 칸 이름(_cols)을 담는다."""
    lines = path.read_text(encoding="utf-8").splitlines()
    rows, cols = [], None
    for i, line in enumerate(lines):
        if not line.lstrip().startswith("|"):
            cols = None
            continue
        c = cells_of(line)
        if c and c[0] == "회차":
            cols = c
            continue
        if cols and c and RUN_ID.match(c[0]):
            r = dict(zip(cols, c + [""] * (len(cols) - len(c))))
            phase = c[0].split("-")[0]
            r.update(_line=i, _cols=cols, phase=phase, task="pour" if phase == "OOD" else TASKS[c[0][2]])
            r["key"] = KEY.get(r["모델"].lower(), "")
            r["n"] = int(r.get("데이터 수") or 80)
            r["off"] = int(r.get("이동(cm)") or 0)
            rows.append(r)
    return lines, rows


def find(rows: list[dict], run_id: str) -> dict:
    for r in rows:
        if r["회차"] == run_id:
            return r
    raise SystemExit(f"시트에 없는 회차: {run_id}")


def cmd_init(a) -> None:
    if a.sheet.exists() and not a.force:
        raise SystemExit(f"{a.sheet} 가 이미 있다(기록을 덮어쓰지 않는다). 새로 만들려면 --force")
    a.sheet.parent.mkdir(parents=True, exist_ok=True)
    a.sheet.write_text(build(), encoding="utf-8", newline="\n")
    _, rows = read(a.sheet)
    by = defaultdict(int)
    for r in rows:
        by[r["phase"]] += 1
    print(f"{a.sheet} : " + ", ".join(f"{k} {v}회" for k, v in by.items()))


def cmd_command(a) -> None:
    _, rows = read(a.sheet)
    r = find(rows, a.run_id)
    if not r["key"]:
        raise SystemExit(f"{a.run_id}: 모델이 아직 정해지지 않았다({r['모델']}). 먼저 fill 을 실행한다.")
    path = checkpoint(r["key"], r["task"], r["n"])
    if not Path(path, "model.safetensors").exists():
        print(f"[주의] 로컬에 체크포인트가 없다. 먼저 받는다:\n\"{HF_EXE}\" download {hf_repo(r['key'], r['task'], r['n'])} --local-dir \"{path}\"\n")
    where = f", 컵·병 {r['off']:+d}cm 이동" if r["off"] else ""
    print(f"[{a.run_id}] {TASK_KO[r['task']]} · {LABEL[r['key']]} · 데이터 {r['n']}개{where}")
    print(f'"{PYTHON}" tools\\rollout\\rollout_handoff.py --strategy.type=base --policy.path="{path}" --policy.device=cuda'
          + ARM + TASK_FLAGS[r["task"]] + (" " + a.extra if a.extra else ""))


def cmd_fill(a) -> None:
    lines, rows = read(a.sheet)
    ood = a.ood.split(",") if a.ood else []
    for m in ood:
        if m not in MODELS:
            raise SystemExit(f"모델은 {MODELS} 중 하나: {m}")
    best = {"pour": a.pour, "place": a.place}
    changed = 0
    for r in rows:
        new = None
        if r["phase"] == "B" and r["모델"] == UNSET_B and best.get(r["task"]):
            new = LABEL[best[r["task"]]]
        elif r["phase"] == "OOD" and r["모델"] in UNSET_OOD and len(ood) == 2:
            new = LABEL[ood[UNSET_OOD.index(r["모델"])]]
        if new:
            c = cells_of(lines[r["_line"]])
            c[r["_cols"].index("모델")] = new
            lines[r["_line"]] = md_row(c)
            changed += 1
    a.sheet.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(f"{changed}줄의 모델 칸을 채웠다")


def cmd_downloads(a) -> None:
    _, rows = read(a.sheet)
    for model, task, n in sorted({(r["key"], r["task"], r["n"]) for r in rows if r["key"]}):
        path = checkpoint(model, task, n)
        if not Path(path, "model.safetensors").exists():
            print(f'"{HF_EXE}" download {hf_repo(model, task, n)} --local-dir "{path}"')


# ---------------------------------------------------------------- 집계
def judged(r: dict) -> bool:
    return r.get(STAGES[r["task"]][0], "").upper() in ("O", "X")


def stats(rows: list[dict]) -> dict:
    rows = [r for r in rows if judged(r)]
    if not rows:
        return {"n": 0}
    st = STAGES[rows[0]["task"]]
    done = []
    for r in rows:
        d = 0
        for s in st:
            if r.get(s, "").upper() != "O":
                break
            d += 1
        done.append(d)
    out = {
        "n": len(rows),
        "success": sum(d == len(st) for d in done) / len(rows),
        "progress": sum(d / len(st) for d in done) / len(rows),
        "stage": [sum(d >= i for d in done) / len(rows) for i in range(1, len(st) + 1)],
        "interv": sum(r.get("개입", "") == "예" for r in rows),
        "estop": sum(r.get("비상정지", "") == "예" for r in rows),
    }
    spills = [r.get("흘림", "") for r in rows if r.get("흘림", "") in SPILL]
    out["spill"] = {k: spills.count(k) for k in SPILL} if spills else None
    durs = []
    for r in rows:
        try:
            durs.append(float(r.get("시간(초)", "")))
        except ValueError:
            pass
    out["duration"] = sum(durs) / len(durs) if durs else None
    return out


def pct(x) -> str:
    return "—" if x is None else f"{x * 100:.0f}%"


def spill_text(s: dict) -> str:
    return "—" if not s.get("spill") else " / ".join(f"{k} {v}" for k, v in s["spill"].items())


def cmd_summary(a) -> None:
    _, rows = read(a.sheet)
    a.out.mkdir(parents=True, exist_ok=True)
    groups = defaultdict(list)
    for r in rows:
        groups[(r["phase"], r["task"], r["key"], r["n"], r["off"])].append(r)
    md = ["# 실물 평가 집계", "", "판정이 적힌 회차만 센다. 성공 = 모든 단계 O, 진행 = 통과한 단계 수 / 전체 단계 수의 평균.", ""]

    md += ["## A. 데이터 80개 모델 비교", ""]
    for task in TASK_KO:
        st = STAGES[task]
        spill_col = task == "pour"
        md += [f"### {TASK_KO[task]}", "",
               "| 모델 | 회차 수 | 성공률 | 평균 진행 | " + " | ".join(st) + (" | 흘림(없음/조금/많음)" if spill_col else "") + " | 평균 시간(초) | 개입 | 비상정지 |",
               "|---|---|---|---|" + "---|" * len(st) + ("---|" if spill_col else "") + "---|---|---|"]
        for m in MODELS:
            s = stats(groups[("A", task, m, 80, 0)])
            if not s["n"]:
                md.append(f"| {LABEL[m]} | 0 | 미실시 |" + " |" * (len(st) + 4 + spill_col))
                continue
            md.append(f"| {LABEL[m]} | {s['n']} | {pct(s['success'])} | {pct(s['progress'])} | "
                      + " | ".join(pct(x) for x in s["stage"])
                      + (f" | {spill_text(s)}" if spill_col else "")
                      + " | " + (f"{s['duration']:.0f}" if s["duration"] else "—") + f" | {s['interv']} | {s['estop']} |")
        md.append("")
        plot_stages(a.out / f"A_{TASK_HF[task]}_단계별통과율.png", task, {m: stats(groups[("A", task, m, 80, 0)]) for m in MODELS})

    md += ["## B. 데이터 수별 (A 1등 모델, 80개는 A 결과)", "", "| 과제 | 모델 | 데이터 수 | 회차 수 | 성공률 | 평균 진행 |", "|---|---|---|---|---|---|"]
    curves_b = {}
    for task in TASK_KO:
        used = {r["key"] for r in rows if r["phase"] == "B" and r["task"] == task and r["key"]}
        for m in [m for m in MODELS if m in used]:
            pts = []
            for n in (20, 40, 60, 80):
                s = stats(groups[("B" if n != 80 else "A", task, m, n, 0)])
                md.append(f"| {TASK_KO[task]} | {LABEL[m]} | {n} | {s['n']} | {pct(s.get('success'))} | {pct(s.get('progress'))} |")
                if s["n"]:
                    pts.append((n, s["success"]))
            curves_b[f"{TASK_KO[task]} · {LABEL[m]}"] = pts
    md.append("")
    plot_curve(a.out / "B_데이터수별_성공률.png", curves_b, "학습 데이터 수(에피소드)", "데이터 수별 성공률", [20, 40, 60, 80])

    md += ["## OOD. 컵·병 좌우 이동 (물따르기, 0cm는 A 결과)", "", "| 모델 | 이동(cm) | 회차 수 | 성공률 | 평균 진행 |", "|---|---|---|---|---|"]
    curves_o = {}
    used = {r["key"] for r in rows if r["phase"] == "OOD" and r["key"]}
    for m in [m for m in MODELS if m in used]:
        pts = []
        for off in sorted({0, *OOD_OFFSETS}):
            s = stats(groups[("OOD" if off else "A", "pour", m, 80, off)])
            md.append(f"| {LABEL[m]} | {off:+d} | {s['n']} | {pct(s.get('success'))} | {pct(s.get('progress'))} |")
            if s["n"]:
                pts.append((off, s["success"]))
        curves_o[LABEL[m]] = pts
    md.append("")
    plot_curve(a.out / "OOD_이동거리별_성공률.png", curves_o, "컵·병 이동 거리(cm, + = 오른쪽)",
               "학습 위치에서 벗어날 때 성공률 (물따르기)", sorted({0, *OOD_OFFSETS}), colors={LABEL[m]: COLOR[m] for m in MODELS})

    fails = defaultdict(int)
    for r in rows:
        if judged(r) and r.get("실패 증상", ""):
            fails[(TASK_KO[r["task"]], LABEL.get(r["key"], r["모델"]), r["실패 증상"])] += 1
    if fails:
        md += ["## 실패 증상 빈도", "", "| 과제 | 모델 | 증상 | 횟수 |", "|---|---|---|---|"]
        md += [f"| {t} | {m} | {c} | {k} |" for (t, m, c), k in sorted(fails.items(), key=lambda x: -x[1])]
    (a.out / "요약.md").write_text("\n".join(md) + "\n", encoding="utf-8", newline="\n")
    print(f"{a.out / '요약.md'} 와 그래프를 만들었다")


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    installed = {f.name for f in font_manager.fontManager.ttflist}
    font = next((f for f in ("Malgun Gothic", "AppleGothic", "NanumGothic") if f in installed), "DejaVu Sans")
    plt.rcParams.update({"font.family": font, "axes.unicode_minus": False, "axes.spines.top": False,
                         "axes.spines.right": False, "axes.edgecolor": "#9a9893", "axes.labelcolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                         "grid.color": "#e6e5e1", "grid.linewidth": 0.8, "axes.axisbelow": True})
    return plt


def plot_stages(path: Path, task: str, by_model: dict) -> None:
    done = {m: s for m, s in by_model.items() if s["n"]}
    if not done:
        return
    plt = _mpl()
    st = STAGES[task]
    fig, ax = plt.subplots(figsize=(9, 4.6), dpi=200)
    w = 0.8 / len(done)
    for i, (m, s) in enumerate(done.items()):
        xs = [j + (i - (len(done) - 1) / 2) * w for j in range(len(st))]
        bars = ax.bar(xs, [v * 100 for v in s["stage"]], width=w - 0.03, color=COLOR[m], label=f"{LABEL[m]} (n={s['n']})")
        ax.bar_label(bars, labels=[f"{v * 100:.0f}" for v in s["stage"]], fontsize=7, color="#52514e", padding=2)
    ax.set_xticks(range(len(st)), st, fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("해당 단계까지 통과한 비율(%)")
    ax.set_title(f"{TASK_KO[task]} 단계별 통과율 (데이터 80개)", loc="left", fontsize=11, color="#0b0b0b")
    ax.legend(frameon=False, fontsize=8, ncols=len(done), loc="upper left", bbox_to_anchor=(0, -0.13))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_curve(path: Path, curves: dict, xlabel: str, title: str, xticks: list, colors: dict | None = None) -> None:
    curves = {k: v for k, v in curves.items() if v}
    if not curves:
        return
    plt = _mpl()
    palette = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
    fig, ax = plt.subplots(figsize=(7, 4), dpi=200)
    for i, (name, pts) in enumerate(curves.items()):
        c = (colors or {}).get(name, palette[i % len(palette)])
        ax.plot([p[0] for p in pts], [p[1] * 100 for p in pts], color=c, linewidth=2, marker="o", markersize=7,
                markeredgecolor="white", markeredgewidth=1.5, label=name)
    ax.set_xticks(xticks)
    ax.set_ylim(-5, 105)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("성공률(%)")
    ax.set_title(title, loc="left", fontsize=11, color="#0b0b0b")
    ax.legend(frameon=False, fontsize=8, loc="upper left", bbox_to_anchor=(0, -0.16), ncols=len(curves))
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet", type=Path, default=SHEET)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("init", help="시트 만들기"); p.add_argument("--force", action="store_true"); p.set_defaults(fn=cmd_init)
    p = sub.add_parser("cmd", help="회차의 롤아웃 명령 출력")
    p.add_argument("run_id"); p.add_argument("--extra", default="", help="명령 끝에 붙일 플래그. 예: --extra=--start.pose=place_start")
    p.set_defaults(fn=cmd_command)
    p = sub.add_parser("fill", help="B·OOD 모델 칸 채우기")
    p.add_argument("--pour", choices=MODELS); p.add_argument("--place", choices=MODELS)
    p.add_argument("--ood", help="OOD 비교 모델 2개, 예: groot,act")
    p.set_defaults(fn=cmd_fill)
    sub.add_parser("downloads", help="없는 체크포인트의 다운로드 명령").set_defaults(fn=cmd_downloads)
    p = sub.add_parser("summary", help="요약.md + 그래프"); p.add_argument("--out", type=Path, default=OUT); p.set_defaults(fn=cmd_summary)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
