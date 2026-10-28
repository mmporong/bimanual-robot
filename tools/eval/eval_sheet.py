"""실물 롤아웃 평가 기록 시트: 만들기 · 1등 모델 채우기 · 체크포인트 받기 명령 · 집계(표+그래프).

평가 설계는 docs/20261028_IL_실물평가_일정표.md 를 따른다(π0 제외, GR00T·SmolVLA·ACT).
    A   : 데이터 80개 모델 3종 × 2과제 × 10회 (모델 순서를 회차마다 돌린다)
    B   : A 1등 모델(과제별)의 데이터 20·40·60개 × 2과제 × 5회 (80개는 A 결과를 쓴다)
    OOD : 물따르기, 컵·병을 함께 좌우로 ±2·±5·±10cm 옮겨 1등 모델과 비교 모델 × 3회

    python tools/eval/eval_sheet.py init                                  # 시트 만들기(이미 있으면 거부)
    python tools/eval/eval_sheet.py fill --pour groot --place smolvla --ood groot,act
    python tools/eval/eval_sheet.py downloads                             # 아직 없는 체크포인트의 hf 명령 출력
    python tools/eval/eval_sheet.py summary                               # summary.md + 그래프 PNG

시트는 엑셀로 열어 회차마다 한 줄씩 채운다. stage1~5 칸은 O(통과)/X(실패), 해당 없음은 '-'.
앞 단계가 X면 뒤 단계도 X로 적는다. 성공·진행 점수는 집계가 계산하므로 적지 않는다.
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SHEET = REPO / "docs/eval/20261028_실물평가_기록시트.csv"
OUT = REPO / "docs/eval/summary"
OUTPUTS = r"C:\Users\USER\lerobot-training\outputs"
HF_EXE = r"C:\Users\USER\miniconda3\envs\so101_env\Scripts\hf.exe"

TASKS = {"pour": "water_pour", "place": "cup_place"}
STAGES = {
    "pour": ["컵 파지(왼팔)", "병 파지(오른팔)", "붓기(내용물이 컵에)", "병 내려놓기", "컵 차체 적재·유지"],
    "place": ["컵 쥔 채 테이블로 이동", "컵 내려놓기(세워짐)", "그리퍼 놓고 팔 빼기"],
}
MODELS = ["groot", "smolvla", "act"]
LABEL = {"groot": "GR00T N1.7", "smolvla": "SmolVLA", "act": "ACT"}
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
FIELDS = ["run_id", "phase", "order", "task", "model", "data_n", "offset_cm", "repeat", "checkpoint",
          "done_at", "stage1", "stage2", "stage3", "stage4", "stage5", "content_g", "in_cup_g", "spilled_g",
          "duration_s", "intervention", "estop", "failure_code", "video", "notes"]
TBD = "TBD"


def checkpoint(model: str, task: str, n: int) -> str:
    step = BEST_STEP[(model, task)][n]
    return rf"{OUTPUTS}\{TASKS[task]}_chassis_{model}_{n}\checkpoints\{step:06d}\pretrained_model"


def hf_repo(model: str, task: str, n: int) -> str:
    return f"jun-seo/{TASKS[task]}_chassis_{model}_{n}_{BEST_STEP[(model, task)][n]}S"


def rotate(items: list, k: int) -> list:
    k %= len(items)
    return items[k:] + items[:k]


def row(run_id: str, phase: str, order: int, task: str, model: str, n: int, offset: int, rep: int) -> dict:
    r = {f: "" for f in FIELDS}
    r.update(run_id=run_id, phase=phase, order=order, task=task, model=model, data_n=n, offset_cm=offset, repeat=rep,
             checkpoint=checkpoint(model, task, n) if model != TBD and not model.startswith(TBD) else "")
    if task == "place":
        r.update(stage4="-", stage5="-", content_g="-", in_cup_g="-", spilled_g="-")
    return r


def plan() -> list[dict]:
    rows, order = [], 0
    for task in TASKS:  # A: 과제마다 10라운드, 라운드마다 모델 순서를 한 칸씩 돌린다
        k = 0
        for rep in range(1, 11):
            for model in rotate(MODELS, rep - 1):
                order += 1; k += 1
                rows.append(row(f"A-{task}-{k:02d}", "A", order, task, model, 80, 0, rep))
    for task in TASKS:  # B: 1등 모델의 20·40·60개
        k = 0
        for rep in range(1, 6):
            for n in rotate([20, 40, 60], rep - 1):
                order += 1; k += 1
                rows.append(row(f"B-{task}-{k:02d}", "B", order, task, TBD, n, 0, rep))
    k = 0  # OOD: 가까운 거리부터(먼 거리 생략 규칙을 쓰려고)
    for offset in OOD_OFFSETS:
        for rep in range(1, 4):
            for slot in rotate([f"{TBD}1", f"{TBD}2"], rep - 1):
                order += 1; k += 1
                rows.append(row(f"OOD-pour-{k:02d}", "OOD", order, "pour", slot, 80, offset, rep))
    return rows


def read(path: Path) -> list[dict]:
    for enc in ("utf-8-sig", "cp949"):  # 엑셀이 다른 인코딩으로 저장해도 읽는다
        try:
            with path.open(encoding=enc, newline="") as f:
                return list(csv.DictReader(f))
        except UnicodeDecodeError:
            continue
    raise SystemExit(f"{path}: 인코딩을 읽지 못했다")


def write(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)


def cmd_init(a) -> None:
    if a.sheet.exists() and not a.force:
        raise SystemExit(f"{a.sheet} 가 이미 있다(기록을 덮어쓰지 않는다). 새로 만들려면 --force")
    rows = plan()
    write(a.sheet, rows)
    by = defaultdict(int)
    for r in rows:
        by[r["phase"]] += 1
    print(f"{a.sheet} : " + ", ".join(f"{k} {v}회" for k, v in by.items()))


def cmd_fill(a) -> None:
    rows = read(a.sheet)
    best = {"pour": a.pour, "place": a.place}
    ood = a.ood.split(",") if a.ood else []
    for m in [a.pour, a.place, *ood]:
        if m and m not in MODELS:
            raise SystemExit(f"모델은 {MODELS} 중 하나: {m}")
    changed = 0
    for r in rows:
        if r["phase"] == "B" and r["model"] == TBD and best.get(r["task"]):
            r["model"] = best[r["task"]]
        elif r["phase"] == "OOD" and r["model"] in (f"{TBD}1", f"{TBD}2") and len(ood) == 2:
            r["model"] = ood[int(r["model"][-1]) - 1]
        else:
            continue
        r["checkpoint"] = checkpoint(r["model"], r["task"], int(r["data_n"]))
        changed += 1
    write(a.sheet, rows)
    print(f"{changed}줄 채움")


def cmd_downloads(a) -> None:
    need = sorted({(r["model"], r["task"], int(r["data_n"])) for r in read(a.sheet) if r["model"] in MODELS})
    for model, task, n in need:
        path = checkpoint(model, task, n)
        if not Path(path, "model.safetensors").exists():
            print(f'"{HF_EXE}" download {hf_repo(model, task, n)} --local-dir "{path}"')


def judged(r: dict) -> bool:
    return r["stage1"].strip().upper() in ("O", "X")


def progress(r: dict) -> tuple[int, int]:
    n = len(STAGES[r["task"]])
    done = 0
    for i in range(1, n + 1):
        if r[f"stage{i}"].strip().upper() != "O":
            break
        done += 1
    return done, n


def stats(rows: list[dict]) -> dict:
    rows = [r for r in rows if judged(r)]
    if not rows:
        return {"n": 0}
    n_st = len(STAGES[rows[0]["task"]])
    pr = [progress(r) for r in rows]
    out = {
        "n": len(rows),
        "success": sum(d == t for d, t in pr) / len(rows),
        "progress": sum(d / t for d, t in pr) / len(rows),
        "stage": [sum(d >= i for d, _ in pr) / len(rows) for i in range(1, n_st + 1)],
        "interv": sum(r["intervention"].strip().upper() in ("Y", "O") for r in rows),
    }
    ratios = []
    for r in rows:
        try:
            ratios.append(float(r["in_cup_g"]) / float(r["content_g"]))
        except (ValueError, ZeroDivisionError):
            pass
    out["pour_ratio"] = sum(ratios) / len(ratios) if ratios else None
    durs = [float(r["duration_s"]) for r in rows if r["duration_s"].strip().replace(".", "", 1).isdigit()]
    out["duration"] = sum(durs) / len(durs) if durs else None
    return out


def pct(x) -> str:
    return "—" if x is None else f"{x * 100:.0f}%"


def cmd_summary(a) -> None:
    rows = read(a.sheet)
    a.out.mkdir(parents=True, exist_ok=True)
    src = a.sheet.resolve()
    src = src.relative_to(REPO).as_posix() if src.is_relative_to(REPO) else src.name
    md = ["# 실물 평가 집계", "", f"원본: `{src}` · 판정이 적힌 회차만 센다(성공 = 모든 단계 O).", ""]
    groups = defaultdict(list)
    for r in rows:
        groups[(r["phase"], r["task"], r["model"], int(r["data_n"]), int(r["offset_cm"] or 0))].append(r)

    md += ["## A. 데이터 80개 모델 비교", ""]
    for task in TASKS:
        st = STAGES[task]
        md += [f"### {TASKS[task]}", "", "| 모델 | n | 성공률 | 평균 진행 | " + " | ".join(st) + " | 붓기 비율 | 평균 시간(s) | 개입 |",
               "|---|---|---|---|" + "---|" * len(st) + "---|---|---|"]
        for m in MODELS:
            s = stats(groups[("A", task, m, 80, 0)])
            if s["n"]:
                md.append(f"| {LABEL[m]} | {s['n']} | {pct(s['success'])} | {pct(s['progress'])} | "
                          + " | ".join(pct(x) for x in s["stage"])
                          + f" | {pct(s['pour_ratio']) if task == 'pour' else '-'} | "
                          + (f"{s['duration']:.0f}" if s["duration"] else "—") + f" | {s['interv']} |")
            else:
                md.append(f"| {LABEL[m]} | 0 | 미실시 |" + " |" * (len(st) + 4))
        md.append("")
        plot_stages(a.out / f"A_{task}_stage_pass.png", task, {m: stats(groups[("A", task, m, 80, 0)]) for m in MODELS})

    md += ["## B. 데이터 수별 (1등 모델, 80개는 A 결과)", "", "| 과제 | 모델 | 데이터 수 | n | 성공률 | 평균 진행 |", "|---|---|---|---|---|---|"]
    curves_b = {}
    for task in TASKS:
        models = {r["model"] for r in rows if r["phase"] == "B" and r["task"] == task and r["model"] in MODELS}
        for m in [m for m in MODELS if m in models]:
            pts = []
            for n in (20, 40, 60, 80):
                s = stats(groups[("B" if n != 80 else "A", task, m, n, 0)])
                md.append(f"| {TASKS[task]} | {LABEL[m]} | {n} | {s['n']} | {pct(s.get('success'))} | {pct(s.get('progress'))} |")
                if s["n"]:
                    pts.append((n, s["success"], s["n"]))
            curves_b[f"{TASKS[task]} · {LABEL[m]}"] = pts
    md.append("")
    plot_curve(a.out / "B_data_count.png", curves_b, "학습 데이터 수(에피소드)", "데이터 수별 성공률", [20, 40, 60, 80])

    md += ["## OOD. 컵·병 좌우 이동 (물따르기, 0cm는 A 결과)", "", "| 모델 | 이동(cm) | n | 성공률 | 평균 진행 |", "|---|---|---|---|---|"]
    curves_o = {}
    ood_models = {r["model"] for r in rows if r["phase"] == "OOD"}
    for m in [m for m in MODELS if m in ood_models]:
        pts = []
        for off in sorted({0, *OOD_OFFSETS}):
            s = stats(groups[("OOD" if off else "A", "pour", m, 80, off)])
            md.append(f"| {LABEL[m]} | {off:+d} | {s['n']} | {pct(s.get('success'))} | {pct(s.get('progress'))} |")
            if s["n"]:
                pts.append((off, s["success"], s["n"]))
        curves_o[LABEL[m]] = pts
    md.append("")
    plot_curve(a.out / "OOD_offset.png", curves_o, "컵·병 이동 거리(cm, + = 오른쪽)", "학습 위치에서 벗어날 때 성공률 (물따르기)",
               sorted({0, *OOD_OFFSETS}), colors={LABEL[m]: COLOR[m] for m in MODELS})

    fails = defaultdict(int)
    for r in rows:
        if judged(r) and r["failure_code"].strip():
            fails[(r["task"], r["model"], r["failure_code"].strip())] += 1
    if fails:
        md += ["## 실패 증상(failure_code) 빈도", "", "| 과제 | 모델 | 증상 | 횟수 |", "|---|---|---|---|"]
        md += [f"| {TASKS[t]} | {LABEL.get(m, m)} | {c} | {k} |" for (t, m, c), k in sorted(fails.items(), key=lambda x: -x[1])]
    (a.out / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"{a.out / 'summary.md'} 와 그래프를 만들었다")


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    installed = {f.name for f in font_manager.fontManager.ttflist}
    font = next((f for f in ("Malgun Gothic", "AppleGothic", "NanumGothic") if f in installed), "DejaVu Sans")
    plt.rcParams.update({"font.family": font,
                         "axes.unicode_minus": False, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#9a9893", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                         "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e1", "grid.linewidth": 0.8,
                         "axes.axisbelow": True})
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
    ax.set_xticks(range(len(st)), [f"{i + 1}. {t}" for i, t in enumerate(st)], fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("해당 단계까지 통과한 비율(%)")
    ax.set_title(f"{TASKS[task]} 단계별 통과율 (데이터 80개)", loc="left", fontsize=11, color="#0b0b0b")
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
        xs, ys = [p[0] for p in pts], [p[1] * 100 for p in pts]
        ax.plot(xs, ys, color=c, linewidth=2, marker="o", markersize=7, markeredgecolor="white", markeredgewidth=1.5, label=name)
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
    p = sub.add_parser("init"); p.add_argument("--force", action="store_true"); p.set_defaults(fn=cmd_init)
    p = sub.add_parser("fill")
    p.add_argument("--pour", choices=MODELS); p.add_argument("--place", choices=MODELS)
    p.add_argument("--ood", help="OOD 비교 모델 2개, 예: groot,act")
    p.set_defaults(fn=cmd_fill)
    sub.add_parser("downloads").set_defaults(fn=cmd_downloads)
    p = sub.add_parser("summary"); p.add_argument("--out", type=Path, default=OUT); p.set_defaults(fn=cmd_summary)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
