"""차체 결합 SmolVLA 4개 모델의 서버 로그에서 train/eval loss를 뽑아 CSV와 그래프를 만든다.

사용: python tools/training/make_loss_report.py <리포트 폴더>
  <리포트 폴더>/logs/<모델명>.log (서버 ~/lerobot-training/<모델명>.log 사본)을 읽고
  같은 폴더에 *_loss.csv, eval_loss_60_vs_80.png, train_loss_60_vs_80.png를 쓴다.
"""
import csv
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker

plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.cwd()
LOGS = HERE / "logs"
TRAIN = re.compile(r"ot_train.py:\d+ step:(\S+) .*? loss:([\d.]+) grdn:([\d.]+) lr:([\d.e+-]+)")
EVAL = re.compile(r"step (\d+): eval_loss=([\d.]+)")
TASKS = {"water_pour": "물 따르기 (water_pour)", "cup_place": "컵 배치 (cup_place)"}
SERIES = {"60": ("v1 60개", "#2a78d6"), "80": ("v1+part2 80개", "#eb6834")}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def parse(name):
    text = (LOGS / f"{name}.log").read_text(encoding="utf-8", errors="ignore").replace("\r", "\n")
    # 1000 이상은 "1K"처럼 뭉개져 찍히므로, 200 step마다 한 줄인 순번으로 step을 복원한다
    rows = TRAIN.findall(text)
    assert rows[0][0] == "200", rows[0]
    train = [((i + 1) * 200, float(l), float(g), float(lr)) for i, (_, l, g, lr) in enumerate(rows)]
    ev = {}
    for s, l in EVAL.findall(text):
        ev[int(s)] = float(l)  # 같은 step이 두 번 찍혀도 한 번만
    return train, sorted(ev.items())


def style(ax, title, ylabel):
    ax.set_title(title, loc="left", color=INK, fontsize=12)
    ax.set_xlabel("step", color=MUTED)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(True, color=GRID, linewidth=0.8)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(GRID)
    ax.tick_params(colors=MUTED)


data = {}
for task in TASKS:
    for n in SERIES:
        name = f"{task}_chassis_smolvla_{n}"
        data[(task, n)] = parse(name)
        train, ev = data[(task, n)]
        with open(HERE / f"{name}_train_loss.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["step", "train_loss", "grad_norm", "lr"])
            w.writerows(train)
        with open(HERE / f"{name}_eval_loss.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["step", "eval_loss"])
            w.writerows(ev)

for kind in ("eval", "train"):
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), facecolor="#fcfcfb")
    for ax, (task, label) in zip(axes, TASKS.items()):
        ax.set_facecolor("#fcfcfb")
        for n, (slabel, color) in SERIES.items():
            train, ev = data[(task, n)]
            if kind == "eval":
                xs, ys = zip(*ev)
                ax.plot(xs, ys, color=color, linewidth=2, marker="o", markersize=4, label=slabel)
                bs, bl = min(ev, key=lambda p: p[1])
                ax.plot([bs], [bl], marker="*", markersize=16, color=color,
                        markeredgecolor="#fcfcfb", markeredgewidth=1.5, zorder=5)
                ax.annotate(f"최적 {bs} step\n{bl:.4f}", (bs, bl), textcoords="offset points",
                            xytext=(-10, 22 if n == "60" or task == "water_pour" else -38), ha="right", color=INK, fontsize=9)
            else:
                xs = [t[0] for t in train]
                ys = [t[1] for t in train]
                ax.plot(xs, ys, color=color, linewidth=2, label=slabel)
                ax.set_yscale("log")
                ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
                ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        if kind == "eval":
            ax.set_ylim(0, None)
        style(ax, label, "eval loss (검증 6개 에피소드)" if kind == "eval" else "train loss (200 step마다, 로그축)")
        ax.legend(frameon=False, labelcolor=INK)
    fig.suptitle("SmolVLA 차체 결합 데이터 — " + ("검증 loss와 최적 체크포인트(★)" if kind == "eval" else "학습 loss"),
                 x=0.01, ha="left", color=INK, fontsize=14)
    fig.tight_layout()
    fig.savefig(HERE / f"{kind}_loss_60_vs_80.png", dpi=150)
    plt.close(fig)

for (task, n), (train, ev) in data.items():
    bs, bl = min(ev, key=lambda p: p[1])
    print(f"{task}_{n}: last_step={train[-1][0]} last_train_loss={train[-1][1]} evals={len(ev)} best={bs}:{bl}")
