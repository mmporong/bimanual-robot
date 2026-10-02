#!/usr/bin/env python3
"""Isaac 5.1 순정 ggao 병 파지·상승·같은 책상 복귀. 실기체 접근 없음."""
import argparse
import math
from pathlib import Path

import bottle_contact_model
from simulate_cup_contact import run


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=bottle_contact_model.DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--stay-open", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--bottle-mass-kg", type=float, dest="cup_mass_kg", help="병과 내용물 합산 가정 질량")
    parser.add_argument("--spawn-y-offset-mm", type=float)
    parser.add_argument("--spawn-x-offset-mm", type=float)
    parser.add_argument("--place", action="store_true")
    parser.set_defaults(recover=False)
    args = parser.parse_args()
    if args.headless and args.stay_open:
        parser.error("headless + stay-open은 허용하지 않습니다")
    if args.cup_mass_kg is not None and (not math.isfinite(args.cup_mass_kg) or args.cup_mass_kg <= 0):
        parser.error("bottle-mass-kg은 유한한 양수여야 합니다")
    for value in (args.spawn_y_offset_mm, args.spawn_x_offset_mm):
        if value is not None and not math.isfinite(value):
            parser.error("spawn offset은 유한한 숫자여야 합니다")
    run(args, backend=bottle_contact_model)
