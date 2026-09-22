"""식당 배치·충돌용 외곽·2D 주행 계획의 공통 원본. 실기체 접근 없음."""
from __future__ import annotations

import heapq
import json
import math
from pathlib import Path

import numpy as np

DEFAULT_LAYOUT = Path(__file__).resolve().parents[1]/"config/simulation/restaurant_layout.json"


def load_layout(path=DEFAULT_LAYOUT):
    layout = json.loads(Path(path).read_text())
    if layout.get("schema") != "restaurant_layout_v1":
        raise ValueError("식당 배치 스키마 오류")
    bounds = np.asarray(layout["room_bounds_m"], dtype=float)
    if bounds.shape != (4,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError("식당 경계 오류")
    for key in ("wall_height_m", "map_resolution_m", "planning_radius_m"):
        if not math.isfinite(layout[key]) or layout[key] <= 0:
            raise ValueError("식당 양수 설정 오류: "+key)
    for name, pose in layout["waypoints"].items():
        if len(pose) != 3 or not np.isfinite(pose).all():
            raise ValueError("정차 위치 오류: "+name)
    table_ids=[table["id"] for table in layout["tables"]]
    if (not table_ids or len(table_ids) != len(set(table_ids))
            or {"dock","kitchen"}.intersection(table_ids)
            or not set(table_ids).issubset(layout["waypoints"])):
        raise ValueError("테이블 ID 중복 또는 접근 목표 누락")
    for _, center, size in obstacle_boxes(layout):
        if len(center) != 2 or len(size) != 2 or not np.isfinite([*center,*size]).all() or min(size) <= 0:
            raise ValueError("장애물 크기 오류")
    required={"back_counter","banquette_south","banquette_north","plant_1","plant_2","dock_charger"}
    ids=[item["id"] for item in layout["obstacles"]]
    if len(ids) != len(set(ids)) or not required.issubset(ids):
        raise ValueError("식당 필수 가구 ID 누락 또는 중복")
    if any(item["yaw_deg"] not in (0,180) for item in layout["chairs"]):
        raise ValueError("현재 의자 충돌 외곽은 0/180도만 지원")
    return layout


def obstacle_boxes(layout):
    kitchen = layout["kitchen"]
    yield "worktop", kitchen["center_m"][:2], kitchen["size_m"][:2]
    for item in layout["tables"]:
        yield item["id"], item["center_m"], item["size_m"]
    for index, item in enumerate(layout["chairs"]):
        yield f"chair_{index}", item["center_m"], [.48,.52]
    for item in layout["obstacles"]:
        yield item["id"], item["center_m"], item["size_m"]


def occupancy(layout, padding_m=0.):
    xmin,xmax,ymin,ymax = layout["room_bounds_m"]
    resolution = layout["map_resolution_m"]
    xs = np.arange(xmin+resolution/2,xmax,resolution)
    ys = np.arange(ymin+resolution/2,ymax,resolution)
    x,y = np.meshgrid(xs,ys)
    occupied = (x < xmin+.08+padding_m)|(x > xmax-.08-padding_m)|(y < ymin+.08+padding_m)|(y > ymax-.08-padding_m)
    for _,center,size in obstacle_boxes(layout):
        occupied |= ((abs(x-center[0]) <= size[0]/2+padding_m)&(abs(y-center[1]) <= size[1]/2+padding_m))
    return occupied


def plan_route(layout, start_name, goal_name):
    """보수적 2D A*: 상판까지 투영. Nav2 실행·바퀴 동역학 검증 아님."""
    grid = occupancy(layout,layout["planning_radius_m"])
    xmin,_,ymin,_ = layout["room_bounds_m"]
    resolution = layout["map_resolution_m"]
    def cell(pose):
        return int((pose[1]-ymin)/resolution),int((pose[0]-xmin)/resolution)
    start,goal = cell(layout["waypoints"][start_name]),cell(layout["waypoints"][goal_name])
    def free(point):
        return 0 <= point[0] < grid.shape[0] and 0 <= point[1] < grid.shape[1] and not grid[point]
    if not free(start) or not free(goal):
        raise ValueError("시작 또는 목표 위치에 차체 여유 공간 부족")
    queue,previous,cost = [(0,start)],{start:None},{start:0.}
    while queue:
        _,current = heapq.heappop(queue)
        if current == goal:
            break
        for dy,dx in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt = current[0]+dy,current[1]+dx
            if not free(nxt) or (dx and dy and (not free((current[0]+dy,current[1])) or not free((current[0],current[1]+dx)))):
                continue
            candidate = cost[current]+math.hypot(dx,dy)
            if candidate < cost.get(nxt,math.inf):
                cost[nxt],previous[nxt] = candidate,current
                heapq.heappush(queue,(candidate+math.dist(nxt,goal),nxt))
    if goal not in previous:
        raise ValueError("연결 가능한 주행 경로 없음")
    cells,current = [],goal
    while current is not None:
        cells.append(current)
        current = previous[current]
    return [[xmin+(x+.5)*resolution,ymin+(y+.5)*resolution] for y,x in reversed(cells)]


def export_navigation(layout, output):
    output = Path(output)
    grid = occupancy(layout)
    image = np.where(grid[::-1],0,254).astype(np.uint8)
    (output/"restaurant.pgm").write_bytes(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode()+image.tobytes())
    xmin,_,ymin,_ = layout["room_bounds_m"]
    (output/"restaurant.yaml").write_text(f"image: restaurant.pgm\nmode: trinary\nresolution: {layout['map_resolution_m']}\norigin: [{xmin}, {ymin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    routes = []
    for target in (table["id"] for table in layout["tables"]):
        for start,end in (("dock","kitchen"),("kitchen",target),(target,"dock")):
            path = plan_route(layout,start,end)
            routes.append({"start":start,"goal":end,"path_xy_m":path,
                           "length_m":sum(math.dist(a,b) for a,b in zip(path,path[1:]))})
    report = {"schema":"restaurant_route_check_v1","routes":routes,
              "planning_radius_m":layout["planning_radius_m"],"collision_scope":"sampled_2d_inflated_layout",
              "nav2_executed":False,"base_physics_verified":False,"hardware_accessed":False}
    (output/"routes.json").write_text(json.dumps(report,indent=2)+"\n")
    return report
