import copy
import json

import numpy as np
import pytest

from restaurant_layout import load_layout, occupancy, plan_route, export_navigation


def test_all_service_legs_have_clearance_and_no_corner_cutting():
    layout=load_layout()
    grid=occupancy(layout,layout["planning_radius_m"])
    xmin,_,ymin,_=layout["room_bounds_m"]
    res=layout["map_resolution_m"]
    assert len(layout["tables"]) == 4
    for target in (table["id"] for table in layout["tables"]):
        for start,goal in (("dock","kitchen"),("kitchen",target),(target,"dock")):
            path=np.asarray(plan_route(layout,start,goal))
            cells=np.floor((path-[xmin,ymin])/res).astype(int)
            assert len(path)>2
            assert not grid[cells[:,1],cells[:,0]].any()
            assert np.max(np.linalg.norm(np.diff(path,axis=0),axis=1)) <= res*2**.5+1e-10
            assert np.linalg.norm(path[-1]-layout["waypoints"][goal][:2]) <= res
            for a,b in zip(cells,cells[1:]):
                if np.all(a!=b):
                    assert not grid[a[1],b[0]] and not grid[b[1],a[0]]


def test_blocked_goal_is_not_reported_as_success():
    layout=copy.deepcopy(load_layout())
    layout["waypoints"]["kitchen"]=[1.16,0,0]
    with pytest.raises(ValueError):plan_route(layout,"dock","kitchen")


def test_closed_corridor_is_rejected():
    layout=copy.deepcopy(load_layout())
    layout["obstacles"].append({"id":"blocker","center_m":[-1,0],"size_m":[.2,7]})
    with pytest.raises(ValueError):plan_route(layout,"dock","kitchen")


def test_export_map_is_uninflated_and_claims_only_route_planning(tmp_path):
    layout=load_layout()
    report=export_navigation(layout,tmp_path)
    assert len(report["routes"]) == 12
    assert {r["goal"] for r in report["routes"] if r["start"] == "kitchen"} == {t["id"] for t in layout["tables"]}
    assert not report["nav2_executed"] and not report["base_physics_verified"]
    raw=(tmp_path/"restaurant.pgm").read_bytes()
    header=b"P5\n170 140\n255\n"
    assert raw.startswith(header)
    values=np.frombuffer(raw[len(header):],dtype=np.uint8).reshape(140,170)
    assert np.array_equal(values==0,occupancy(layout)[::-1])
    saved=json.loads((tmp_path/"routes.json").read_text())
    assert saved == report
    assert len(saved["routes"]) == 12


def test_missing_table_waypoint_rejected(tmp_path):
    layout=load_layout()
    del layout["waypoints"]["table_4"]
    path=tmp_path/"missing.json"
    path.write_text(json.dumps(layout))
    with pytest.raises(ValueError):
        load_layout(path)


def test_waypoint_nan_rejected(tmp_path):
    layout=load_layout();layout["waypoints"]["dock"][0]=float("nan")
    path=tmp_path/"bad.json";path.write_text(json.dumps(layout))
    with pytest.raises(ValueError):load_layout(path)


@pytest.mark.parametrize("mutation",["missing","duplicate","chair_rotation"])
def test_unsupported_scene_layout_rejected(tmp_path,mutation):
    layout=load_layout()
    if mutation == "missing":
        layout["obstacles"].pop()
    elif mutation == "duplicate":
        layout["obstacles"].append(layout["obstacles"][0])
    else:
        layout["chairs"][0]["yaw_deg"]=45
    path=tmp_path/"bad.json"
    path.write_text(json.dumps(layout))
    with pytest.raises(ValueError):
        load_layout(path)
