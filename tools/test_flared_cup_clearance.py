"""확관 컵 하강 경로 회귀: 두 강체 패드와 속 빈 컵 벽의 표본 SAT 검사."""
import argparse
import copy
import itertools
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from bimanual_pour_plan import BimanualContactChain, base_positions, build_plan
from simulate_bimanual_pour import run
from solve_task_poses import rpy_matrix


@pytest.fixture(scope="module")
def actual_plan(tmp_path_factory):
    output=tmp_path_factory.mktemp("flared")/"plan"
    run(argparse.Namespace(output_dir=output,settle_only=False,plan_only=True))
    return output,json.loads((output/"plan.json").read_text())


def _walls(profile,wall):
    faces=((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))
    edges=sorted({tuple(sorted((face[i],face[(i+1)%4]))) for face in faces for i in range(4)})
    for sector in range(32):
        for lower,upper in zip(profile,profile[1:]):
            vertices=np.array([(r*np.cos(a),r*np.sin(a),z)
                               for z,radius in (lower,upper)
                               for r in (radius-wall,radius)
                               for a in (2*np.pi*sector/32-np.pi/32*1.02,
                                         2*np.pi*sector/32+np.pi/32*1.02)])
            normals=np.array([np.cross(vertices[f[1]]-vertices[f[0]],vertices[f[2]]-vertices[f[0]]) for f in faces])
            directions=np.array([vertices[b]-vertices[a] for a,b in edges])
            yield vertices,normals,directions


def _minimum_gap(model,manifest,plan):
    left,right=manifest["left_config"],manifest["right_config"]
    chain=BimanualContactChain(model,left,right)
    root=ET.parse(model).getroot()
    pads=[]
    corners=np.array(list(itertools.product((-1,1),repeat=3)))
    for name in ("left_gripper_link","left_moving_jaw_link"):
        for collision in root.findall(f"./link[@name='{name}']/collision"):
            box=collision.find("geometry/box")
            if box is None:
                continue
            size=np.fromstring(box.get("size"),sep=" ")
            if max(size) < .07:
                continue
            origin=collision.find("origin")
            rotation=rpy_matrix(*np.fromstring(origin.get("rpy","0 0 0"),sep=" "))
            center=np.fromstring(origin.get("xyz","0 0 0"),sep=" ")
            pads.append((name,corners*size/2 @ rotation.T+center,rotation))
    assert len(pads) == 2
    poses={pose["name"]:pose for pose in plan["poses"]}
    start=np.radians(poses["LEFT_PREGRASP_ABOVE"]["left_joint_deg"])
    end=np.radians(poses["LEFT_ALIGN_MIDDLE"]["left_joint_deg"])
    walls=list(_walls(left["cup_radius_profile_m"],left["container_wall_m"]))
    minimum=float("inf")
    for fraction in np.linspace(0,1,241):
        state=base_positions("left",start*(1-fraction)+end*fraction)
        state["left_gripper"]=left["gripper_open_rad"]
        transforms=chain.transforms(state)
        for name,vertices,local_rotation in pads:
            transform=transforms[name]
            box=vertices @ transform[:3,:3].T+transform[:3,3]-left["cup_center_m"]
            axes=(transform[:3,:3] @ local_rotation).T
            for wall,normals,edges in walls:
                cross=np.cross(axes[:,None,:],edges[None,:,:]).reshape(-1,3)
                directions=np.concatenate((axes,normals,cross))
                lengths=np.linalg.norm(directions,axis=1)
                directions=directions[lengths>1e-10]/lengths[lengths>1e-10,None]
                a,b=box @ directions.T,wall @ directions.T
                gap=np.max(np.maximum(a.min(axis=0)-b.max(axis=0),b.min(axis=0)-a.max(axis=0)))
                minimum=min(minimum,float(gap))
    return minimum


def test_flared_rim_has_clearance_and_rejects_old_backoff(actual_plan):
    output,manifest=actual_plan
    model=output/"replacement_hypothesis.urdf"
    assert _minimum_gap(model,manifest,manifest["plan"]) > .002
    old=copy.deepcopy(manifest)
    old["left_config"]["approach"]["pregrasp_backoff_m"]=.045
    plan=build_plan(model,old["left_config"],old["right_config"])
    assert _minimum_gap(model,old,plan) < 0
