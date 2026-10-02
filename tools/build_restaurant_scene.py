#!/usr/bin/env python3
"""Isaac Sim 5.1 발표용 식당 USD·지도·카메라 렌더. 실물 접근 없음."""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import shutil

from restaurant_layout import load_layout, export_navigation, obstacle_boxes


def validate_scene_geometry(stage, layout):
    """렌더 충돌체의 XY 외곽이 지도 장애물 안에 포함되는지 검사한다."""
    from pxr import Usd, UsdGeom, UsdPhysics
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_])
    bounds=[(name,[c[0]-s[0]/2,c[1]-s[1]/2],[c[0]+s[0]/2,c[1]+s[1]/2])
            for name,c,s in obstacle_boxes(layout)]
    matches=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if not path.startswith("/Restaurant/") or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if path == "/Restaurant/Floor" or path.startswith("/Restaurant/Walls/"):
            continue
        extent=cache.ComputeWorldBound(prim).ComputeAlignedRange()
        low,high=extent.GetMin(),extent.GetMax()
        owners=[name for name,a,b in bounds if all(a[i]-1e-5 <= low[i] and high[i] <= b[i]+1e-5 for i in (0,1))]
        if not owners:
            raise ValueError("지도 장애물 외곽을 벗어난 장면 충돌체: "+path)
        matches.append({"prim":path,"map_obstacles":owners})
    if not matches:
        raise ValueError("식당 가구 충돌체가 없음")
    return matches


def build(stage, layout, floor_texture=None):
    from pxr import Gf, Sdf, Tf, UsdGeom, UsdLux, UsdPhysics, UsdShade
    root = "/Restaurant"
    UsdGeom.Xform.Define(stage, root)
    def prim_path(name):
        return root+"/"+"/".join(Tf.MakeValidIdentifier(part) for part in name.split("/"))
    palette = {"ivory":(.82,.78,.68),"oak":(.48,.27,.12),"sage":(.19,.29,.23),
               "brass":(.52,.34,.13),"dark":(.055,.065,.06),"stone":(.58,.57,.51),
               "white":(.91,.88,.79),"soil":(.045,.027,.012),"leaf":(.08,.23,.10),
               "warm_light":(1.,.72,.35)}
    materials = {}
    for name,color in palette.items():
        material = UsdShade.Material.Define(stage,root+"/Materials/"+name)
        shader = UsdShade.Shader.Define(stage,material.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.25 if name == "brass" else .55)
        shader.CreateInput("metallic",Sdf.ValueTypeNames.Float).Set(.8 if name == "brass" else 0.)
        if name == "warm_light":
            shader.CreateInput("emissiveColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(3,2.2,1.1))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
        materials[name] = material

    def finish(shape,center,scale,material,collision=True,rotation=None):
        shape.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        if rotation is not None:
            shape.AddRotateXYZOp().Set(Gf.Vec3f(*map(float,rotation)))
        if scale is not None:
            shape.AddScaleOp().Set(Gf.Vec3f(*map(float,scale)))
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(materials[material])
        if collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        return shape

    def box(name,center,size,material,collision=True,rotation=None):
        shape=UsdGeom.Cube.Define(stage,prim_path(name))
        shape.CreateSizeAttr(1.)
        return finish(shape,center,size,material,collision,rotation)

    def cylinder(name,center,radius,height,material,collision=True,rotation=None):
        shape=UsdGeom.Cylinder.Define(stage,prim_path(name))
        shape.CreateRadiusAttr(float(radius));shape.CreateHeightAttr(float(height));shape.CreateAxisAttr("Z")
        return finish(shape,center,None,material,collision,rotation)

    def sphere(name,center,scale,material,rotation=None):
        shape=UsdGeom.Sphere.Define(stage,prim_path(name))
        shape.CreateRadiusAttr(1.)
        return finish(shape,center,scale,material,False,rotation)

    def rounded_top(name,center,width,depth,thickness,material):
        # 네 모서리 원호를 갖는 실제 메쉬. 재질만으로 둥근 모양을 흉내내지 않는다.
        radius=min(width,depth)*.10
        outline=[]
        for cx,cy,start in ((width/2-radius,depth/2-radius,0),(-width/2+radius,depth/2-radius,90),
                            (-width/2+radius,-depth/2+radius,180),(width/2-radius,-depth/2+radius,270)):
            for n in range(9):
                angle=math.radians(start+n*90/8)
                outline.append((cx+radius*math.cos(angle),cy+radius*math.sin(angle)))
        count=len(outline)
        points=[(x,y,z) for z in (-thickness/2,thickness/2) for x,y in outline]
        indices=list(reversed(range(count)))+list(range(count,2*count))
        for i in range(count):
            j=(i+1)%count;indices.extend([i,j,j+count,i+count])
        shape=UsdGeom.Mesh.Define(stage,prim_path(name))
        shape.CreatePointsAttr(points);shape.CreateFaceVertexCountsAttr([count,count]+[4]*count)
        shape.CreateFaceVertexIndicesAttr(indices);shape.CreateSubdivisionSchemeAttr("none")
        finish(shape,center,None,material)
        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")

    xmin,xmax,ymin,ymax=layout["room_bounds_m"]
    height=layout["wall_height_m"]
    midx,midy=(xmin+xmax)/2,(ymin+ymax)/2
    obstacles={item["id"]:item for item in layout["obstacles"]}
    known={"back_counter","banquette_south","banquette_north","plant_1","plant_2","dock_charger"}
    for name,item in obstacles.items():
        if name not in known:
            box("AdditionalObstacles/"+name,[*item["center_m"],.35],[*item["size_m"],.70],"dark")
    box("Floor",[midx,midy,-.06],[xmax-xmin,ymax-ymin,.12],"stone")
    if floor_texture:
        surface=UsdGeom.Mesh.Define(stage,root+"/WoodFloor")
        surface.CreatePointsAttr([(xmin,ymin,.008),(xmax,ymin,.008),(xmax,ymax,.008),(xmin,ymax,.008)])
        surface.CreateFaceVertexCountsAttr([4]);surface.CreateFaceVertexIndicesAttr([0,1,2,3])
        surface.CreateSubdivisionSchemeAttr("none")
        UsdGeom.PrimvarsAPI(surface).CreatePrimvar("st",Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set([(0,0),(3,0),(3,3.5),(0,3.5)])
        material=UsdShade.Material.Define(stage,root+"/Materials/WoodFloor")
        shader=UsdShade.Shader.Define(stage,material.GetPath().AppendChild("Surface"));shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.65)
        reader=UsdShade.Shader.Define(stage,material.GetPath().AppendChild("UV"));reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname",Sdf.ValueTypeNames.Token).Set("st")
        texture=UsdShade.Shader.Define(stage,material.GetPath().AppendChild("Texture"));texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file",Sdf.ValueTypeNames.Asset).Set(str(floor_texture))
        texture.CreateInput("sourceColorSpace",Sdf.ValueTypeNames.Token).Set("sRGB")
        texture.CreateInput("wrapS",Sdf.ValueTypeNames.Token).Set("repeat")
        texture.CreateInput("wrapT",Sdf.ValueTypeNames.Token).Set("repeat")
        texture.CreateInput("st",Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),"result")
        shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(),"rgb")
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
        UsdShade.MaterialBindingAPI.Apply(surface.GetPrim()).Bind(material)
    # 좁은 이음매와 번갈아 이어지는 마루. 전체 floor collider 하나만 사용한다.
    for row in range(28):
        y=ymin+(row+.5)*(ymax-ymin)/28
        for col in range(5):
            x=xmin+(col+.5)*(xmax-xmin)/5
            box(f"Floorboards/R{row}C{col}",[x,y,.003],[(xmax-xmin)/5-.008,(ymax-ymin)/28-.004,.006],"ivory",False)
    box("Walls/East",[xmax,midy,height/2],[.12,ymax-ymin+.12,height],"ivory")
    box("Walls/North",[midx,ymax,height/2],[xmax-xmin+.12,.12,height],"ivory")
    # 발표용 절개 벽: 물리 경계는 보존하고 렌더에서만 숨긴다.
    for name,center,size in (("South",[midx,ymin,height/2],[xmax-xmin+.12,.12,height]),("West",[xmin,midy,height/2],[.12,ymax-ymin+.12,height])):
        wall=box("Walls/"+name,center,size,"ivory")
        wall.CreateVisibilityAttr("invisible")
    box("Skirting/East",[xmax-.075,midy,.08],[.035,ymax-ymin,.16],"oak",False)
    box("Skirting/North",[midx,ymax-.075,.08],[xmax-xmin,.035,.16],"oak",False)

    # 주방: 검증된 작업대 높이는 유지하고 배경 수납·싱크·선반을 별도로 둔다.
    counter=obstacles["back_counter"]
    cx,cy=counter["center_m"];cw,cd=counter["size_m"]
    for i in range(8):
        y=cy-cd/2+(i+.5)*cd/8
        box(f"Kitchen/Cabinet{i}",[cx,y,.38],[cw-.04,cd/8-.02,.76],"sage")
        box(f"Kitchen/Handle{i}",[cx-cw/2+.012,y,.65],[.018,.18,.018],"brass",False)
    box("Kitchen/BackWorktop",[cx,cy,.79],[cw,cd,.055],"white")
    k=layout["kitchen"]
    rounded_top("Kitchen/PourWorktop",k["center_m"],k["size_m"][0],k["size_m"][1],k["size_m"][2],"white")
    for i,sign in enumerate((-1,1)):
        box(f"Kitchen/WorkLeg{i}",[k["center_m"][0]+k["size_m"][0]/2-.09,k["center_m"][1]+sign*(k["size_m"][1]/2-.10),.35],[.06,.06,.70],"oak")
    for shelf in range(2):
        z=1.45+shelf*.46
        box(f"Kitchen/Shelf{shelf}",[cx+.11,cy,z],[.34,cd-.6,.045],"oak",False)
        for j in range(9):
            y=cy-cd*.38+j*cd*.76/8
            cylinder(f"Kitchen/Jar{shelf}_{j}",[cx+.07,y,z+.10],.045,.16,"white",False)
            cylinder(f"Kitchen/Lid{shelf}_{j}",[cx+.07,y,z+.19],.049,.015,"oak",False)
    box("Kitchen/Sink",[cx-.03,cy-cd*.30,.825],[.32,.48,.02],"dark",False)
    cylinder("Kitchen/Faucet",[cx+.11,cy-cd*.30,.96],.014,.26,"brass",False)
    box("Kitchen/CoffeeMachine",[cx-.04,cy+cd*.33,.995],[.32,.52,.36],"dark",False)
    box("Kitchen/CoffeeFront",[cx-.211,cy+cd*.33,1.01],[.016,.41,.21],"brass",False)
    cylinder("Kitchen/CoffeeCup",[cx-.26,cy+cd*.33,.86],.034,.075,"white",False)

    for table in layout["tables"]:
        name=table["id"];x,y=table["center_m"];w,d=table["size_m"];z=table["height_m"]
        rounded_top(f"Dining/{name}/Top",[x,y,z-.025],w,d,.05,"oak")
        cylinder(f"Dining/{name}/Pedestal",[x,y,.35],.075,.68,"dark")
        cylinder(f"Dining/{name}/Foot",[x,y,.025],.26,.05,"dark")
        for side in (-1,1):
            cylinder(f"Dining/{name}/Plate{side}",[x+side*.34,y,z+.006],.12,.012,"white",False)
            box(f"Dining/{name}/Napkin{side}",[x+side*.34,y-.19,z+.005],[.17,.10,.008],"sage",False)
        cylinder(f"Dining/{name}/Vase",[x,y,z+.09],.045,.18,"ivory",False)
        for j in range(5):
            sphere(f"Dining/{name}/Flower{j}",[x+.025*math.cos(j),y+.025*math.sin(j),z+.21+j*.006],[.025,.025,.022],"white")
    for i,chair in enumerate(layout["chairs"]):
        x,y=chair["center_m"];sign=1 if chair["yaw_deg"] == 0 else -1
        rounded_top(f"Chairs/C{i}/Seat",[x,y,.45],.45,.44,.065,"sage")
        rounded_top(f"Chairs/C{i}/Back",[x,y-sign*.20,.71],.44,.06,.37,"oak")
        for dx in (-.17,.17):
            for dy in (-.16,.16):
                cylinder(f"Chairs/C{i}/Leg{int(dx*100)}_{int(dy*100)}",[x+dx,y+dy,.225],.018,.45,"oak")
    for i,key in enumerate(("banquette_south","banquette_north")):
        item=obstacles[key];x,y=item["center_m"];w,d=item["size_m"]
        box(f"Bench/B{i}/Base",[x,y,.23],[w,d,.46],"oak")
        box(f"Bench/B{i}/Seat",[x,y,.48],[w,d,.10],"sage")
        box(f"Bench/B{i}/Back",[x,y+(-1 if i == 0 else 1)*(d/2-.06),.73],[w,.12,.47],"sage")
        for j in range(5):
            box(f"Bench/B{i}/Seam{j}",[x-w/2+(j+.5)*w/5,y,.535],[.008,d-.02,.005],"oak",False)
    for i,key in enumerate(("plant_1","plant_2")):
        item=obstacles[key];x,y=item["center_m"];r=min(item["size_m"])*.38
        cylinder(f"Plants/P{i}/Pot",[x,y,.20],r,.40,"ivory")
        cylinder(f"Plants/P{i}/Soil",[x,y,.405],r*.9,.012,"soil",False)
        for j in range(12):
            angle=j*137.5; a=math.radians(angle);z=.5+(j%4)*.15
            cylinder(f"Plants/P{i}/Stem{j}",[x,y,z/2+.20],.007,z-.38,"leaf",False)
            sphere(f"Plants/P{i}/Leaf{j}",[x+.10*math.cos(a),y+.10*math.sin(a),z+.12],[.06,.19,.025],"leaf",[25,20,angle])
    # 서비스 안내 패널과 충전 구역. 실제 충전 기능을 의미하지 않는다.
    dock=obstacles["dock_charger"];dx,dy=dock["center_m"];dw,dd=dock["size_m"]
    box("Dock/Pad",[dx+.40,dy,.012],[.70,.80,.024],"sage",False)
    box("Dock/Charger",[dx,dy,.22],[dw,dd,.44],"dark")
    box("Dock/Indicator",[dx+dw/2+.008,dy,.33],[.01,.25,.025],"warm_light",False)
    box("Brand/Panel",[xmax-.075,midy,2.23],[.035,2.25,.52],"sage",False)
    for i in range(17):
        box(f"Brand/Fluting{i}",[xmax-.102,midy-1.0+i*.125,2.23],[.023,.013,.45],"brass",False)

    pendant_positions=[table["center_m"] for table in layout["tables"]]+[[.10,0]]
    for i,(x,y) in enumerate(pendant_positions):
        cylinder(f"Lighting/Pendant{i}",[x,y,2.35],.19,.08,"brass",False)
        cylinder(f"Lighting/Diffuser{i}",[x,y,2.30],.165,.012,"warm_light",False)
        cylinder(f"Lighting/Cable{i}",[x,y,2.58],.006,.40,"dark",False)
        light=UsdLux.DiskLight.Define(stage,root+f"/Lighting/Light{i}")
        light.AddTranslateOp().Set(Gf.Vec3d(x,y,2.29));light.CreateRadiusAttr(.16)
        light.CreateIntensityAttr(650);light.CreateColorAttr(Gf.Vec3f(1,.84,.66))
    dome=UsdLux.DomeLight.Define(stage,root+"/Lighting/Daylight");dome.CreateIntensityAttr(450)
    key=UsdLux.DistantLight.Define(stage,root+"/Lighting/Sun")
    key.CreateIntensityAttr(2200);key.CreateAngleAttr(12)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,-35))
    for y in (-2.4,2.4):
        light=UsdLux.RectLight.Define(stage,root+f"/Lighting/Window{int((y+3)*10)}")
        light.AddTranslateOp().Set(Gf.Vec3d(-4.2,y,2.1));light.AddRotateXYZOp().Set(Gf.Vec3f(0,-90,0))
        light.CreateWidthAttr(2.0);light.CreateHeightAttr(1.5);light.CreateIntensityAttr(600)
    return root


def run(args):
    layout=load_layout(args.layout)
    output=args.output_dir.resolve()
    if output.exists():
        raise FileExistsError("새 출력 폴더 필요")
    if shutil.disk_usage(output.parent).free < 2*1024**3:
        raise RuntimeError("장면 생성 여유 공간 2 GiB 미만")
    output.mkdir(parents=True)
    routes=export_navigation(layout,output)
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":args.headless,"width":1600,"height":1000,"renderer":"RayTracedLighting","anti_aliasing":3,"multi_gpu":False,"fast_shutdown":True})
    success=False
    try:
        import omni.usd
        import omni.kit.app
        import carb
        carb.settings.get_settings().set("/app/viewport/grid/enabled",False)
        carb.settings.get_settings().set("/persistent/app/viewport/displayOptions",0)
        from pxr import Usd, UsdGeom, UsdPhysics
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        import numpy as np
        omni.usd.get_context().new_stage()
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdPhysics.Scene.Define(stage,"/PhysicsScene")
        texture=args.floor_texture.resolve(strict=True) if args.floor_texture else None
        build(stage,layout,texture)
        collision_map_matches=validate_scene_geometry(stage,layout)
        reference=args.robot_scene.resolve(strict=True)
        robot=stage.DefinePrim("/Robot","Xform")
        robot.GetReferences().AddReference(str(reference),"/hold_flow")
        for name in ("Cup","Bottle"):
            prim=stage.DefinePrim("/Props/"+name,"Xform")
            prim.GetReferences().AddReference(str(reference),"/World/"+name)
        # 정지 프리뷰: 기존 장면의 초기 pose만 참조한다. 움직임을 성공으로 표시하지 않는다.
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        stage.SetDefaultPrim(stage.GetPrimAtPath("/Restaurant"))
        for name,view in layout["presentation_cameras"].items():
            camera=UsdGeom.Camera.Define(stage,"/Cameras/"+name)
            camera.CreateFocalLengthAttr(18 if name in ("overview","floorplan") else 24)
            camera.CreateClippingRangeAttr((.01,100.))
            set_camera_view(np.array(view["eye"]),np.array(view["target"]),camera_prim_path=str(camera.GetPath()))
        stage.GetRootLayer().Export(str(output/"restaurant.usda"))
        async def capture(path):
            await capture_viewport_to_file(get_active_viewport(),str(path)).wait_for_result()
        for name,view in layout["presentation_cameras"].items():
            get_active_viewport().camera_path="/Cameras/"+name
            for _ in range(90):app.update()
            task=asyncio.ensure_future(capture(output/(name+".png")))
            while not task.done():app.update()
            task.result()
            print("RESTAURANT_VIEW",name,flush=True)
        check=Usd.Stage.Open(str(output/"restaurant.usda"))
        if not check or any(not check.GetPrimAtPath(path) or not check.GetPrimAtPath(path).GetChildren()
                            for path in ("/Robot","/Props/Cup","/Props/Bottle")):
            raise RuntimeError("USD 재개방 또는 로봇·컵·병 참조 오류")
        manifest={"schema":"restaurant_scene_v1","layout":layout,"reference_scene":str(reference),
                  "reference_sha256":hashlib.sha256(reference.read_bytes()).hexdigest(),
                  "layout_sha256":hashlib.sha256(args.layout.read_bytes()).hexdigest(),
                  "builder_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  "prim_count":len(list(stage.Traverse())),"route_count":len(routes["routes"]),
                  "collision_map_matches":collision_map_matches,
                  "hardware_accessed":False,"nav2_executed":False,"full_service_verified":False,
                  "robot_mode":"static_reference_not_mobile_dynamics",
                  "assets":"project_authored_geometry_and_existing_project_robot",
                  "floor_texture":None if texture is None else {"path":str(texture),"sha256":hashlib.sha256(texture.read_bytes()).hexdigest(),
                      "source":"https://polyhaven.com/a/wood_floor","license":"CC0"},
                  "scene_reopened":True}
        (output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
        success=True
    finally:
        import omni.kit.app
        omni.kit.app.get_app().post_quit(0 if success else 2)
        app.close()


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout",type=Path,default=Path(__file__).resolve().parents[1]/"config/simulation/restaurant_layout.json")
    parser.add_argument("--robot-scene",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--floor-texture",type=Path)
    run(parser.parse_args())
