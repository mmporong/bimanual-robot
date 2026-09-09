#!/usr/bin/env python3
"""기존 후보 형상을 VTK 깊이 버퍼로 렌더. 원본 STL 면과 축척을 보존한다."""
import json
import math
from pathlib import Path
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
import render_product_candidates as model
from product_real_parts import arm_metadata

OUT=model.OUT
REAL_OUT=OUT/'Real_scale'
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'

def rgb(value):
    return tuple(int(value[i:i+2],16)/255 for i in (1,3,5))

class Scene:
    def __init__(self,renderer): self.renderer=renderer

    def actor(self,data,color,alpha=1):
        mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(data)
        actor=vtk.vtkActor();actor.SetMapper(mapper)
        prop=actor.GetProperty();prop.SetColor(*rgb(color));prop.SetOpacity(alpha)
        prop.SetAmbient(.28);prop.SetDiffuse(.72);prop.SetSpecular(.15);prop.SetSpecularPower(18)
        self.renderer.AddActor(actor)

    def add_faces(self,faces,color,alpha=1):
        lengths=np.array([len(f) for f in faces],dtype=np.int64)
        points=np.concatenate(faces).astype(np.float64)
        offsets=np.concatenate(([0],np.cumsum(lengths))).astype(np.int64)
        cells=vtk.vtkCellArray()
        cells.SetData(numpy_to_vtkIdTypeArray(offsets,deep=True),numpy_to_vtkIdTypeArray(np.arange(len(points),dtype=np.int64),deep=True))
        data=vtk.vtkPolyData();pts=vtk.vtkPoints();pts.SetData(numpy_to_vtk(points,deep=True));data.SetPoints(pts);data.SetPolys(cells)
        self.actor(data,color,alpha)

    def add_sphere(self,center,radius,color):
        source=vtk.vtkSphereSource();source.SetCenter(*center);source.SetRadius(radius)
        source.SetThetaResolution(24);source.SetPhiResolution(16);source.Update()
        self.actor(source.GetOutput(),color)

    def plot(self,x,y,z,color,lw=1,**kwargs):
        source=vtk.vtkLineSource();source.SetPoint1(x[0],y[0],z[0]);source.SetPoint2(x[-1],y[-1],z[-1])
        tube=vtk.vtkTubeFilter();tube.SetInputConnection(source.GetOutputPort());tube.SetRadius(max(.6,lw*.35));tube.SetNumberOfSides(8);tube.Update()
        self.actor(tube.GetOutput(),color)

def label(renderer,text,x,y,size=22):
    actor=vtk.vtkTextActor();actor.SetInput(text)
    actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport();actor.SetPosition(x,y)
    prop=actor.GetTextProperty();prop.SetFontFamily(vtk.VTK_FONT_FILE);prop.SetFontFile(FONT)
    prop.SetFontSize(size);prop.SetColor(.12,.17,.20)
    renderer.AddActor2D(actor)

def add_scene(window,key,viewport=(0,0,1,1),detail=False,camera_height_mm=model.DEFAULT_CAMERA_HEIGHT_ABOVE_DECK,
              camera_pitch_deg=model.DEFAULT_CAMERA_PITCH_DEG):
    renderer=vtk.vtkRenderer();renderer.SetViewport(*viewport);renderer.SetBackground(1,1,1);window.AddRenderer(renderer)
    scene=Scene(renderer)
    model.chassis(scene);model.tower(scene,key);model.shared_top_and_parts(scene,camera_height_mm,camera_pitch_deg)
    camera=renderer.GetActiveCamera();camera.ParallelProjectionOn();camera.SetViewUp(0,0,1)
    if detail:
        camera.SetFocalPoint(35,0,990);camera.SetPosition(1300,-850,1350);camera.SetParallelScale(390)
    else:
        camera.SetFocalPoint(0,0,620);camera.SetPosition(1900,-1050,1350);camera.SetParallelScale(770)
    renderer.ResetCameraClippingRange()
    titles={'A':'A · 노출 프레임','B':'B · 두 기둥','C':'C · 프레임 + 외장','D':'D · 절곡 판금'}
    label(renderer,titles[key]+(' · 상부 확대' if detail else ' · 실제 축척 부품'),.06,.94,26)
    label(renderer,'SO101: URDF/STL 원본 크기',.06,.89,18)
    label(renderer,'Astra: 165(W) × 48(H) × 40(D) mm 외곽 근사',.06,.855,17)
    label(renderer,f'Astra 중심: 상판 +{camera_height_mm} mm / 바닥 {720+camera_height_mm} mm',.06,.82,17)
    label(renderer,f'표시 하향각: {camera_pitch_deg}°',.06,.785,17)
    if not detail:
        label(renderer,'상판 300×300 / 하단 450×340 / 바퀴 포함 540 mm',.06,.06,18)
    label(renderer,'자세·접합·충돌 미검증 / 오른손 그리퍼는 치수 근사',.06,.025,16)
    return renderer

def save(window,path):
    window.Render()
    capture=vtk.vtkWindowToImageFilter();capture.SetInput(window);capture.ReadFrontBufferOff();capture.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(path));writer.SetInputConnection(capture.GetOutputPort());writer.Write()
    window.Finalize()

def window(width,height):
    w=vtk.vtkRenderWindow();w.SetOffScreenRendering(1);w.SetSize(width,height);w.SetMultiSamples(4)
    return w

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    REAL_OUT.mkdir(parents=True,exist_ok=True)
    for key in 'ABCD':
        w=window(1200,1500);add_scene(w,key);save(w,REAL_OUT/f'{key}_real_scale.png')
    w=window(2000,2200)
    for key,vp in zip('ABCD',[(0,.5,.5,1),(.5,.5,1,1),(0,0,.5,.5),(.5,0,1,.5)]):add_scene(w,key,vp)
    save(w,REAL_OUT/'real_scale_candidates.png')
    w=window(1400,1100);add_scene(w,'B',detail=True);save(w,REAL_OUT/'B_real_scale_detail.png')
    w=window(1900,1350)
    add_scene(w,'B',(0,0,.5,1),camera_height_mm=380,camera_pitch_deg=30)
    add_scene(w,'B',(.5,0,1,1),camera_height_mm=540,camera_pitch_deg=43)
    save(w,REAL_OUT/'B_camera_height_380_540_comparison.png')
    metadata={p:arm_metadata(prefix=p,mount_xyz_mm=(0,y,model.ARM_MOUNT_Z),joint_positions=model.ARM_POSE) for p,y in [('left_',75),('right_',-75)]}
    for arm in metadata.values():
        arm['source_urdf']=str(Path(arm['source_urdf']).relative_to(model.ROOT))
    metadata['camera']={'envelope_whd_mm':[165,48,40], 'status':'공식 Astra Series 외곽; 세부 형상·렌즈 위치는 근사',
                        'candidate_center_heights_above_deck_mm':[380,540],
                        'recommended_center_height_above_deck_mm':540,
                        'initial_pitch_down_deg':model.DEFAULT_CAMERA_PITCH_DEG,
                        'source':'https://www.orbbec.com/products/structured-light-camera/astra-series/'}
    beta=45.5/2
    compared={}
    for height,pitch in ((380,30),(540,43)):
        limits=[]
        for target_x in (443,492):
            distance=target_x-(-120)
            limits.append(round(height-distance*math.tan(math.radians(pitch-beta)),1))
        compared[f'deck_plus_{height}mm_pitch_{pitch}deg_max_object_height_mm']=limits
    metadata['camera']['compared_configurations']={
        **compared,
        'target_x_from_chassis_center_mm':[443,492],
        'camera_x_from_chassis_center_mm':-120,
    }
    (OUT/'real_parts_provenance.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    contract={'units':'mm','base_frame':[450,340],'deck':[300,300],'deck_z':720,
              'wheel_outer_width':540,'deck_side_inset':75,'deck_fore_aft_inset':20,
              'drive_centers':[[110,-255,35],[110,255,35]],'assumed_wheel_width':30,
              'caster_xy':[[-110,-215],[-110,215]],'camera_envelope_whd':[165,48,40],
              'camera_center_height_above_deck_candidates':[380,540],
              'recommended_camera_center_height_above_deck':540,
              'initial_camera_pitch_down_deg':model.DEFAULT_CAMERA_PITCH_DEG,
              'scope':'SO101 URDF/STL 원본 축척. 카메라 외곽 근사. 오른손/캐스터 프록시; 자세는 시각화용'}
    (OUT/'geometry_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n')
    print(REAL_OUT/'real_scale_candidates.png')

if __name__=='__main__':main()
