#!/usr/bin/env python3
"""공통 실측 하단을 고정한 네 제품 후보의 좌표 기반 3D 개념 렌더.

SO101은 저장소 URDF/STL의 실제 축척. 카메라는 공식 외곽 치수 근사.
캐스터와 오른손 그리퍼는 프록시다. 모든 좌표 mm, 외장/접합 강도는 미검증.
"""
from pathlib import Path
import itertools
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from product_real_parts import arm_visuals

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/assets/product_candidates_20260909'
SILVER='#abb8c2'; WHITE='#e8edf0'; DARK='#33404b'; ORANGE='#e79839'; BLUE='#4885ad'
plt.rcParams['font.family']='Noto Sans CJK JP'
ARM_POSE={'shoulder_lift':-1.1,'elbow_flex':1.1,'wrist_flex':-.7}
ARM_MOUNT_Z=728.4  # STL 바닥이 root에서 -2.4 mm: 6 mm 어댑터 상면 z726에 맞춘 표시 위치.
DEFAULT_CAMERA_HEIGHT_ABOVE_DECK=540
DEFAULT_CAMERA_PITCH_DEG=43
A_BRACE_WIDTH=20
A_BRACE_THICKNESS=1.5

def poly(ax, faces, color, alpha=1):
    if hasattr(ax, 'add_faces'):
        ax.add_faces(faces,color,alpha)
        return
    ax.add_collection3d(Poly3DCollection(faces,facecolors=color,edgecolors='#64717d',linewidths=.35,alpha=alpha))

def box(ax, center, size, color=SILVER, alpha=1):
    x,y,z=center; a,b,c=np.array(size)/2
    v=np.array(list(itertools.product((x-a,x+a),(y-b,y+b),(z-c,z+c))))
    faces=[[v[i] for i in ix] for ix in ((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))]
    poly(ax,faces,color,alpha)

def bar(ax,p,q,width=7,color=ORANGE):
    ax.plot(*np.array([p,q]).T,color=color,lw=width,solid_capstyle='round')

def flat_strap_faces(p,q,width=A_BRACE_WIDTH,thickness=A_BRACE_THICKNESS,plane_normal=(0,1,0)):
    """평면 안의 폭과 평면 밖 두께를 보존한 직육면체 가새 면을 반환한다."""
    if width<=0 or thickness<=0:
        raise ValueError('flat strap width and thickness must be positive')
    start=np.asarray(p,dtype=float);end=np.asarray(q,dtype=float)
    direction=end-start
    length=np.linalg.norm(direction)
    if length==0:
        raise ValueError('flat strap endpoints must differ')
    direction/=length
    normal=np.asarray(plane_normal,dtype=float)
    normal_length=np.linalg.norm(normal)
    if normal_length==0:
        raise ValueError('flat strap plane normal must be non-zero')
    normal/=normal_length
    if not np.isclose(np.dot(direction,normal),0.0,atol=1e-9):
        raise ValueError('flat strap direction must lie in its mounting plane')
    across=np.cross(normal,direction)
    across*=width/(2*np.linalg.norm(across))
    depth=normal*thickness/2
    vertices=np.array([
        start-across-depth,start+across-depth,start+across+depth,start-across+depth,
        end-across-depth,end+across-depth,end+across+depth,end-across+depth,
    ])
    return [vertices[list(indexes)] for indexes in (
        (0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0),
    )]

def flat_strap(ax,p,q,width=A_BRACE_WIDTH,thickness=A_BRACE_THICKNESS,plane_normal=(0,1,0),color=ORANGE):
    poly(ax,flat_strap_faces(p,q,width,thickness,plane_normal),color)

def mounted_flat_strap_faces(p,q,outward_normal,width=A_BRACE_WIDTH,thickness=A_BRACE_THICKNESS):
    """프로파일 외면의 두 끝점을 받아 바깥쪽에 접하는 가새 면을 반환한다."""
    normal=np.asarray(outward_normal,dtype=float)
    normal_length=np.linalg.norm(normal)
    if normal_length==0:
        raise ValueError('mounted flat strap outward normal must be non-zero')
    normal/=normal_length
    offset=normal*thickness/2
    return flat_strap_faces(np.asarray(p)+offset,np.asarray(q)+offset,width,thickness,normal)

def mounted_flat_strap(ax,p,q,outward_normal,width=A_BRACE_WIDTH,thickness=A_BRACE_THICKNESS,color=ORANGE):
    poly(ax,mounted_flat_strap_faces(p,q,outward_normal,width,thickness),color)

def wheel(ax,x,y,z,r=35,w=30):
    t=np.linspace(0,2*np.pi,32)
    a=np.array([x+r*np.cos(t),np.full_like(t,y-w/2),z+r*np.sin(t)]).T
    b=a+np.array([0,w,0])
    poly(ax,[a,b]+[[a[i],a[i+1],b[i+1],b[i]] for i in range(len(t)-1)],DARK)

def ball(ax,x,y,z,r=8,color=SILVER):
    if hasattr(ax, 'add_sphere'):
        ax.add_sphere((x,y,z),r,color)
        return
    u,v=np.meshgrid(np.linspace(0,2*np.pi,16),np.linspace(0,np.pi,10))
    ax.plot_surface(x+r*np.cos(u)*np.sin(v),y+r*np.sin(u)*np.sin(v),z+r*np.cos(v),color=color,linewidth=0,shade=True)

def chassis(ax):
    for x in (-160,160):box(ax,(x,0,50),(20,450,20))
    for y in (-215,-60,60,215):box(ax,(0,y,50),(300,20,20))
    for x in (-130,130):box(ax,(x,0,70),(20,450,20),BLUE)
    for y in (-255,255):
        wheel(ax,110,y,35)
        box(ax,(110,np.sign(y)*230,35),(8,40,8),DARK)
    for y in (-215,215):
        ball(ax,-110,y,8)
        box(ax,(-110,y,28),(16,16,24),DARK)

def camera(ax,height_above_deck=DEFAULT_CAMERA_HEIGHT_ABOVE_DECK,pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
    # 공식 Astra Series 외곽 165(W)×48(H)×40(D) mm 안의 형상 근사.
    # 본체/받침 세부 분할과 렌즈 위치는 사진 기반이며 실측/광학 보정값 아님.
    ring=[]
    for cy,angles in [(58.5,np.linspace(-np.pi/2,np.pi/2,24)),
                      (-58.5,np.linspace(np.pi/2,3*np.pi/2,24))]:
        ring.extend([(cy+24*np.cos(t),24*np.sin(t)) for t in angles])
    ring=np.asarray(ring)
    near=np.column_stack((np.full(len(ring),20),ring))
    far=near.copy();far[:,0]=-20
    faces=[near,far]+[[near[i],near[(i+1)%len(ring)],far[(i+1)%len(ring)],far[i]] for i in range(len(ring))]
    center=np.array([-120,0,720+height_above_deck])
    pitch=np.deg2rad(pitch_deg)
    rotation=np.array(((np.cos(pitch),0,np.sin(pitch)),(0,1,0),(-np.sin(pitch),0,np.cos(pitch))))
    poly(ax,[np.asarray(f)@rotation.T+center for f in faces],DARK)
    box(ax,(-120,0,center[2]-43),(32,65,18),DARK)
    # 렌즈는 외곽 전면의 식별용 표시이며 실제 광학 중심을 뜻하지 않는다.
    for y,r in [(-45,6),(-10,5),(45,6)]:
        t=np.linspace(0,2*np.pi,32)
        lens=np.column_stack((np.full(32,20.01),y+r*np.cos(t),r*np.sin(t)))
        lens=lens@rotation.T+center
        poly(ax,[lens],'#10191e')
        local_inner=np.column_stack((np.full(32,20.02),y+.45*r*np.cos(t),.45*r*np.sin(t)))
        poly(ax,[local_inner@rotation.T+center],'#427e92')

def shared_top_and_parts(ax,camera_height_above_deck=DEFAULT_CAMERA_HEIGHT_ABOVE_DECK,camera_pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
    top_structure(ax)
    for prefix,y in [('left_',75),('right_',-75)]:
        box(ax,(0,y,723),(90,75,6),SILVER)
        for triangles,color in arm_visuals(prefix=prefix,mount_xyz_mm=(0,y,ARM_MOUNT_Z),joint_positions=ARM_POSE):
            poly(ax,triangles,color)
    camera_z=720+camera_height_above_deck
    mast_top=camera_z-24
    box(ax,(-120,0,(720+mast_top)/2),(20,20,mast_top-720),SILVER)
    camera(ax,camera_height_above_deck,camera_pitch_deg)
    bar(ax,(-60,0,720),(-120,0,820),2,SILVER)
    box(ax,(-30,0,120),(140,160,70),DARK)
    box(ax,(-20,0,185),(110,150,35),DARK)
    box(ax,(175,0,180),(60,20,8),SILVER)
    box(ax,(190,0,200),(40,55,30),DARK)
    box(ax,(35,0,550),(170,180,3),SILVER)
    box(ax,(35,0,553),(145,155,3),'#9da9ad')
    box(ax,(40,0,593),(55,55,75),WHITE)
    ball(ax,145,-100,685,9,'#ba4442')

def top_structure(ax):
    for x in (-130,130):box(ax,(x,0,708),(20,280,20))
    for y in (-130,130,-75,75):box(ax,(0,y,708),(240,20,20))
    box(ax,(0,0,719),(300,300,2),WHITE)

def tower(ax,key):
    if key in ('A','C'):
        for x,y in itertools.product((-130,130),repeat=2):box(ax,(x,y,389),(20,20,618))
        for y in (-140,140):
            outward=(0,int(np.sign(y)),0)
            mounted_flat_strap(ax,(-130,y,85),(130,y,693),outward)
            mounted_flat_strap(ax,(130,y,85),(-130,y,693),outward)
        mounted_flat_strap(ax,(-140,-130,85),(-140,130,693),(-1,0,0))
        mounted_flat_strap(ax,(-140,130,85),(-140,-130,693),(-1,0,0))
        if key=='C':
            for y in (-147,147):box(ax,(0,y,389),(294,2,618),WHITE)
            box(ax,(-147,0,389),(2,294,618),WHITE)
            box(ax,(147,0,287),(2,294,414),WHITE)
            box(ax,(147,0,674),(2,294,48),WHITE)
            for y in (-134,134):box(ax,(147,y,572),(2,26,156),WHITE)
            box(ax,(150,-85,350),(3,6,45),DARK)
    elif key=='B':
        for y in (-130,130):
            box(ax,(0,y,389),(40,20,618),WHITE)
            for x in (-130,130):
                poly(ax,[[(x,y,80),(0,y,80),(0,y,300)]],SILVER)
        box(ax,(-100,0,660),(2,260,70),SILVER)
    else:
        # 두 절곡 측판을 U형 부재 조합으로 단순 표현: 창을 남기는 가장자리와 허리 리브.
        for y in (-140,140):
            for x in (-115,115):box(ax,(x,y,389),(40,2,618),WHITE)
            for z in (100,389,678):box(ax,(0,y,z),(260,2,40),WHITE)
            for x in (-130,130):box(ax,(x,y-12*np.sign(y),389),(2,26,618),SILVER)
        box(ax,(-132,0,390),(2,260,110),SILVER)

def render(ax,key):
    chassis(ax);tower(ax,key);shared_top_and_parts(ax)
    # 비교의 기준을 실제 좌표에 표시. 정면 방향 x>0에서 y가 좌우 폭이다.
    for length,x,z,label in [(300,185,785,'상판 300'),(450,185,-15,'하단 450'),(540,210,-70,'바퀴 포함 540')]:
        ax.plot([x,x],[-length/2,length/2],[z,z],color=BLUE,lw=1)
        ax.text(x,0,z-25,label,ha='center',fontsize=8,color=BLUE)
    ax.set_xlim(-290,290);ax.set_ylim(-310,310);ax.set_zlim(-110,1350)
    ax.set_box_aspect((580,620,1460),zoom=1.0);ax.set_proj_type('ortho');ax.view_init(elev=13,azim=18)
    ax.set_axis_off()

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    titles={'A':'A · 노출 프레임','B':'B · 두 기둥','C':'C · 프레임 + 외장','D':'D · 절곡 판금'}
    fig=plt.figure(figsize=(14,16))
    for i,(key,title) in enumerate(titles.items(),1):
        ax=fig.add_subplot(2,2,i,projection='3d');render(ax,key)
        ax.set_title(title,fontsize=15,y=.95)
    fig.suptitle('상판 300 / 하단 450 / 바퀴 포함 540 — 동일 좌표·축척의 제품 후보',fontsize=20,y=.98)
    fig.text(.5,.065,'SO101 원본 축척 · Astra 외곽 165×48×40 · 오른손/캐스터 근사 · 구조/접지/파지 검증 전',ha='center',fontsize=11)
    fig.subplots_adjust(left=0,right=1,top=.91,bottom=.09,wspace=0)
    fig.savefig(OUT/'dimension_locked_candidates.png',dpi=180)
    fig.savefig(OUT/'dimension_locked_candidates.svg')
    plt.close(fig)
    for key,title in titles.items():
        fig=plt.figure(figsize=(8,11));ax=fig.add_subplot(111,projection='3d');render(ax,key)
        ax.set_title(title+' — 치수 고정 입체도',fontsize=16)
        fig.text(.5,.04,'상판 300×300 / 하단 450×340 / 전체 폭 540 mm\nSO101 원본 축척 · 카메라 외곽 근사 · 제조 도면 아님',ha='center',fontsize=11)
        fig.savefig(OUT/f'{key}_dimension_locked.png',dpi=170);plt.close(fig)
    contract={'units':'mm','base_frame':[450,340],'deck':[300,300],'deck_z':720,
              'wheel_outer_width':540,'deck_side_inset':75,'deck_fore_aft_inset':20,
              'drive_centers':[[110,-255,35],[110,255,35]],'assumed_wheel_width':30,
              'caster_xy':[[-110,-215],[-110,215]],'tower_post_xy':[[-130,-130],[-130,130],[130,-130],[130,130]],
              'a_brace_flat_bar':[A_BRACE_WIDTH,A_BRACE_THICKNESS],
              'camera_envelope_whd':[165,48,40],
              'camera_center_height_above_deck':DEFAULT_CAMERA_HEIGHT_ABOVE_DECK,
              'scope':'SO101 URDF/STL 원본 축척. 카메라 외곽 근사. 오른손/캐스터 프록시; 자세는 시각화용'}
    (OUT/'geometry_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n')
    print(OUT/'dimension_locked_candidates.png')

if __name__=='__main__':main()
