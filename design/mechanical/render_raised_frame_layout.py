#!/usr/bin/env python3
"""새 개념안의 정투상 배치도 생성. 부품은 간섭 검토 전의 외곽/기호다."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import yaml

ROOT=Path(__file__).resolve().parents[2]
D=yaml.safe_load(Path(__file__).with_name('raised_frame_concept_20260909.yaml').read_text())['geometry']
plt.rcParams['font.family']='Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,3,figsize=(17,11),gridspec_kw={'width_ratios':[1.15,1,1.1]})
blue='#176ca4'; grey='#b9c1c9'; green='#2b7953'

def box(ax,x,y,w,h,color=grey,alpha=1):
    ax.add_patch(Rectangle((x,y),w,h,facecolor=color,edgecolor='#3d4852',lw=1,alpha=alpha))

def dimension(ax,a,b,label,offset=(0,0)):
    ax.annotate('',xy=a,xytext=b,arrowprops={'arrowstyle':'<->','color':blue})
    ax.text((a[0]+b[0])/2+offset[0],(a[1]+b[1])/2+offset[1],label,color=blue,ha='center',va='center',fontsize=9,
            bbox={'facecolor':'white','edgecolor':'none','pad':1})

def finish(ax,title):
    ax.set_aspect('equal');ax.set_title(title,fontsize=14,pad=15)
    ax.axis('off')

h=D['deck_height']*1000
post=D['tower_width']*500
ringtop=h-D['deck_thickness']*1000
posttop=ringtop-20
ax=axes[0]
box(ax,-225,40,450,20)
for y in (-post,post):
    box(ax,y-10,80,20,posttop-80)
box(ax,-225,60,450,20)
box(ax,-140,posttop,280,20)
box(ax,-150,ringtop,300,2)
for y in (-150,148):box(ax,y,ringtop-20,2,20)
wheelwidth=D['assumed_wheel_width']*1000
for y in (-270,270-wheelwidth):box(ax,y,0,wheelwidth,70,'#333940')
for y in (-215,215):
    ax.add_patch(Circle((y,12),8,fill=False,linestyle='--',edgecolor=blue))
ax.plot([-post,post],[80,posttop],color=blue,alpha=.3,ls='--')
ax.plot([post,-post],[80,posttop],color=blue,alpha=.3,ls='--')
box(ax,-70,85,140,70,green)
box(ax,-100,160,200,35,'#7faa92')
ax.text(0,210,'하단 배터리·보드 배치안',ha='center',fontsize=10)
for y in (-75,75):
    box(ax,y-35,h,70,70,'#e8b654')
    ax.text(y,h+87,'SO-101',ha='center',fontsize=9)
box(ax,-10,h,20,230)
box(ax,-82.5,h+230,165,40,'#454c55')
dimension(ax,(-310,0),(-310,h),'720',(-22,0))
dimension(ax,(-150,h+125),(150,h+125),'상판 300',(0,15))
dimension(ax,(-270,-35),(270,-35),'바퀴 포함 540',(0,-15))
ax.text(0,350,'전면 개방\n점선 X는 후면 보강',ha='center',fontsize=10)
ax.set_xlim(-365,310);ax.set_ylim(-90,1050);finish(ax,'정면 · 바퀴 폭은 30 mm 가정')

ax=axes[1]
box(ax,-170,40,340,20)
for x in (-post,post):
    box(ax,x-10,60,20,20)
    box(ax,x-10,80,20,posttop-80)
box(ax,-140,posttop,280,20);box(ax,-150,ringtop,300,2)
for x in (-150,148):box(ax,x,ringtop-20,2,20)
ax.plot([-post,post],[80,posttop],color=blue,lw=2)
ax.plot([post,-post],[80,posttop],color=blue,lw=2)
ax.add_patch(Circle((110,35),35,color='#333940'))
ax.add_patch(Circle((-110,12),8,fill=False,edgecolor=blue))
box(ax,-110,85,140,70,green)
box(ax,-100,h,111,70,'#e8b654')
box(ax,-130,h,20,250)
box(ax,-140,h+230,40,40,'#454c55')
box(ax,162,183,55,34,'#454c55')
ax.text(190,245,'라이다\n후방 가림 검토',ha='center',fontsize=9)
dimension(ax,(-110,-30),(110,-30),'접지 전후 220',(0,-16))
dimension(ax,(245,80),(245,posttop),'기둥 618',(23,0))
ax.text(-170,-75,'뒤',ha='center');ax.text(170,-75,'앞',ha='center')
ax.text(0,470,'좌우 측면 X 보강',ha='center',fontsize=10,bbox={'facecolor':'white','edgecolor':'none'})
ax.set_xlim(-230,310);ax.set_ylim(-90,1050);finish(ax,'측면 · 캐스터 원은 위치 기호')

ax=axes[2]
for x in (-170,150):box(ax,-225,x,450,20)
for y in (-225,-70,50,205):box(ax,y,-150,20,300)
for x in (-post,post):box(ax,-225,x-10,450,20,'#8ba3b4',.65)
for y in (-270,270-wheelwidth):box(ax,y,75,wheelwidth,70,'#333940')
for y in (-215,215):ax.add_patch(Circle((y,-110),8,fill=False,edgecolor=blue,lw=2))
ax.add_patch(Rectangle((-150,-150),300,300,fill=False,edgecolor=green,ls='--',lw=2))
for x in (-post,post):
    for y in (-post,post):box(ax,y-10,x-10,20,20,blue)
dimension(ax,(-225,215),(225,215),'하단 프레임 450',(0,15))
dimension(ax,(310,-170),(310,170),'340',(20,0))
dimension(ax,(-50,-20),(50,-20),'100',(0,15))
ax.text(0,265,'앞',ha='center');ax.text(0,-210,'뒤',ha='center')
ax.text(0,-290,'초록 점선: 상판 300×300\n파랑 사각형: 기둥 중심 ±130\n파랑 원: 뒤 캐스터, 외측 레일 아래\n회색 가로재 2개 추가: 하중 분산',ha='center',fontsize=10)
ax.set_xlim(-320,365);ax.set_ylim(-400,310);finish(ax,'상면 · 상부 부품 생략')
fig.suptitle('450×340 하단 유지 / 300×300 상판 / 720 높이 — 좌표 기반 배치 검토',fontsize=20,y=.98)
fig.text(.5,.035,'개념안: 실제 캐스터 크기·서보 장착부·팔 자세·체결부 형상 미확정. 제조 도면이나 구조 안전 승인 자료가 아님.',ha='center',fontsize=11)
fig.tight_layout(rect=(0,.07,1,.94))
out=ROOT/'docs/assets/raised_frame_20260909'
out.mkdir(parents=True,exist_ok=True)
for ext in ('png','svg'):
    fig.savefig(out/f'coordinate_layout.{ext}',dpi=170)
print(out/'coordinate_layout.png')
