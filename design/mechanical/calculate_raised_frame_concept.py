#!/usr/bin/env python3
"""상승 프레임 개념 비교. 강체 전도·보 부재 계산이며 FEA/실물 승인 아님."""
from pathlib import Path
import json
import math
import yaml


def point_margin(point, polygon):
    """CCW 지지다각형 각 변까지 부호 있는 최단 수직거리.

    >>> point_margin((0, 0), [(-1,-1),(1,-1),(1,1),(-1,1)])
    1.0
    >>> point_margin((2, 0), [(-1,-1),(1,-1),(1,1),(-1,1)])
    -1.0
    """
    return min(((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])) /
               math.hypot(b[0]-a[0], b[1]-a[1])
               for a, b in zip(polygon, polygon[1:]+polygon[:1]))


def main():
    spec = yaml.safe_load(Path(__file__).with_name('raised_frame_concept_20260909.yaml').read_text())
    d, m, s = (spec[k] for k in ('geometry', 'materials', 'screening'))
    L = d['deck_height']-d['deck_thickness']-d['upper_ring_height']-d['lower_frame_top']-d['lower_crossmember_height']
    E, g = m['E_aluminum'], s['g']
    members = {}
    for name, n, I in [('two_2020',2,m['I_2020']), ('four_2020',4,m['I_2020']),
                       ('two_2040_weak',2,m['I_2040_weak']), ('two_2040_strong',2,m['I_2040_strong']),
                       ('four_2040_weak',4,m['I_2040_weak']), ('four_2040_strong',4,m['I_2040_strong'])]:
        F, M = s['lateral_force'], s['upper_applied_moment']
        k = 3*n*E*I/L**3
        members[name] = {'tip_deflection_mm':1000*(F/k+M*L**2/(2*n*E*I)),
                         'root_bending_stress_MPa':(F*L+M)/n*(0.020 if 'strong' in name else 0.010)/I/1e6,
                         'force_only_frequency_Hz':math.sqrt(k/s['effective_top_mass'])/(2*math.pi)}
    width = d['tower_width']
    diag = math.hypot(width,L)
    A = m['strap_width']*m['strap_thickness']
    # 한쪽으로 밀면 각 X의 인장 대각재 하나만 유효. 좌우 두 측면을 합산.
    kbrace = 2*m['E_steel']*A*width**2/diag**3
    equivalent_force = s['lateral_force']+s['upper_applied_moment']/L
    brace = {'ideal_side_pair_k_N_per_m':kbrace, 'active_diagonal_tension_N':equivalent_force*diag/(2*width),
             'joint_sensitivity_deflection_mm':{str(f):1000*equivalent_force/(kbrace*f) for f in s['ideal_to_joint_stiffness_fractions']}}
    rho = m['profile_2020_kg_per_m']
    components = list(spec['mass_assumptions'].items())
    components += [('base_profiles',dict(mass=2.1*rho,x=0,z=0.05)),
                   ('lower_crossmembers',dict(mass=0.9*rho,x=0,z=0.07)),
                   ('posts',dict(mass=4*L*rho,x=0,z=0.08+L/2)),
                   ('upper_ring_and_arm_beams',dict(mass=d['upper_profile_total_length']*rho,x=0,z=d['deck_height']-d['deck_thickness']-0.01)),
                   ('deck',dict(mass=0.3**2*d['reference_flat_deck_thickness']*m['density_aluminum'],x=0,z=d['deck_height']-d['deck_thickness']/2)),
                   ('six_tension_straps',dict(mass=6*diag*A*m['density_steel'],x=0,z=0.08+L/2))]
    yf = d['wheel_outer_half_width']-d['assumed_wheel_width']/2
    yr = d['rear_contact_half_width']
    xf,xr = d['support_front_x'],d['support_rear_x']
    polygon = [(xr,-yr),(xf,-yf),(xf,yf),(xr,yr)]
    cases = {}
    opt=spec['weight_optimization']
    optimized_deck_mass=0.3**2*opt['folded_deck_skin_thickness']*m['density_aluminum']+opt['folded_deck_flange_mass_allowance']
    optimized_brace_mass=6*diag*A*m['density_aluminum']
    for placement in ('low_battery_boards','top_battery_boards','optimized_low','optimized_top'):
        for pose_name, pose in spec['poses'].items():
            parts=[]
            for name, original in components:
                v=original.copy()
                if name in ('left_arm','right_arm'):
                    v.update(x=pose['arm_com_x'], z=pose['arm_com_z'])
                    v['y']=0.075 if name=='left_arm' else -0.075
                if placement in ('top_battery_boards','optimized_top') and name in ('battery','boards_pi'):
                    v['z']=0.76
                if placement.startswith('optimized') and name=='deck':
                    v['mass']=optimized_deck_mass
                if placement.startswith('optimized') and name=='six_tension_straps':
                    v['mass']=optimized_brace_mass
                parts.append(v)
            for side in ('left','right'):
                parts.append(dict(mass=pose[f'{side}_payload'],x=pose['payload_x'],
                                  y=pose.get('payload_y_half',0.15)*(1 if side=='left' else -1),
                                  z=pose.get('payload_z',0.80)))
            mass=sum(v['mass'] for v in parts)
            x=sum(v['mass']*v['x'] for v in parts)/mass
            z=sum(v['mass']*v['z'] for v in parts)/mass
            y=sum(v['mass']*v.get('y',0) for v in parts)/mass
            margin=xf-x
            dynamic={}
            for slope in s['slope_degrees']:
                for accel in s['acceleration_cases']:
                    effective_x=x+z*(math.tan(math.radians(slope))+accel/(g*math.cos(math.radians(slope))))
                    dynamic[f'{slope}deg_{accel}mps2']=1000*point_margin((effective_x,y),polygon)
            front_load=mass*g*(x-xr)/(xf-xr)
            three_point={str(missing):1000*point_margin((x,y),polygon[:missing]+polygon[missing+1:]) for missing in range(4)}
            cases[f'{placement}/{pose_name}']={'mass_kg':mass,'com_x_mm':x*1000,'com_y_mm':y*1000,'com_z_mm':z*1000,
                'static_front_margin_mm':margin*1000,'flat_tip_acceleration_mps2':g*margin/z,
                'flat_lateral_0p5mps2_worst_margin_mm':1000*min(point_margin((x,y+sign*z*0.5/g),polygon) for sign in (-1,1)),
                'front_wheel_pair_load_N':front_load,'dynamic_margin_mm':dynamic,
                'three_contact_static_margin_mm_missing_vertex':three_point,
                'three_contact_all_meet_30mm_screen':min(three_point.values())>=1000*s['screening_margin_target'],
                'remaining_external_moment_at_30mm_margin_Nm':mass*g*(margin-s['screening_margin_target'])}
    parked=cases['optimized_low/parked']
    drive={}
    mass=parked['mass_kg']
    for slope in s['slope_degrees']:
        for crr in s['rolling_coefficient_cases']:
            theta=math.radians(slope)
            force=mass*(s['acceleration_cases'][0]+g*(math.sin(theta)+crr*math.cos(theta)))
            drive[f'{slope}deg_crr{crr}']={'total_force_N':force,'torque_per_wheel_Nm':force*d['wheel_radius']/2}
    steps={}
    R=d['wheel_radius']
    # 고립된 바퀴가 날카로운 수직 턱 모서리에 걸렸을 때 축 수평추력의 이상 정역학.
    for h in s['step_height_cases']:
        ratio=math.sqrt(2*R*h-h*h)/(R-h)
        steps[str(h)]={'axle_push_to_wheel_load_ratio':ratio,
                       'axle_push_per_front_wheel_N':ratio*parked['front_wheel_pair_load_N']/2,
                       'ideal_pair_push_exceeds_assumed_flat_traction':ratio>s['tire_friction_assumption']}
    arm_payload_torque={f'{mass}kg_{reach}m':mass*g*reach for mass in (0.12,0.30,0.56,1.10) for reach in (0.20,0.25,0.30)}
    report={'scope':'가정 기반 개념 선별, 접합/판/열/충격/동역학 미검증',
            'post_length_mm':1000*L, 'beam_only_comparison':members,'bracing':brace,
            'support_polygon_m_assumed_wheel_width':polygon,'cases':cases,'drive':drive,
            'candidate_no_load_speed_mps':s['no_load_rpm_candidate']*2*math.pi*R/60,
            'traction_cap_flat_N':s['tire_friction_assumption']*parked['front_wheel_pair_load_N'],
            'ideal_rated_pair_force_N':2*s['rated_torque_candidate']/R,
            'sharp_step_geometry_only':steps,
            'arm_payload_only_torque_Nm_excludes_arm_weight':arm_payload_torque,
            'optimized_deck_mass_kg':optimized_deck_mass,
            'optimized_brace_mass_kg':optimized_brace_mass,
            'optimized_aluminum_brace_joint_sensitivity_mm':{str(f):1000*equivalent_force/(kbrace*E/m['E_steel']*f) for f in s['ideal_to_joint_stiffness_fractions']},
            'component_masses_kg':{name:v['mass'] for name,v in components}}
    payload=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
    output=Path(__file__).resolve().parents[2]/'docs/assets/raised_frame_20260909/calculation.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(payload)
    print(output)


if __name__=='__main__':
    main()
