"""Eight physical parts; CAD mm, SI tests. Run the Slim `build` command.

gen_step(phase_deg) also rebuilds the complete named assembly at a design pose.
This pose function is CAD construction only; simulation receives phase 0 once.
All runtime output motion comes from the driven revolute/rod/slider chain.
"""
from math import pi, sin, cos, sqrt, radians, degrees, atan2
from pathlib import Path
from build123d import Box, Cylinder, Compound, Pos, Rot, Axis, fillet, export_step
from simulation_ready_cad_workflow.validation_pipeline.slim_harness.asset import Asset, Part

CRANK_X, AXIS_Z = -55.0, 52.0
RADIUS, ROD_LENGTH = 18.0, 80.0
RADIAL_CLEARANCE = 0.25  # source clearance; native collision policy is unchanged
KEY_WIDTH = 7.0  # diagonal 9.899 mm: fits through the 10.50 mm bearing bore
KEY_SOCKET_WIDTH = 7.15
WRIST_X = CRANK_X + RADIUS + ROD_LENGTH
SLIDER_Y = 38.4
DENSITY = 2700.0  # nominal aluminum alloy density; no strength claim
CRANK_DENSITY = 7800.0  # steel crank train, including its retained screws
RATE_HZ, MAX_TORQUE = 0.75, 0.35
DURATION = 6.0


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0+x1)/2, (y0+y1)/2, (z0+z1)/2) * Box(x1-x0, y1-y0, z1-z0)


def cy(x, z, radius, y0, y1):
    return Pos(x, (y0+y1)/2, z) * Rot(90, 0, 0) * Cylinder(radius, y1-y0)


def cz(x, y, radius, z0, z1):
    return Pos(x, y, (z0+z1)/2) * Cylinder(radius, z1-z0)


def capsule_y(x0, x1, z, radius, y0, y1):
    return (cy(x0, z, radius, y0, y1) + cy(x1, z, radius, y0, y1)
            + box(x0, x1, y0, y1, z-radius, z+radius))


def screw_y(x, z, head_y0, head_y1, shank_y0, shank_y1,
            head_radius=4.6, shank_radius=1.5, recess_side=1):
    # Smooth thread envelopes avoid claiming resolved thread friction.
    # The seated threaded attachment is rigid, documented in OPERATING.md.
    shape = cy(x, z, head_radius, head_y0, head_y1) + cy(x, z, shank_radius, shank_y0, shank_y1)
    if recess_side > 0:
        shape -= box(x-2.5, x+2.5, head_y1-1.0, head_y1+0.1, z-0.65, z+0.65)
    else:
        shape -= box(x-2.5, x+2.5, head_y0-0.1, head_y0+1.0, z-0.65, z+0.65)
    return shape


def shapes_at(phase_deg=0.0):
    base = box(-125, 125, -55, 55, 0, 6)
    base = fillet(base.edges().filter_by(Axis.Z), 4)
    # Integral pedestal, shaft-bearing boss and root reinforcement.
    base += box(CRANK_X-13, CRANK_X+13, -10, 10, 6, AXIS_Z)
    base += cy(CRANK_X, AXIS_Z, 13, -10, 10)
    base += box(CRANK_X-20, CRANK_X+20, -15, 15, 6, 14)
    base -= cy(CRANK_X, AXIS_Z, 5+RADIAL_CLEARANCE, -16, 16)
    base -= cy(CRANK_X, 26, 7, -16, 16)
    # Rectangular guide is closed on all four transverse sides.
    base += box(58, 88, SLIDER_Y-10, SLIDER_Y+10, 6, 62)
    base -= box(57, 89, SLIDER_Y-5.25, SLIDER_Y+5.25, AXIS_Z-5.25, AXIS_Z+5.25)
    base -= box(64, 82, SLIDER_Y-11, SLIDER_Y+11, 15, 33)
    # Four mounting slots are source details; the mounting boundary is external.
    for x in (-108, 108):
        for y in (-40, 40):
            slot = cz(x, y-3, 3.2, -1, 7) + cz(x, y+3, 3.2, -1, 7)
            slot += box(x-3.2, x+3.2, y-3, y+3, -1, 7)
            base -= slot

    pin_x = CRANK_X + RADIUS
    crank = capsule_y(CRANK_X, pin_x, AXIS_Z, 7, 10.4, 16.4)
    crank += cy(CRANK_X, AXIS_Z, 9, 10.4, 16.4)
    crank += cy(CRANK_X, AXIS_Z, 5, -10.4, 10.4)
    crank += box(CRANK_X-KEY_WIDTH/2, CRANK_X+KEY_WIDTH/2, -20.4, -10.4,
                 AXIS_Z-KEY_WIDTH/2, AXIS_Z+KEY_WIDTH/2)
    crank -= cy(CRANK_X, AXIS_Z, 1.55, -20.5, -13.4)
    crank += cy(pin_x, AXIS_Z, 5, 16.4, 20.6)
    crank += cy(pin_x, AXIS_Z, 3, 20.6, 27.4)
    crank -= cy(pin_x, AXIS_Z, 1.55, 22.4, 27.5)

    grip_x = CRANK_X - 32
    hand = capsule_y(grip_x, CRANK_X, AXIS_Z, 6, -20.4, -16.4)
    hand += cy(CRANK_X, AXIS_Z, 9, -20.4, -10.4)
    hand -= box(CRANK_X-KEY_SOCKET_WIDTH/2, CRANK_X+KEY_SOCKET_WIDTH/2, -21, -10,
                AXIS_Z-KEY_SOCKET_WIDTH/2, AXIS_Z+KEY_SOCKET_WIDTH/2)
    hand += cy(grip_x, AXIS_Z, 4.0, -55, -20.4)
    hand += cy(grip_x, AXIS_Z, 6.5, -52, -28)
    hand += cy(grip_x, AXIS_Z, 7, -55, -52)
    hand += cy(grip_x, AXIS_Z, 7, -28, -25)

    spindle_screw = screw_y(CRANK_X, AXIS_Z, -23.4, -20.4, -20.4, -14.4,
                           head_radius=6.0, recess_side=-1)
    crank_screw = screw_y(pin_x, AXIS_Z, 27.4, 30.4, 23.4, 27.4)

    # Two circular eyes and a relieved straight web, all one physical rod.
    rod = cy(pin_x, AXIS_Z, 6, 21, 27) + cy(WRIST_X, AXIS_Z, 6, 21, 27)
    rod += box(pin_x, WRIST_X, 21, 27, AXIS_Z-3.5, AXIS_Z+3.5)
    rod -= cy(pin_x, AXIS_Z, 3+RADIAL_CLEARANCE, 20, 28)
    rod -= cy(WRIST_X, AXIS_Z, 3+RADIAL_CLEARANCE, 20, 28)
    # Shallow web pockets preserve a central 3.6 mm thickness.
    rod -= capsule_y(pin_x+13, WRIST_X-13, AXIS_Z, 1.7, 20.9, 22.2)
    rod -= capsule_y(pin_x+13, WRIST_X-13, AXIS_Z, 1.7, 25.8, 27.1)

    slider = box(WRIST_X-10, WRIST_X+10, SLIDER_Y-7, SLIDER_Y+7, AXIS_Z-8, AXIS_Z+8)
    slider += box(WRIST_X+10, WRIST_X+95, SLIDER_Y-5, SLIDER_Y+5, AXIS_Z-5, AXIS_Z+5)
    slider += cy(WRIST_X, AXIS_Z, 5, 27.4, 31.4)
    slider += cy(WRIST_X, AXIS_Z, 3, 20.6, 27.4)
    slider -= cy(WRIST_X, AXIS_Z, 1.55, 20.5, 25.6)
    wrist_screw = screw_y(WRIST_X, AXIS_Z, 17.6, 20.6, 20.6, 24.6, recess_side=-1)

    shapes = dict(base_frame=base, drive_crank=crank, hand_crank=hand,
                  spindle_retaining_screw=spindle_screw,
                  crankpin_retaining_screw=crank_screw, connecting_rod=rod,
                  slider_pushrod=slider, wrist_retaining_screw=wrist_screw)
    if phase_deg:
        theta = radians(phase_deg)
        moving_pin = (CRANK_X+RADIUS*cos(theta), AXIS_Z-RADIUS*sin(theta))
        wrist_x = moving_pin[0] + sqrt(ROD_LENGTH**2-(moving_pin[1]-AXIS_Z)**2)
        rod_angle = -degrees(atan2(AXIS_Z-moving_pin[1], wrist_x-moving_pin[0]))
        crank_pose = Pos(CRANK_X,0,AXIS_Z)*Rot(0,phase_deg,0)*Pos(-CRANK_X,0,-AXIS_Z)
        for name in ('drive_crank','hand_crank','spindle_retaining_screw','crankpin_retaining_screw'):
            shapes[name] = crank_pose * shapes[name]
        shapes['connecting_rod'] = (Pos(moving_pin[0],0,moving_pin[1]) * Rot(0,rod_angle,0)
                                    * Pos(-pin_x,0,-AXIS_Z) * rod)
        for name in ('slider_pushrod','wrist_retaining_screw'):
            shapes[name] = Pos(wrist_x-WRIST_X,0,0)*shapes[name]
    for name, shape in shapes.items():
        shape.label = name
        assert len(shape.solids()) == 1, (name, len(shape.solids()))
        assert shape.is_valid and shape.volume > 0, name
    return shapes


def region(p0, p1, radius):
    return {'points': [p0,p1], 'radius': radius}


def rotary(jid, a, b, x, z, y0, y1, shaft_body, bore_body, radius):
    r = region([x,y0,z], [x,y1,z], radius+0.3)
    center = [x,(y0+y1)/2,z]
    return dict(id=jid, type='revolute', body_a=a, body_b=b,
                anchor=center, axis=[0,1,0], region_a=r, region_b=r,
                support={'shaft': dict(body=shaft_body, center=center, axis=[0,1,0], radius=radius, length=y1-y0),
                         'bore': dict(body=bore_body, center=center, axis=[0,1,0], radius=radius+RADIAL_CLEARANCE, length=y1-y0),
                         'max_clearance': 0.30})


def joint_definitions():
    # Native controller applies +torque to body_a and measures omega_a-omega_b.
    return [rotary('crank_bearing','crank','frame',CRANK_X,AXIS_Z,-9,9,'crank','frame',5),
            rotary('crankpin','crank','rod',CRANK_X+RADIUS,AXIS_Z,21.2,26.8,'crank','rod',3),
            rotary('wrist','slider','rod',WRIST_X,AXIS_Z,21.2,26.8,'slider','rod',3),
            dict(id='output_guide',type='prismatic',body_a='frame',body_b='slider',
                 anchor=[73,SLIDER_Y,AXIS_Z],axis=[1,0,0],
                 region_a=region([58,SLIDER_Y,AXIS_Z],[88,SLIDER_Y,AXIS_Z],7.5),
                 region_b=region([WRIST_X+10,SLIDER_Y,AXIS_Z],[WRIST_X+95,SLIDER_Y,AXIS_Z],7.1))]


def test_definitions():
    tests=[]
    for loaded in (False, True):
        tid = 'loaded_reciprocation' if loaded else 'unloaded_reciprocation'
        def assertion(name, criteria, purpose, expression):
            return dict(id=name,criterion_ids=criteria,purpose=purpose,expression=expression)
        assertions = [
            assertion('complete_cycles',['F1'],'At least three full consecutive-extrema cycles; both legs >=20 mm and return <=1 mm, including startup.',
                      'cycles(signal("slider.position.x_m"), 0.020, 0.001) >= 3'),
            assertion('crank_rate_lower',['F1','F3'],'Supplementary input-rate observation; exact output-extrema cycle rates are reported by observe_trace.py.',
                      'mean(after(signal("crank.angular_speed.y_rad_s"), 0.5))/(2*3.141592653589793) >= 0.5'),
            assertion('crank_rate_upper',['F1','F3'],'Input-rate upper bound, with output rate independently measured from recorded extrema.',
                      'mean(after(signal("crank.angular_speed.y_rad_s"), 0.5))/(2*3.141592653589793) <= 1.0'),
            assertion('finite_crank_effort',['F3','F4'],'Only the intended crank bearing is driven; effort never exceeds the declared limit.',
                      'max(abs(signal("input.crank_bearing.effort"))) <= 0.350000001'),
        ]
        for joint in ('crank_bearing','crankpin','wrist','output_guide'):
            assertions.append(assertion(joint+'_engaged',['F4','F5'],'Finite physical connection remains engaged throughout the uninterrupted run.',
                                        f'min(signal("{joint}.active")) >= 1'))
        if loaded:
            assertions.append(assertion('load_work_each_cycle',['F1','F2'],
                'With constant -5 N, every accepted >=20 mm +X leg performs >=0.10 J; observe_trace.py lists individual push and return works.',
                'cycles(5*signal("slider.position.x_m"), 0.10, 0.005) >= 3'))
        tests.append(dict(id=tid,purpose=('Loaded' if loaded else 'Unloaded')+
            ' uninterrupted reciprocation from rest. Evaluate consecutive full output cycles, each travel leg, return closure, actual extremum timestamps/rate and crank effort. '+
            ('Constant -5 N throughout opposes +X pushing and assists -X return; report load work separately per stroke.' if loaded else 'No deliberately applied output load.'),
            duration_s=DURATION,step_s=0.001,
            inputs=[dict(joint='crank_bearing',mode='speed',value=RATE_HZ*2*pi,max_effort=MAX_TORQUE,gain=1.0,start_s=0,end_s=DURATION)],
            loads=[dict(id='working_resistance',body='slider',force_n=[-5,0,0],start_s=0,end_s=DURATION)] if loaded else [],
            assertions=assertions))
    return tests


def build():
    shapes = shapes_at()
    # Include the complete assembly in the normal preserved scene delivery.
    export_step(Compound(label='hand_cranked_slider',children=list(shapes.values())),
                Path(__file__).with_name('assembly.step'))
    membership = {name: ('frame' if name=='base_frame' else 'rod' if name=='connecting_rod'
                       else 'slider' if name in ('slider_pushrod','wrist_retaining_screw') else 'crank') for name in shapes}
    colors = dict(frame=[0.28,0.36,0.43],crank=[0.85,0.38,0.12],rod=[0.82,0.76,0.40],slider=[0.20,0.58,0.72])
    return Asset(parts={name: Part(shape,body=membership[name],role='fixed' if name=='base_frame' else 'dynamic',
                                   density_kg_m3=CRANK_DENSITY if membership[name]=='crank' else DENSITY,
                                   color=colors[membership[name]]) for name,shape in shapes.items()},
                 joints=joint_definitions(),tests=test_definitions(),gravity_m_s2=[0,0,-9.81],
                 description='Eight-part hand-cranked 36 mm reciprocating pushrod; 80 mm connecting rod; two operating conditions.',
                 dependencies=['OPERATING.md','observe_trace.py','assembly.step',
                               'REVISION_NOTES.md','DEVELOPMENT_EVIDENCE.md',
                               'evidence/loaded_v2_prefix_audit.json','evidence/unloaded_v2_prefix_audit.json',
                               'evidence/loaded_v2_prefix_observations.json','evidence/unloaded_v2_prefix_observations.json',
                               'evidence/author_budget_snapshot.json','R02_REPAIR.md','insertion_check.py',
                               'evidence/r02_insertion_observations.json',
                               'R04_CONTINUATION.md',
                               'evidence/r04_preserved_loaded_observations.json',
                               'evidence/r04_preserved_loaded_prefix_audit.json',
                               'evidence/r04_preserved_unloaded_observations.json',
                               'evidence/r04_preserved_unloaded_prefix_audit.json',
                               'evidence/r04_loaded_transport.json',
                               'evidence/r04_unloaded_transport.json',
                               'evidence/r04_author_budget_snapshot.json'])


def gen_step(phase_deg=0):
    return Compound(label='hand_cranked_slider',children=list(shapes_at(phase_deg).values()))


if __name__ == '__main__':
    export_step(gen_step(), Path(__file__).with_name('assembly.step'))
