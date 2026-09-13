"""Rebuildable seven-part gravity-return pedal bin. CAD coordinates: mm.

Build with the Slim build command. gen_step() supplies the complete machine
assembly (the separately declared level support is an environmental item).
"""
from pathlib import Path
from math import sqrt, atan2, degrees
from build123d import Box, Cylinder, Pos, Rot, Compound, Axis, export_step, fillet
from simulation_ready_cad_workflow.validation_pipeline.slim_harness.asset import Asset, Part

ALUMINUM_DENSITY = 2700.0
HINGE = (-101.0, 0.0, 263.0)
UPPER = (-126.0, 117.0, 263.0)
LOWER = (125.0, 117.0, 75.0)
FOOT_POINT = (135.0, 75.0, 66.0)
PRESS_ON = 0.20
RELEASE = 1.60
FORCE_N = 24.0


def block(x0, x1, y0, y1, z0, z1):
    return Pos((x0+x1)/2, (y0+y1)/2, (z0+z1)/2) * Box(x1-x0, y1-y0, z1-z0)


def ycylinder(x, z, radius, y0, y1):
    return Pos(x, (y0+y1)/2, z) * Rot(90, 0, 0) * Cylinder(radius, y1-y0)


def round_box(x0, x1, y0, y1, z0, z1, radius):
    shape = block(x0,x1,y0,y1,z0,z1)
    return fillet(shape.edges().filter_by(Axis.Z), radius)


def rivet(x, z, y0, y1):
    # One cold-headed rivet, second head formed after insertion; not two caps.
    return (ycylinder(x,z,3.0,y0,y1)
            + ycylinder(x,z,5.0,y0-2,y0)
            + ycylinder(x,z,5.0,y1,y1+2))


def geometry():
    base = round_box(-130,175,-100,135,4,8,12)
    # Integral rectangular feet have real 4 mm square contact faces.
    for x,y in [(-115,-85),(155,-85),(-115,120),(155,120)]:
        base += block(x-2,x+2,y-2,y+2,0,5)
    outer = round_box(-90,90,-80,80,7,259,12)
    cavity = round_box(-86,86,-76,76,10,262,8)
    frame = base + (outer-cavity)
    # Three cast contact pips support the closed lid above the rim.
    for x,y in [(-65,-78),(-65,78),(88,0)]:
        frame += block(x-1,x+1,y-1,y+1,258.5,260)
    # Two integral rear hinge lugs, with exact carrier bores for the rivet.
    for ya,yb in [(-50,-36),(36,50)]:
        # The 5 mm lug radius leaves 1 mm around the lid's swept rear edge.
        lug = block(-108,-85,ya,yb,246,258.5) + ycylinder(-101,263,5,ya,yb)
        frame += lug-ycylinder(-101,263,3,ya-1,yb+1)
    # Side/front guide: rectangular stem has 0.2 mm running clearance per face.
    sleeve = block(114,136,89,111,90,106)-block(119.8,130.2,94.8,105.2,89,107)
    bracket = block(84,120,78,94,90,106)
    frame += sleeve + bracket
    # Lower pedal stop is integral with the base; 42 mm nominal downstroke.
    frame += round_box(108,161,48,85,7,17,4)
    for x,y in [(115,55),(153,55),(134,80)]:
        frame += block(x-1,x+1,y-1,y+1,16.5,18)
    # Lid plate, integral main barrel, connecting tongue and side rear crank.
    lid = round_box(-95,95,-85,85,260,266,12)
    lid += block(-101,-88,-32,32,260,266)
    lid += ycylinder(-101,263,6,-32,32)
    lid -= ycylinder(-101,263,3.30,-33,33)
    lid += block(-101,-87,75,113,260,266)
    lid += block(-126,-94,105,113,260,266)
    lid += ycylinder(-126,263,7,105,113)
    lid -= ycylinder(-126,263,3,104,114)
    # Integral gravity arm: opening raises this 0.32 kg side lobe.
    # Its potential-energy rise reduces the kinetic energy reaching the stop.
    # Full rigid source geometry and mass are retained; there is no spring,
    # damper, output controller, or additional purchased/assembled part.
    gravity_lobe_height = 0.32/(ALUMINUM_DENSITY*1e-9*50*40)
    lid += block(-104,-94,108,156,260,266)
    lid += block(-104,-98,147,153,90,263)
    lid += block(-126,-76,140,180,63-gravity_lobe_height/2,63+gravity_lobe_height/2)
    # Vertical foot carriage. The continuous stem reaches through the sleeve
    # at both stops; the lower rod eye stays below the sleeve throughout.
    pedal = round_box(106,164,48,110,60,66,7)
    pedal += block(120,130,95,105,62,150)
    pedal += ycylinder(125,75,7,103,113)
    pedal -= ycylinder(125,75,3,94,114)
    # Five integral shallow tread ribs make the foot surface recognizable.
    for xc in [114,124,134,144,154]:
        pedal += block(xc-1,xc+1,53,87,65.5,67)
    # Oblique two-eye pull rod, outside the waste cavity on the left.
    dx,dz=LOWER[0]-UPPER[0],LOWER[2]-UPPER[2]
    length=sqrt(dx*dx+dz*dz)
    angle=degrees(atan2(dx,dz))
    rod=(Pos((UPPER[0]+LOWER[0])/2,117,(UPPER[2]+LOWER[2])/2)
         * Rot(0,angle,0) * Box(8,6,length))
    for x,_,z in [UPPER,LOWER]:
        rod += ycylinder(x,z,7,114,120)
        rod -= ycylinder(x,z,3.30,113,121)
    shapes = {
        'bin_shell':frame,
        'hinge_rivet':rivet(-101,263,-50,50),
        'hinged_lid':lid,
        'crank_rivet':rivet(-126,263,105,120.3),
        'sliding_pedal':pedal,
        'pedal_rivet':rivet(125,75,95,120.3),
        'side_pull_rod':rod,
        'level_support':block(-220,220,-200,200,-8,0),
    }
    for name,shape in shapes.items():
        shape.label=name
        if len(shape.solids()) != 1 or not shape.is_valid:
            raise ValueError(f'{name}: expected one connected valid manufactured solid; got {len(shape.solids())}')
    return shapes


def region(x,y,z, y0=None,y1=None,r=5):
    return {'points':[[x,y if y0 is None else y0,z],[x,y if y1 is None else y1,z]],'radius':r}


def joints():
    return [
        {'id':'lid_hinge','type':'revolute','body_a':'lid','body_b':'bin_frame',
         'anchor':list(HINGE),'axis':[0,1,0],
         'region_a':region(-101,0,263,-30,30),
         'region_b':region(-101,0,263,-50,50)},
        {'id':'pedal_guide','type':'prismatic','body_a':'pedal','body_b':'bin_frame',
         'anchor':[125,100,100],'axis':[0,0,-1],
         'region_a':{'points':[[125,100,63],[125,100,150]],'radius':7},
         'region_b':{'points':[[125,100,90],[125,100,106]],'radius':9}},
        {'id':'rod_upper','type':'revolute','body_a':'pull_rod','body_b':'lid',
         'anchor':list(UPPER),'axis':[0,1,0],
         'region_a':region(*UPPER,114,120),
         'region_b':region(*UPPER,105,120)},
        {'id':'rod_lower','type':'revolute','body_a':'pull_rod','body_b':'pedal',
         'anchor':list(LOWER),'axis':[0,1,0],
         'region_a':region(*LOWER,114,120),
         'region_b':region(*LOWER,95,120)},
    ]


def signal(name):
    return f'signal("{name}")'


def norm_sq(body,quantity,unit):
    return '('+'+'.join(f'{signal(f"{body}.{quantity}.{axis}_{unit}")}**2' for axis in 'xyz')+')'


def displacement(body):
    return '('+'+'.join(f'({signal(f"{body}.position.{axis}_m")}-first({signal(f"{body}.position.{axis}_m")}))**2' for axis in 'xyz')+')**0.5'


def tests():
    assertions=[]
    def add(id,criteria,purpose,expression):
        assertions.append({'id':id,'criterion_ids':criteria,'purpose':purpose,'expression':expression})
    # Original-source projection proof and its conservative pose limits are
    # documented in OPERATING.md. 1.10 rad opens a vertical D80 passage.
    add('insertion_passage',['pedal_open_and_hold'],
        'Source-bound D80 vertical passage at (40,0), present by 2.0 s after onset and held through release.',
        'min(window(-signal("lid_hinge.coordinate"),1.00,1.60)) >= 1.10')
    add('open_com_settled',['pedal_open_and_hold'],'Lid COM speed magnitude during final 0.2 s before release.',
        f'max(window({norm_sq("lid","linear_speed","m_s")},1.40,1.60)) <= 0.01**2')
    add('open_angle_settled',['pedal_open_and_hold'],'Lid angular speed magnitude during final 0.2 s before release.',
        f'max(window({norm_sq("lid","angular_speed","rad_s")},1.40,1.60)) <= 0.0872664626**2')
    add('pedal_travel',['pedal_effort_travel'],
        'Conservative displacement bound for the actual foot operating point: COM translation plus 0.15 m times quaternion rotation error.',
        f'max({displacement("pedal")}+0.15*signal("pedal.orientation_error_rad")) <= 0.060')
    add('closed_com',['gravity_closing'],'Return within 3 s of release and stay within COM tolerance for at least 0.5 s.',
        f'max(window({displacement("lid")},2.80,3.40)) <= 0.002')
    add('closed_orientation',['gravity_closing'],'Quaternion orientation error relative to initial closed pose during 0.6 s return interval.',
        'max(window(signal("lid.orientation_error_rad"),2.80,3.40)) <= 0.0523598776')
    add('foot_off',['gravity_closing','mechanism_causality'],'No additional external foot work after the half-open load interval ends.',
        'span(after(signal("load.foot_press.work_j"),1.602)) <= 0.000000001')
    for j in ['lid_hinge','pedal_guide','rod_upper','rod_lower']:
        add(f'{j}_undriven',['mechanism_causality'],f'{j} receives no independently commanded joint input.',
            f'max(abs(signal("input.{j}.effort"))) <= 0.000000001')
        add(f'{j}_connected',['physical_model'],f'{j} retains finite physical connection regions throughout.',
            f'min(signal("{j}.active")) >= 1')
    add('support_translation',['nominal_conditions'],
        'Unclamped empty frame remains near initial position; also bounds the source-bound passage proof.',
        f'max({displacement("bin_frame")}) <= 0.0005')
    add('support_rotation',['nominal_conditions'],
        'Free frame remains upright on level support; no mounting restraint.',
        'max(signal("bin_frame.orientation_error_rad")) <= 0.002')
    return [{'id':'press_hold_release','purpose':
        'One uninterrupted empty upright gravity run: downward foot press opens a D80 passage, settles on the built stop, then gravity alone returns the lid.',
        'duration_s':3.4,'step_s':0.001,'inputs':[],
        'loads':[{'id':'foot_press','body':'pedal','force_n':[0,0,-FORCE_N],
                  'point':list(FOOT_POINT),'start_s':PRESS_ON,'end_s':RELEASE}],
        'assertions':assertions}]


def build():
    shapes=geometry()
    from export_assembly import write_files
    write_files(shapes,Path(__file__).resolve().parent,ALUMINUM_DENSITY,FOOT_POINT)
    membership={'bin_shell':'bin_frame','hinge_rivet':'bin_frame',
        'hinged_lid':'lid','crank_rivet':'lid','sliding_pedal':'pedal',
        'pedal_rivet':'pedal','side_pull_rod':'pull_rod','level_support':'level_support'}
    colors={'bin_frame':[0.20,0.43,0.49],'lid':[0.78,0.82,0.83],
        'pedal':[0.24,0.25,0.27],'pull_rod':[0.82,0.65,0.30],
        'level_support':[0.70,0.70,0.68]}
    return Asset(parts={name:Part(shape,body=membership[name],
        role='fixed' if name=='level_support' else 'dynamic',
        density_kg_m3=ALUMINUM_DENSITY,color=colors[membership[name]])
        for name,shape in shapes.items()},joints=joints(),tests=tests(),
        gravity_m_s2=[0,0,-9.81],
        description='Seven physical bin parts, three of them headed rivets, plus one level external support. No bin-to-world restraint. Downward foot load only; gravity return.',
        dependencies=['OPERATING.md','export_assembly.py','pedal_bin.step','source_geometry_facts.json'])


def gen_step():
    return Compound(label='pedal_bin',children=[shape for name,shape in geometry().items() if name!='level_support'])
