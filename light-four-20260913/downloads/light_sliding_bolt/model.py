"""Rebuildable five-part small door with a contact-operated sliding bolt.

CAD units: millimetres. Dynamics signals: SI. Run through Slim `build`.
`gen_step()` also returns the complete named, assembled STEP shape.
"""
from pathlib import Path
import copy
from build123d import Box, Cylinder, Pos, Rot, Compound, export_step
from simulation_ready_cad_workflow.validation_pipeline.slim_harness.asset import Asset, Part

HINGE_X, HINGE_Y = 14.5, 91.0
BOLT_X, BOLT_Z = 13.0, 130.0
BOLT_STROKE = 20.0


def block(x0, x1, y0, y1, z0, z1):
    return Pos((x0+x1)/2, (y0+y1)/2, (z0+z1)/2) * Box(x1-x0,y1-y0,z1-z0)


def cyl_z(x,y,z0,z1,r):
    return Pos(x,y,(z0+z1)/2)*Cylinder(r,z1-z0)


def cyl_x(x0,x1,y,z,r):
    return Pos((x0+x1)/2,y,z)*Rot(0,90,0)*Cylinder(r,x1-x0)


def make_parts(door_angle_deg=0.0, bolt_stroke_mm=0.0):
    # One machined/cast frame with rectangular clear opening, counterbored
    # external mounting holes, integral keeper, and end knuckle supports.
    frame = block(-8,8,-108,108,0,260) - block(-9,9,-90,90,10,250)
    for z0,z1 in ((20,40),(220,240)):
        frame += block(-8,HINGE_X,91,104,z0,z1)
        frame += cyl_z(HINGE_X,HINGE_Y,z0,z1,7)
        frame -= cyl_z(HINGE_X,HINGE_Y,z0-1,z1+1,3.0)
    keeper = block(4,23,-106,-94,121,139)
    keeper -= block(8.7,17.3,-107,-93,125.7,134.3)
    frame += keeper
    for y in (-100,100):
        for z in (55,130,205):
            frame -= cyl_x(-9,9,y,z,2.75)
            frame -= cyl_x(4.5,9,y,z,4.6)

    # Retained hinge rivet: lower manufactured head, upper head swaged after
    # insertion. Both heads are part of the same physical rivet.
    rivet = cyl_z(HINGE_X,HINGE_Y,20,240,3)
    rivet += cyl_z(HINGE_X,HINGE_Y,17.5,20,5)
    rivet += cyl_z(HINGE_X,HINGE_Y,240,242.5,5)

    # Door is a single aluminium part. Shallow recessed panels leave a 5 mm web.
    door = block(-4,4,-87,87,13,247)
    for z0,z1 in ((31,103),(159,229)):
        door -= block(2,5,-62,62,z0,z1)
        door -= block(-5,-3,-62,62,z0,z1)
    # Two finite bearing eyes replace the long continuous sleeve.
    # Both remain integral with the door and captured by the existing frame.
    for z0,z1 in ((42,54),(206,218)):
        door += block(3,HINGE_X,80,89,z0,z1)
        door += cyl_z(HINGE_X,HINGE_Y,z0,z1,6)
        door -= cyl_z(HINGE_X,HINGE_Y,z0-1,z1+1,3.3)
    guide = block(3,22,-78,-45,122,138)
    guide -= block(8.7,17.3,-79,-44,125.7,134.3)
    door += guide
    # Two shallow stiffening lands emphasise the guide mounting.
    door += block(3,6,-83,-40,116,122)
    door += block(3,6,-83,-40,138,144)

    # Delivered withdrawn: nose y=-84 is 3 mm inside the opening edge.
    # Positive slider coordinate translates this -Y; q=20 mm gives 10 mm
    # keeper engagement. Thumb flange reaches the guide at q=20 mm.
    bolt = block(9,17,-84,-17,126,134)
    bolt += block(8,26,-25,-17,124,136)
    # Fine transverse grip grooves, cut only in the accessible outer face.
    for z in (126,129,132):
        bolt -= block(25,27,-24,-18,z,z+0.7)

    # Separate welded grip body. Its COM lies in the thick front grip bar.
    # A body COM force follows the material point throughout the opening.
    handle = block(12,22,-86,-74,107,153)
    for z0,z1 in ((107,115),(145,153)):
        handle += block(4,12,-85,-75,z0,z1)
    # Move grip above bolt to avoid all bolt and guide swept material.
    handle = Pos(0,0,57)*handle

    bolt = Pos(0,-bolt_stroke_mm,0)*bolt
    swing = Pos(HINGE_X,HINGE_Y,0)*Rot(0,0,door_angle_deg)*Pos(-HINGE_X,-HINGE_Y,0)
    return {'door_frame':frame, 'hinge_rivet':rivet, 'door_leaf':swing*door,
            'sliding_bolt':swing*bolt, 'pull_handle':swing*handle}


def region(p0,p1,radius):
    return {'points':[list(p0),list(p1)],'radius':radius}


def assertion(identifier, criteria, purpose, expression):
    return {'id':identifier,'criterion_ids':criteria,'purpose':purpose,'expression':expression}


def proposed_tests():
    return [{
        'id':'manual_lock_release_open',
        'purpose':'Complete insertion, unassisted 10 N locked retention for1.1 s, manual withdrawal, and an opening pull at the same declared physical edge-grip point; retain all original function thresholds.',
        'duration_s':2.5,'step_s':0.001,
        'inputs':[
            {'joint':'bolt_slide','mode':'position','value':0.020,'gain':120,'damping':5,'max_effort':15,'start_s':0.0,'end_s':0.2},
            {'joint':'bolt_slide','mode':'position','value':0.0,'gain':180,'damping':5,'max_effort':15,'start_s':1.4,'end_s':2.5}
        ],
        'loads':[
            {'id':'locked_edge_pull','body':'door','force_n':[10,0,0],'point':[13,-80,187],'start_s':0.3,'end_s':1.4},
            {'id':'unlocked_edge_pull','body':'door','force_n':[10,0,0],'point':[13,-80,187],'start_s':1.8,'end_s':1.81}
        ],
        'assertions':[
            assertion('inserted_travel',['manual_bolt_operation'],'At least5 mm manual travel and >=19 mm actual operating insertion.',"min(window(signal('bolt_slide.coordinate'),0.18,0.2)) >= 0.019"),
            assertion('manual_force_limit',['manual_bolt_operation'],'Both manipulation directions stay within20 N.',"max(abs(signal('input.bolt_slide.effort'))) <= 20"),
            assertion('no_active_locked_hold',['locked_retention','geometric_locking'],'No manual input holds the bolt or door during locked loading.',"max(abs(window(signal('input.bolt_slide.effort'),0.3,1.4))) <= 0.000001"),
            assertion('locked_angle',['locked_retention'],'Door remains within2 degrees of closed throughout the1.1 s locked load.',"max(abs(window(signal('door_hinge.coordinate'),0.3,1.4))) <= 0.034906585"),
            assertion('locked_engagement',['locked_retention','geometric_locking'],'Maintain >=8 mm axial keeper overlap, subject to source-contact audit.',"min(window(signal('bolt_slide.coordinate'),0.3,1.4)) >= 0.018"),
            assertion('withdrawn_travel',['manual_bolt_operation'],'Actual insertion-to-withdrawal displacement >=5 mm.',"last(window(signal('bolt_slide.coordinate'),0.18,0.2)) - last(window(signal('bolt_slide.coordinate'),1.7,1.8)) >= 0.005"),
            assertion('withdrawn_clearance',['manual_bolt_operation'],'Withdraw to within0.5 mm of q0 before opening, leaving2.5 mm inboard clearance.',"max(abs(window(signal('bolt_slide.coordinate'),1.7,1.8))) <= 0.0005"),
            assertion('opening_45',['unlocked_opening'],'Reach45 degrees within0.7 s after opening force begins; original maximum deadline is3 s.',"max(window(signal('door_hinge.coordinate'),1.8,2.5)) >= 0.785398164"),
            assertion('remain_open',['unlocked_opening'],'Remain usefully open at the endpoint.',"last(signal('door_hinge.coordinate')) >= 0.785398164"),
            assertion('retained_hinge',['locked_retention','unlocked_opening'],'Door including welded grip stays attached through the supported hinge.',"min(signal('door_hinge.active')) >= 1"),
            assertion('retained_slider',['manual_bolt_operation'],'Bolt remains supported by its finite guide.',"min(signal('bolt_slide.active')) >= 1")
        ]
    }]


def build():
    shapes=make_parts()
    assembly_shapes=[]
    for name,shape in shapes.items():
        child=copy.copy(shape); child.label=name; assembly_shapes.append(child)
    export_step(Compound(label='small_door_sliding_bolt',children=assembly_shapes),Path(__file__).with_name('assembly.step'))
    joints=[
        {'id':'door_hinge','type':'revolute','body_a':'door','body_b':'frame',
         'anchor':[HINGE_X,HINGE_Y,48],'axis':[0,0,1],
         'region_a':region((HINGE_X,HINGE_Y,44),(HINGE_X,HINGE_Y,52),3.4),
         'region_b':region((HINGE_X,HINGE_Y,44),(HINGE_X,HINGE_Y,52),3.1),
         'support':{'shaft':{'body':'frame','center':[HINGE_X,HINGE_Y,48],'axis':[0,0,1],'radius':3.0,'length':8},
                    'bore':{'body':'door','center':[HINGE_X,HINGE_Y,48],'axis':[0,0,1],'radius':3.3,'length':8},'max_clearance':0.31}},
        {'id':'bolt_slide','type':'prismatic','body_a':'bolt','body_b':'door',
         'anchor':[BOLT_X,-21,BOLT_Z],'axis':[0,-1,0],
         'region_a':region((13,-80,130),(13,-28,130),4.4),
         'region_b':region((13,-76,130),(13,-47,130),4.4)}
    ]
    parts={
        'door_frame':Part(shapes['door_frame'],body='frame',role='fixed',density_kg_m3=7800,color=[0.23,0.29,0.36]),
        'hinge_rivet':Part(shapes['hinge_rivet'],body='frame',role='fixed',density_kg_m3=7800,color=[0.23,0.29,0.36]),
        'door_leaf':Part(shapes['door_leaf'],body='door',density_kg_m3=2700,color=[0.18,0.52,0.65]),
        'sliding_bolt':Part(shapes['sliding_bolt'],body='bolt',density_kg_m3=7800,color=[0.76,0.65,0.3]),
        'pull_handle':Part(shapes['pull_handle'],body='door',density_kg_m3=2700,color=[0.78,0.81,0.84])
    }
    return Asset(parts=parts,joints=joints,tests=proposed_tests(),gravity_m_s2=[0,0,-9.81],
                 description='Five physical parts: upright panel door, integral keeper frame and guide, retained hinge rivet, sliding bolt and welded pull handle.',
                 dependencies=['OPERATING.md','assembly.step','DEVELOPMENT.md','inspect_source.py','source_pose_facts.json','delivery_evidence.json','delivery_budget_checkpoint.json','RECOVERY.md','budget_decision.json','recovered_prefix_audit.json','CONTINUATION.md'])


def gen_step():
    shapes=make_parts()
    for name,shape in shapes.items(): shape.label=name
    return Compound(label='small_door_sliding_bolt',children=list(shapes.values()))


if __name__=='__main__':
    export_step(gen_step(),Path(__file__).with_name('assembly.step'))
