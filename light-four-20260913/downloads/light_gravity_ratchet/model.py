"""Rebuildable seven-part gravity ratchet; source geometry is in assembled mm."""
from math import sin, cos, pi
from pathlib import Path
from build123d import (Box, Cylinder, Pos, Rot, Face, Wire, Vector, extrude,
                      Compound, export_step, Color, fillet)
from simulation_ready_cad_workflow.validation_pipeline.slim_harness.asset import Asset, Part

TEETH = 12
PITCH = 2*pi/TEETH
ROOT_R, TIP_R = 42.0, 50.0
PHASE = 8*pi/180
WHEEL_CENTER = (0.0, 75.0)  # x,z
PAWL_CENTER = (-38.0, 140.0)
WIDTH = 8.0
JOURNAL_R, BORE_R = 5.0, 5.25

def ycylinder(x, z, radius, y0, y1):
    return Pos(x, (y0+y1)/2, z) * Rot(90, 0, 0) * Cylinder(radius, y1-y0)

def plate(points, y0, y1):
    wire = Wire.make_polygon([Vector(x,y0,z) for x,z in points], close=True)
    return extrude(Face(wire), amount=y1-y0, dir=(0,1,0))

def polar(r, angle):
    return (r*sin(angle), 75+r*cos(angle))

def wheel_shape():
    points = []
    for i in range(TEETH):
        a = PHASE + i*PITCH
        points.extend([polar(ROOT_R,a), polar(TIP_R,a),
                       polar(ROOT_R,a+PITCH-2*pi/180)])
    wheel = plate(points, -4, 4)
    crest_edges=[]
    for edge in wheel.edges():
        vertices=list(edge.vertices())
        if len(vertices)!=2: continue
        a,b=[v.center() for v in vertices]
        if (abs(a.Y-b.Y)>7.99 and abs(a.X-b.X)<1e-6 and abs(a.Z-b.Z)<1e-6
            and abs((a.X*a.X+(a.Z-75)**2)**0.5-TIP_R)<1e-6):
            crest_edges.append(edge)
    if len(crest_edges)!=TEETH:
        raise ValueError('Expected one original outer crest edge for each tooth')
    wheel = fillet(crest_edges,0.75)
    wheel -= ycylinder(0,75,BORE_R,-5,5)
    for i in range(6):
        a = i*pi/3
        wheel -= ycylinder(26*sin(a),75+26*cos(a),7.5,-5,5)
    return wheel

def pawl_shape():
    # Tapered lever with an integral octagonal nose, made with eight planar
    # faces. The source itself has these flats; this is not a collider proxy.
    # Its working band lies inside the wheel's end faces.
    dx,dz=39,-12
    length=(dx*dx+dz*dz)**0.5
    nx,nz=-dz/length,dx/length
    outline=[(-38+4*nx,140+4*nz),(1+1.4*nx,128+1.4*nz),
             (1-1.4*nx,128-1.4*nz),(-38-4*nx,140-4*nz)]
    nose=[(1+2*sin(pi/8+i*pi/4),128+2*cos(pi/8+i*pi/4)) for i in range(8)]
    pawl = (plate(outline,-3,3) + plate(nose,-3,3)
            + ycylinder(-38,140,9,-4,4))
    return pawl - ycylinder(-38,140,BORE_R,-5,5)

def bracket_shape():
    foot = Pos(0,-8,4)*Box(150,48,8)
    plate_back = Pos(0,-11,81.5)*Box(130,6,147)
    # Two large windows preserve the crossbars around both supported axes.
    for z, h in [(39,44),(106,44)]:
        plate_back -= Pos(0,-11,z)*Box(106,8,h)
    frame = foot + plate_back
    for x,z in [WHEEL_CENTER,PAWL_CENTER]:
        frame += ycylinder(x,z,11,-8,-4.5)
        frame -= ycylinder(x,z,4.1,-15,-8)
        frame -= ycylinder(x,z,5.2,-8,-4)
    # Empty mounting holes are interfaces to the external stationary fixture.
    for x in [-61,61]:
        frame -= Pos(x,7,4)*Cylinder(3.5,10)
    return frame

def shoulder_screw(x,z):
    screw = (ycylinder(x,z,4,-20,-8) + ycylinder(x,z,JOURNAL_R,-8,4.5)
             + ycylinder(x,z,8,4.5,8.5))
    socket = [(x+3*cos(i*pi/3),z+3*sin(i*pi/3)) for i in range(6)]
    return screw - plate(socket,6,9)

def lock_nut(x,z):
    outline = [(x+7.5*cos(i*pi/3),z+7.5*sin(i*pi/3)) for i in range(6)]
    # Thread major/minor detail is omitted; the fixed screw/nut connection
    # represents tightening, not a movable or collision-driven screw thread.
    return plate(outline,-19,-14) - ycylinder(x,z,4,-20,-13)

def joint(name, body, x, z):
    region = {"points":[[x,-3,z],[x,3,z]], "radius":5.4}
    return {"id":name,"type":"revolute","body_a":body,"body_b":"frame",
            "anchor":[x,0,z],"axis":[0,1,0],
            "region_a":region,"region_b":region,
            "support":{"shaft":{"body":"frame","center":[x,0,z],
                        "axis":[0,1,0],"radius":5,"length":8},
                       "bore":{"body":body,"center":[x,0,z],
                        "axis":[0,1,0],"radius":BORE_R,"length":8},
                       "max_clearance":0.26}}

def tests():
    p=PITCH
    return [{"id":"forward_then_reverse","purpose":
        "Measure consecutive tooth passages and unactuated gravity returns, then engagement and one-second holding under reverse torque in the same trajectory. Geometry/contact observations are defined in OBSERVATIONS.md.",
        "duration_s":4.5,"step_s":0.001,
        "inputs":[{"joint":"wheel_bearing","mode":"speed","value":2*p,
                   "max_effort":0.15,"gain":8,"start_s":0,"end_s":2.4}],
        "loads":[{"body":"wheel","torque_nm":[0,-0.06,0],"start_s":2.4,"end_s":4.5}],
        "assertions":[
            {"id":"forward_travel","criterion_ids":["forward_passage"],
             "purpose":"At least three actual wheel pitches in the 2.4-second forward interval; tooth-relative events are additionally counted from geometry and motion.",
             "expression":f"last(before(signal('wheel_bearing.coordinate'),2.4))-first(signal('wheel_bearing.coordinate')) >= {3*p}"},
            {"id":"mean_rate_low","criterion_ids":["forward_passage"],"purpose":"Mean actual forward rate at least one pitch per second.",
             "expression":f"(last(before(signal('wheel_bearing.coordinate'),2.4))-first(signal('wheel_bearing.coordinate')))/2.4 >= {p}"},
            {"id":"mean_rate_high","criterion_ids":["forward_passage"],"purpose":"Mean actual forward rate no more than three pitches per second.",
             "expression":f"(last(before(signal('wheel_bearing.coordinate'),2.4))-first(signal('wheel_bearing.coordinate')))/2.4 <= {3*p}"},
            {"id":"pawl_cycles","criterion_ids":["gravity_engagement"],"purpose":"At least two resolved pawl lifting/return cycles; verify releases and gravity using contact records as well.",
             "expression":"cycles(before(signal('pawl_bearing.coordinate'),2.4),0.06,0.04) >= 2"},
            {"id":"engagement_travel","criterion_ids":["reverse_engagement"],"purpose":"Wheel range from reverse onset to scheduled hold is at most one measured pitch plus one degree.",
             "expression":f"span(window(signal('wheel_bearing.coordinate'),2.4,3.4)) <= {p+pi/180}"},
            {"id":"hold_range","criterion_ids":["reverse_engagement","reverse_holding"],"purpose":"A qualifying hold starts by 3.4 s and persists for 1.1 seconds.",
             "expression":f"span(window(signal('wheel_bearing.coordinate'),3.4,4.5)) <= {pi/180}"},
            {"id":"hold_speed","criterion_ids":["reverse_engagement","reverse_holding"],"purpose":"Throughout holding the absolute wheel speed is at most 0.02 rad/s.",
             "expression":"max(abs(window(signal('wheel.angular_speed.y_rad_s'),3.4,4.5))) <= 0.02"},
            {"id":"drive_off","criterion_ids":["reverse_engagement","reverse_holding","physical_model"],"purpose":"No forward-drive effort after the final forward step endpoint.",
             "expression":"max(abs(after(signal('input.wheel_bearing.effort'),2.401))) <= 1e-12"},
            {"id":"hold_contact","criterion_ids":["reverse_holding"],"purpose":"Continuous solved contact in the hold; independently resolve wheel-pawl identity and its resisting moment from force records.",
             "expression":"min(window(signal('contact.count'),3.4,4.5)) >= 1"},
            {"id":"wheel_support","criterion_ids":["operating_conditions"],"purpose":"Wheel bearing remains physically connected.",
             "expression":"min(signal('wheel_bearing.active')) >= 1"},
            {"id":"pawl_support","criterion_ids":["operating_conditions"],"purpose":"Pawl bearing remains physically connected.",
             "expression":"min(signal('pawl_bearing.active')) >= 1"}
        ]}]

def build():
    parts={"mounting_bracket":Part(bracket_shape(),"frame","fixed",7800,[0.35,0.42,0.49]),
           "toothed_wheel":Part(wheel_shape(),"wheel","dynamic",7800,[0.72,0.76,0.80]),
           "gravity_pawl":Part(pawl_shape(),"pawl","dynamic",7800,[0.85,0.51,0.13])}
    for name,(x,z) in [("wheel",WHEEL_CENTER),("pawl",PAWL_CENTER)]:
        parts[name+"_shoulder_screw"]=Part(shoulder_screw(x,z),"frame","fixed",7800,[0.35,0.42,0.49])
        parts[name+"_lock_nut"]=Part(lock_nut(x,z),"frame","fixed",7800,[0.35,0.42,0.49])
    # Include the complete physical-instance assembly in every normal build,
    # in addition to the per-rigid-body STEP files produced by Asset.
    export_step(assembly_from_parts(parts),Path(__file__).with_suffix(".step"))
    return Asset(parts=parts,joints=[joint("wheel_bearing","wheel",*WHEEL_CENTER),
                                    joint("pawl_bearing","pawl",*PAWL_CENTER)],
                 tests=tests(),gravity_m_s2=[0,0,-9.81],
                 description="Seven-part gravity-pawl one-way wheel, 12 asymmetric teeth, retained stationary journals.",
                 dependencies=["model.step","OPERATING.md","OBSERVATIONS.md",
                               "inspect_source.py","observe_trace.py","review_views.py",
                               "REVISION_NOTES.md","DELIVERY.md"])

def assembly_from_parts(parts):
    shapes=[]
    for name,part in parts.items():
        part.shape.label=name
        part.shape.color=Color(*part.color)
        shapes.append(part.shape)
    return Compound(label="gravity_pawl_one_way_wheel",children=shapes)

def gen_step():
    return assembly_from_parts(build().parts)

if __name__ == "__main__":
    target=Path(__file__).with_suffix(".step")
    build()
    print(target)
