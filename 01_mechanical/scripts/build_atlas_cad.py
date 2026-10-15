"""ATLAS parametric CAD generator for FreeCAD (Part workbench, no GUI needed).

Run inside FreeCAD (Macro > Execute, the Python console, or freecadcmd):
    exec(open('<ATLAS>/01_mechanical/scripts/build_atlas_cad.py').read())

Everything is driven by 00_requirements/atlas_params.json. Edit the YAML, run
sync_params.py, re-run this script: all parts, assemblies and exports regenerate.

Coordinate frame (carrier):  X across width (+ right), Y forward (rear edge y=0),
Z up (floor z=0). Agents drive FORWARD (+Y) up the rear ramp into the bays and stop
with their body front against the front stop. Pogo pins protrude from a pocket in the
stop; the agent's pads are recessed, so pin preload = protrusion - recess.

Outputs
  01_mechanical/docking_system/ATLAS_docking_bay.FCStd       one bay, all named parts
  01_mechanical/atlas_scout/ATLAS_agent.FCStd                agent chassis + docking face
  01_mechanical/atlas_main/ATLAS_carrier_assembly.FCStd      carrier + 2 bays + 2 agents
  01_mechanical/common_interfaces/*.step                     interchange exports
  01_mechanical/docking_system/test_coupons/*.stl            clearance test coupons for printing
  01_mechanical/docking_system/cad_check.json                interference / clearance report
"""
import json, math, os
import FreeCAD as App
import Part

V = App.Vector
try:
    ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
except NameError:  # exec() in the FreeCAD console has no __file__
    ROOT = os.environ.get("ATLAS_ROOT", os.getcwd())   # set ATLAS_ROOT to the repository root
P = json.load(open(os.path.join(ROOT, "00_requirements", "atlas_params.json")))
A, D, C = P["agent"], P["dock"], P["carrier"]

# ---------------------------------------------------------------- dimensions
c_side   = D["side_clearance_mm"]
t_g      = D["guide_thickness_mm"]
h_g      = D["guide_height_mm"]
L_fun    = D["funnel_length_mm"]
flare    = D["funnel_flare_mm"]
L_g      = D["guide_length_mm"]
t_stop   = D["front_stop_thickness_mm"]
h_stop   = D["front_stop_height_mm"]
z_deck   = D["deck_height_mm"]
ramp_ang = math.radians(D["ramp_angle_deg"])
L_ramp   = z_deck / math.tan(ramp_ang)
W_in     = A["body_width_mm"] + 2 * c_side          # bay inner width
L_bay    = L_fun + L_g
t_base   = 4.0
pin_d    = D["latch"]["pin_diameter_mm"]
pin_trav = D["latch"]["pin_travel_mm"]
hole_d   = pin_d + D["latch"]["hole_clearance_mm"]
z_latch  = z_deck + D["latch_height_above_deck_mm"]
z_cont   = z_deck + D["contact_height_above_deck_mm"]
n_cont   = D["contacts"]["count"]
p_cont   = D["contacts"]["pitch_mm"]

A_L, A_W, A_H = A["body_length_mm"], A["body_width_mm"], A["body_height_mm"]
A_gc   = A["ground_clearance_mm"]
wheel_r = A["wheel_diameter_mm"] / 2
wheel_w = A["wheel_width_mm"]
track  = A["track_width_mm"]
axle_from_front = A["wheel_axle_from_front_mm"]
# Front catch-tab latch (rev 0.1.2). The agent carries a vertical tab protruding forward
# through a slot in the front stop; a solenoid cross-pin (along X) passes through the tab
# hole inside the latch housing. Side latches collided with the chassis / the other bay.
tab_x_off = 29.0              # tab centre from agent centreline (clear of contact block & sensor)
tab_t, tab_h, tab_len = 5.0, 14.0, 20.0
tab_hole_y = 12.0             # hole position from agent front face
slot_clear = 1.5              # per side, tab-to-slot
h_house = D["latch_housing_depth_mm"]
pin_protrusion = 2.0          # pogo pin protrusion from the stop face
pad_recess = 1.0              # agent pads recessed behind the body front


def box(lx, ly, lz, x0, y0, z0):
    return Part.makeBox(lx, ly, lz, V(x0, y0, z0))


def prism_xy(pts, z0, h):
    """Extrude a closed XY polygon (list of (x,y)) from z0 by h."""
    w = Part.makePolygon([V(x, y, z0) for x, y in pts] + [V(pts[0][0], pts[0][1], z0)])
    return Part.Face(w).extrude(V(0, 0, h))


def prism_yz(pts, x0, w):
    """Extrude a closed YZ polygon (list of (y,z)) from x0 by w along +X."""
    wire = Part.makePolygon([V(x0, y, z) for y, z in pts] + [V(x0, pts[0][0], pts[0][1])])
    return Part.Face(wire).extrude(V(w, 0, 0))


def cyl(r, h, base, axis):
    return Part.makeCylinder(r, h, base, axis)


# ================================================================ docking bay
def make_bay(xc=0.0, y0=0.0):
    """Return dict name->shape for one bay whose mouth is at y=y0 and centre x=xc.
    The ramp lies behind the mouth (y < y0)."""
    S = {}
    xi = W_in / 2
    S["Ramp"] = prism_yz([(y0 - L_ramp, 0), (y0, z_deck), (y0, 0)], xc - xi, W_in)
    S["Dock_Base"] = box(W_in + 2 * (t_g + flare), L_bay + t_stop, t_base,
                         xc - xi - t_g - flare, y0, z_deck - t_base)
    # funnel lead-ins (left/right), mouth wider by `flare`, throat = bay inner width
    for side, sgn in (("Left", -1), ("Right", 1)):
        inner = [(xc + sgn * (xi + flare), y0), (xc + sgn * xi, y0 + L_fun)]
        outer = [(xc + sgn * (xi + t_g), y0 + L_fun), (xc + sgn * (xi + flare + t_g), y0)]
        S[f"{side}_Funnel"] = prism_xy(inner + outer, z_deck, h_g)
        x0 = xc + xi if sgn > 0 else xc - xi - t_g
        S[f"{side}_Guide"] = box(t_g, L_g, h_g, x0, y0 + L_fun, z_deck)
    y_stop = y0 + L_bay
    cb_w = (n_cont - 1) * p_cont + 10
    pocket = box(cb_w, 6, 14, xc - cb_w / 2, y_stop, z_cont - 7)
    S["Front_Stop"] = box(W_in + 2 * t_g, t_stop, h_stop, xc - xi - t_g, y_stop, z_deck).cut(pocket)
    S["Contact_Block"] = pocket                      # insulated block sits flush in the pocket
    pins = None
    for k in range(n_cont):
        xk = xc - (n_cont - 1) * p_cont / 2 + k * p_cont
        pin = cyl(D["contacts"]["pin_tip_diameter_mm"] / 2, pin_protrusion, V(xk, y_stop, z_cont), V(0, -1, 0))
        pins = pin if pins is None else pins.fuse(pin)
    S["Contact_Pins"] = pins
    # seated sensor (optical interrupter) beside the contact block
    S["Seated_Sensor"] = box(12, 5, 10, xc + cb_w / 2 + 6, y_stop, z_deck + 6)   # flush reflective sensor
    S["Front_Stop"] = S["Front_Stop"].cut(S["Seated_Sensor"])
    # ---- front catch-tab latch: slot through stop + housing, cross pin along X (shown LOCKED)
    xt = xc + tab_x_off
    slot = box(tab_t + 2 * slot_clear, t_stop + h_house - 4, tab_h + 2 * slot_clear,
               xt - tab_t / 2 - slot_clear, y_stop - 0.1, z_latch - tab_h / 2 - slot_clear)
    S["Front_Stop"] = S["Front_Stop"].cut(slot)
    house = box(60, h_house, 50, xc + 12, y_stop + t_stop, z_deck).cut(slot)
    y_pin = y_stop + tab_hole_y
    x_left_wall = xt - tab_t / 2 - slot_clear          # slot left wall
    bore = cyl(pin_d / 2 + 0.2, 40, V(x_left_wall - 3, y_pin, z_latch), V(1, 0, 0))
    S["Latch_Housing"] = house.cut(bore)
    x_tip = x_left_wall - 2.5                          # extended: 2.5 mm into the far wall
    S["Latch_Pin"] = cyl(pin_d / 2, 22, V(x_tip, y_pin, z_latch), V(1, 0, 0))
    S["Latch_Body"] = cyl(7.0, 25.0, V(x_tip + 22, y_pin, z_latch), V(1, 0, 0))     # solenoid
    S["Actuator_Mount"] = box(27, 18, 4, x_tip + 21, y_pin - 9, z_latch - 11)
    S["Latch_Hall_Sensor"] = box(6, 6, 6, x_tip + 14, y_pin + 7, z_latch - 3)
    S["_pin_travel_mm"] = (xt + tab_t / 2 + 0.5) - x_tip                            # tip must clear the tab
    return S


# ================================================================ agent
def make_agent(xc=0.0, y_front=0.0, z_floor=0.0):
    """Agent with its FRONT face at y=y_front, centred on x=xc, wheels on z=z_floor."""
    S = {}
    y0 = y_front - A_L
    body = box(A_W, A_L, A_H, xc - A_W / 2, y0, z_floor + A_gc)
    # wheel wells
    y_ax = y_front - axle_from_front
    for sgn in (-1, 1):
        xw = xc + sgn * track / 2
        well = box(wheel_w + 4, 2 * wheel_r + 6, 2 * wheel_r + 6,
                   xw - (wheel_w + 4) / 2, y_ax - wheel_r - 3, z_floor - 3)
        body = body.cut(well)
    # forward catch tab with cross hole (matches the dock latch pin)
    zt = z_floor + (z_latch - z_deck)
    tab = box(tab_t, tab_len, tab_h, xc + tab_x_off - tab_t / 2, y_front, zt - tab_h / 2)
    tab = tab.cut(cyl(hole_d / 2, tab_t + 2, V(xc + tab_x_off - tab_t / 2 - 1, y_front + tab_hole_y, zt), V(1, 0, 0)))
    body = body.fuse(tab)
    S["Agent_Body"] = body
    for side, sgn in (("L", -1), ("R", 1)):
        xw = xc + sgn * track / 2 - wheel_w / 2
        S[f"Agent_Wheel_{side}"] = cyl(wheel_r, wheel_w, V(xw, y_ax, z_floor + wheel_r), V(1, 0, 0))
    S["Agent_Caster"] = Part.makeSphere(A_gc / 2, V(xc, y0 + 15, z_floor + A_gc / 2))
    # docking face: insulated plate in a pocket in the body front, pads recessed by pad_recess
    zc = z_floor + (z_cont - z_deck)
    S["Agent_Body"] = S["Agent_Body"].cut(box(44, 4, 30, xc - 22, y_front - 4, zc - 15))
    face = box(44, 2, 30, xc - 22, y_front - 4, zc - 15)
    pw, ph = D["contacts"]["pad_size_mm"]
    pad_t = 4 - 2 - pad_recess
    for k in range(n_cont):
        xk = xc - (n_cont - 1) * p_cont / 2 + k * p_cont
        face = face.fuse(box(pw, pad_t, ph, xk - pw / 2, y_front - 2, zc - ph / 2))
    S["Agent_Docking_Face"] = face
    return S


# ================================================================ carrier
def make_carrier():
    S = {}
    L, W, H = C["body_length_mm"], C["body_width_mm"], C["body_height_mm"]
    gc = C["ground_clearance_mm"]
    rw = C["wheel_diameter_mm"] / 2
    ww = C["wheel_width_mm"]
    y_mouth = L_ramp
    chassis = box(W, L, H - gc, -W / 2, 0, gc)
    pitch = D["bay_pitch_mm"]
    # bay pocket: open at the rear, from floor to top, as deep as ramp+bay+stop
    pocket_w = (D["bays"] - 1) * pitch + W_in + 2 * (t_g + flare) + 2
    chassis = chassis.cut(box(pocket_w, y_mouth + L_bay + t_stop + h_house + 1, H + 1,
                              -pocket_w / 2, -1, -1))
    S["Carrier_Chassis"] = chassis
    S["Carrier_Bay_Pan"] = box(pocket_w, L_bay + t_stop + h_house, z_deck - t_base, -pocket_w / 2, y_mouth, 0)
    yw = (L - C["wheelbase_mm"]) / 2
    for i, y in enumerate((yw, L - yw)):
        for side, sgn in (("L", -1), ("R", 1)):
            x0 = sgn * C["track_width_mm"] / 2 - ww / 2
            S[f"Carrier_Wheel_{side}{i}"] = cyl(rw, ww, V(x0, y, rw), V(1, 0, 0))
    return S, y_mouth


def add(doc, S, prefix=""):
    objs = {}
    for name, shp in S.items():
        if name.startswith("_"):
            continue
        o = doc.addObject("Part::Feature", prefix + name)
        o.Shape = shp
        objs[name] = o
    return objs


def save(doc, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc.recompute()
    doc.saveAs(path)


# ================================================================ build
out = {"params_version": P["meta"]["version"]}
mech = os.path.join(ROOT, "01_mechanical")

# 1) single bay + seated agent, for interference check
doc = App.newDocument("ATLAS_docking_bay")
bay = make_bay(0, 0)
add(doc, bay)
agent_seated = make_agent(0, L_bay, z_deck)   # agent front touching the stop, on the deck
add(doc, agent_seated, "Seated_")
save(doc, os.path.join(mech, "docking_system", "ATLAS_docking_bay.FCStd"))

static = ["Dock_Base", "Left_Guide", "Right_Guide", "Left_Funnel", "Right_Funnel", "Front_Stop", "Ramp"]
checks = {}
body = agent_seated["Agent_Body"]
for n in static:
    v = body.common(bay[n]).Volume
    checks[f"agent_body_vs_{n}_overlap_mm3"] = round(v, 3)
    checks[f"agent_body_to_{n}_min_dist_mm"] = round(body.distToShape(bay[n])[0], 3)
for wn in ("Agent_Wheel_L", "Agent_Wheel_R"):
    for n in ("Left_Guide", "Right_Guide", "Left_Funnel", "Right_Funnel"):
        checks[f"{wn}_to_{n}_min_dist_mm"] = round(agent_seated[wn].distToShape(bay[n])[0], 3)
# latch pin extended must pass through the tab hole without touching the agent
checks["latch_pin_vs_agent_body_overlap_mm3"] = round(bay["Latch_Pin"].common(body).Volume, 3)
checks["latch_pin_vs_housing_overlap_mm3"] = round(bay["Latch_Pin"].common(bay["Latch_Housing"]).Volume, 3)
checks["latch_required_travel_mm"] = round(bay["_pin_travel_mm"], 2)
checks["latch_travel_available_mm"] = pin_trav
checks["agent_tab_vs_front_stop_overlap_mm3"] = round(body.common(bay["Front_Stop"]).Volume, 3)
checks["agent_tab_vs_housing_overlap_mm3"] = round(body.common(bay["Latch_Housing"]).Volume, 3)
checks["agent_tab_to_slot_min_dist_mm"] = round(body.distToShape(bay["Latch_Housing"])[0], 3)
for wn in ("Agent_Wheel_L", "Agent_Wheel_R"):
    checks[f"latch_pin_to_{wn}_min_dist_mm"] = round(bay["Latch_Pin"].distToShape(agent_seated[wn])[0], 3)
# pogo preload: pins overlap the pads by (protrusion - recess); must be > 0 and < pin travel (~1.5 mm)
checks["pogo_preload_mm"] = pin_protrusion - pad_recess
checks["pogo_pins_vs_pads_overlap_mm3"] = round(bay["Contact_Pins"].common(agent_seated["Agent_Docking_Face"]).Volume, 3)
checks["pogo_pins_vs_agent_body_overlap_mm3"] = round(bay["Contact_Pins"].common(body).Volume, 3)
out["single_bay"] = checks

# 2) agent part file
doc2 = App.newDocument("ATLAS_agent")
ag = make_agent(0, 0, 0)
add(doc2, ag)
save(doc2, os.path.join(mech, "atlas_scout", "ATLAS_agent.FCStd"))
agent_mass_vol = ag["Agent_Body"].Volume

# 3) full carrier assembly with two docked agents
doc3 = App.newDocument("ATLAS_carrier_assembly")
car, y_mouth = make_carrier()
add(doc3, car)
pitch = D["bay_pitch_mm"]
xs = [(-(D["bays"] - 1) / 2 + k) * pitch for k in range(D["bays"])]
bays, agents = [], []
for k, xc in enumerate(xs):
    b = make_bay(xc, y_mouth); add(doc3, b, f"Bay{k+1}_"); bays.append(b)
    a = make_agent(xc, y_mouth + L_bay, z_deck); add(doc3, a, f"Agent{k+1}_"); agents.append(a)
save(doc3, os.path.join(mech, "atlas_main", "ATLAS_carrier_assembly.FCStd"))

asm = {}
asm["agent1_to_agent2_min_dist_mm"] = round(agents[0]["Agent_Body"].distToShape(agents[1]["Agent_Body"])[0], 3)
asm["bay1_to_bay2_funnel_min_dist_mm"] = round(bays[0]["Right_Funnel"].distToShape(bays[1]["Left_Funnel"])[0], 3)
for k, b in enumerate(bays):
    for part in ("Latch_Body", "Latch_Housing", "Actuator_Mount"):
        asm[f"bay{k+1}_{part}_to_chassis_min_dist_mm"] = round(b[part].distToShape(car["Carrier_Chassis"])[0], 3)
        other = bays[1 - k]
        asm[f"bay{k+1}_{part}_to_other_bay_min_dist_mm"] = round(
            min(b[part].distToShape(other[n])[0] for n in other if not n.startswith("_")), 3)
for k, a in enumerate(agents):
    for wn, w in car.items():
        if wn.startswith("Carrier_Wheel"):
            asm[f"agent{k+1}_to_{wn}_min_dist_mm"] = round(a["Agent_Body"].distToShape(w)[0], 3)
    asm[f"agent{k+1}_vs_chassis_overlap_mm3"] = round(a["Agent_Body"].common(car["Carrier_Chassis"]).Volume, 3)
asm["overall_length_incl_nothing_protruding_mm"] = C["body_length_mm"]
asm["overall_width_mm"] = C["track_width_mm"] + C["wheel_width_mm"]
out["carrier_assembly"] = asm

# 4) STEP exports (interchange; FCStd stays the editable source)
ci = os.path.join(mech, "common_interfaces"); os.makedirs(ci, exist_ok=True)
Part.makeCompound([v for k, v in bay.items() if not k.startswith('_')]).exportStep(os.path.join(ci, "ATLAS_docking_bay.step"))
Part.makeCompound(list(ag.values())).exportStep(os.path.join(ci, "ATLAS_agent.step"))
Part.makeCompound([v for d in [car] + bays + agents for k, v in d.items() if not k.startswith('_')]).exportStep(
    os.path.join(ci, "ATLAS_carrier_assembly.step"))
ag["Agent_Docking_Face"].exportStep(os.path.join(ci, "Agent_Docking_Face.step"))

# 5) clearance test coupons: short base + two guides at each test clearance
cp = os.path.join(mech, "docking_system", "test_coupons"); os.makedirs(cp, exist_ok=True)
coupons = []
for c in D["clearance_test_set_mm"]:
    w_in = A_W + 2 * c
    L_c = 40.0
    base = box(w_in + 2 * t_g, L_c, 3, -w_in / 2 - t_g, 0, 0)
    gl = box(t_g, L_c, h_g, -w_in / 2 - t_g, 0, 3)
    gr = box(t_g, L_c, h_g, w_in / 2, 0, 3)
    shp = base.fuse(gl).fuse(gr).removeSplitter()
    fn = os.path.join(cp, f"Dock_Test_Gauge_clear_{c:.1f}mm.stl")
    shp.exportStl(fn)
    coupons.append(os.path.basename(fn))
out["test_coupons"] = coupons

json.dump(out, open(os.path.join(mech, "docking_system", "cad_check.json"), "w"), indent=2)
print(json.dumps(out, indent=2))
