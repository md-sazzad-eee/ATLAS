#!/usr/bin/env python3
"""Convert atlas_params.yaml -> atlas_params.json and print derived checks.

MATLAB (jsondecode), FreeCAD and COMSOL scripts read the JSON; humans edit the YAML.
"""
import json, math, pathlib, sys
import yaml

HERE = pathlib.Path(__file__).resolve().parent
src = HERE / "atlas_params.yaml"
dst = HERE / "atlas_params.json"

p = yaml.safe_load(src.read_text())

# ---- derived geometry checks (fail loudly if the interface is inconsistent)
a, d, c = p["agent"], p["dock"], p["carrier"]
bay_inner = a["body_width_mm"] + 2 * d["side_clearance_mm"]
bay_outer = bay_inner + 2 * d["guide_thickness_mm"]
mouth_outer = bay_outer + 2 * d["funnel_flare_mm"]
deck_span = d["bay_pitch_mm"] * (d["bays"] - 1) + mouth_outer
derived = {
    "bay_inner_width_mm": bay_inner,
    "bay_outer_width_mm": bay_outer,
    "funnel_mouth_outer_width_mm": mouth_outer,
    "deck_span_required_mm": deck_span,
    "ramp_length_mm": round(d["deck_height_mm"] / math.tan(math.radians(d["ramp_angle_deg"])), 1),
}
errors = []
if mouth_outer > d["bay_pitch_mm"]:
    errors.append(f"adjacent funnel mouths overlap: {mouth_outer} > pitch {d['bay_pitch_mm']}")
if deck_span > c["body_width_mm"]:
    errors.append(f"bays ({deck_span} mm) wider than carrier ({c['body_width_mm']} mm)")
bay_depth = d["funnel_length_mm"] + d["guide_length_mm"]
derived["bay_depth_mm"] = bay_depth
derived["carrier_wheel_inner_face_x_mm"] = c["track_width_mm"] / 2 - c["wheel_width_mm"] / 2
derived["carrier_overall_width_mm"] = c["track_width_mm"] + c["wheel_width_mm"]
derived["carrier_length_used_mm"] = (derived["ramp_length_mm"] + bay_depth + d["front_stop_thickness_mm"]
                                     + d["latch_housing_depth_mm"] + d["carrier_front_reserve_mm"])
if bay_depth < a["body_length_mm"]:
    errors.append(f"bay depth {bay_depth} mm shorter than agent {a['body_length_mm']} mm: agent sticks out")
if derived["carrier_wheel_inner_face_x_mm"] < deck_span / 2 + 5:
    errors.append("carrier wheels intrude into the docking bays")
if derived["carrier_length_used_mm"] > c["body_length_mm"]:
    errors.append(f"ramp+bays+front reserve ({derived['carrier_length_used_mm']:.0f}) longer than carrier")
if d["front_stop_height_mm"] < d["contact_height_above_deck_mm"] + 10:
    errors.append("front stop too low for contact block")
if a["track_width_mm"] + a["wheel_width_mm"] > a["body_width_mm"]:
    errors.append("agent wheels protrude beyond body width")

p["_derived"] = derived
dst.write_text(json.dumps(p, indent=2))
print(f"wrote {dst.name}")
for k, v in derived.items():
    print(f"  {k:32s} {v}")
if errors:
    print("\nINTERFACE ERRORS:")
    for e in errors:
        print("  -", e)
    sys.exit(1)
print("interface checks: OK")
