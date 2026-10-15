"""Print one example per DRC error type and the unconnected list: python3 drc_summary.py report.json"""
import json, sys, collections
d = json.load(open(sys.argv[1]))
vs = [x for x in d["violations"] if x["severity"] == "error"]
print(dict(collections.Counter(x["type"] for x in vs)))
seen = collections.Counter()
for v in vs:
    seen[v["type"]] += 1
    if seen[v["type"]] > 2:
        continue
    print(v["type"], "|", v["description"][:100])
    for i in v["items"]:
        print("    ", round(i["pos"]["x"], 2), round(i["pos"]["y"], 2), i["description"][:90])
print("unconnected:", len(d.get("unconnected_items", [])), " parity:", len(d.get("schematic_parity", [])))
