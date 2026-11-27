"""Onshape glTF -> web GLB for docs/arm.glb. Usage: python tools/build_arm_glb.py "Assembly.gltf" docs/arm.glb

Strips Onshape metadata, turns Z-up into Y-up, makes printed (blue) parts light gray PLA, keeps motor grays."""
import base64, json, struct, sys
src, dst = sys.argv[1], sys.argv[2]
d = json.load(open(src, encoding="utf-8"))
assert len(d["buffers"]) == 1
bin_ = base64.b64decode(d["buffers"][0].pop("uri").split(",", 1)[1])

def strip(o):
    if isinstance(o, dict):
        ext = o.get("extensions")
        if isinstance(ext, dict):
            ext.pop("PTC_onshape_metadata", None)
            if not ext: o.pop("extensions")
        for v in o.values(): strip(v)
    elif isinstance(o, list):
        for v in o: strip(v)
strip(d)
d["extensionsUsed"] = [e for e in d.get("extensionsUsed", []) if e != "PTC_onshape_metadata"]
if not d["extensionsUsed"]: d.pop("extensionsUsed")

for m in d["materials"]:
    p = m.setdefault("pbrMetallicRoughness", {})
    r, g, b, a = p.get("baseColorFactor", [1, 1, 1, 1])
    is_gray = max(r, g, b) - min(r, g, b) < 0.03  # Onshape default part colors are blues; grays are the motors
    if not is_gray:
        p["baseColorFactor"] = [0.62, 0.62, 0.6, 1.0]  # light gray PLA (linear)
        m["name"] = "PLA light gray"
    p["metallicFactor"] = 0.0
    p["roughnessFactor"] = 0.55 if not is_gray else 0.45  # matte-ish plastic

# Onshape is Z-up, glTF is Y-up: wrap the scene in a -90 deg rotation about X
scene = d["scenes"][d.get("scene", 0)]
d["nodes"].append({"name": "Y-up", "rotation": [-0.70710678, 0, 0, 0.70710678], "children": scene["nodes"]})
scene["nodes"] = [len(d["nodes"]) - 1]

js = json.dumps(d, separators=(",", ":")).encode(); js += b" " * (-len(js) % 4)
bin_ += b"\0" * (-len(bin_) % 4)
total = 28 + len(js) + len(bin_)
with open(dst, "wb") as f:
    f.write(struct.pack("<4sII", b"glTF", 2, total))
    f.write(struct.pack("<I4s", len(js), b"JSON") + js)
    f.write(struct.pack("<I4s", len(bin_), b"BIN\0") + bin_)
print("wrote", total, "bytes;", sum(m["name"] == "PLA light gray" for m in d["materials"]), "materials -> PLA")
