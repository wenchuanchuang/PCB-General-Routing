# Use KiCad bundled python.exe
# Read footprints (and their components) from KiCad pcbnew and export to YAML

import pcbnew, math, os
import re

# ---- KiCad 9 specific ----

PCB_PATH = r"REPLACE_WITH_YOUR_PCB_PATH\XXX.kicad_pcb"
board = pcbnew.LoadBoard(PCB_PATH)
if not board:
    raise RuntimeError("No PCB (.kicad_pcb) loaded.")

# Output file path: same directory as the .kicad_pcb file
pcb_path = board.GetFileName()
base_dir  = os.path.dirname(pcb_path)
yaml_dir = os.path.join(base_dir, "yaml")
if not os.path.exists(yaml_dir):
    os.makedirs(yaml_dir)
out_path = os.path.join(yaml_dir, "components.yaml")


# Helper functions

# Unit conversion
def iu_to_mm(iu):
    return float(pcbnew.ToMM(iu))
def get_deg(fp):
    return float(fp.GetOrientationDegrees())
def get_pad_name(pad):
    return pad.GetName()
def get_value_text(fp):
    return str(fp.GetValueText())
# Inverse rotation (-deg)
def rot_inv(dx_mm, dy_mm, deg):
    th = -math.radians(deg)
    cx = dx_mm * math.cos(th) - dy_mm * math.sin(th)
    cy = dx_mm * math.sin(th) + dy_mm * math.cos(th)
    return cx, cy



def _yaml_unquote(s: str) -> str:
    s = s.strip()
    if s.startswith("'") and s.endswith("'"):
        s = s[1:-1].replace("''", "'")
    return s

# Extract (value, footprint) pairs from an existing components.yaml
# and use them as the deduplication basis.
def load_seen_from_yaml(path: str):

    seen_file = set()
    if not os.path.exists(path):
        return seen_file

    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    cur_val = None
    for ln in lines:
        ls = ln.lstrip()
        if ls.startswith("- value:"):   # Example: - value: 'MX30LF1G08AA'
            val = ls.split(":", 1)[1].strip()
            cur_val = _yaml_unquote(val)
        elif cur_val is not None and ls.startswith("footprint:"):   # Example: footprint: 'Jonas:VFBGA63'
            fp = ls.split(":", 1)[1].strip()
            fp = _yaml_unquote(fp)
            key = (cur_val.strip().lower(), fp.strip().lower())
            seen_file.add(key)
            cur_val = None  

    return seen_file


seen = set() # Used to track already exported (value, footprint) pairs
seen |= load_seen_from_yaml(out_path)  # Include (value, footprint) pairs from the existing file


data = {"components": []}


update_amount = 0 
for fp in board.GetFootprints():


    value = fp.Value().GetText()

    fpid    = fp.GetFPID()
    fp_name = fpid.GetLibItemName()
    libnick = fpid.GetLibNickname()
    footprint_full = f"{libnick}:{fp_name}" if libnick else fp_name

    # Footprint position on board (mm) and rotation (deg)
    fp_pos = fp.GetPosition()
    fp_x_mm = iu_to_mm(fp_pos.x)
    fp_y_mm = iu_to_mm(fp_pos.y)
    fp_deg  = get_deg(fp)

    # Whether the footprint is on the back side
    flipped = fp.IsFlipped()

    pads_out = []

    for pad in fp.Pads():
        pad_name = get_pad_name(pad)

        ppos = pad.GetPosition()
        px_mm = iu_to_mm(ppos.x)
        py_mm = iu_to_mm(ppos.y)

        # Convert back to the footprint-local coordinate system:
        # first translate so the footprint origin is the center,
        # then apply inverse rotation of the footprint angle
        dx_mm = px_mm - fp_x_mm
        dy_mm = py_mm - fp_y_mm
        lx_mm, ly_mm = rot_inv(dx_mm, dy_mm, fp_deg)

        # If the footprint is on the back side, mirror X so that
        # left/right remains consistent from the front-view perspective
        if flipped:
            lx_mm = -lx_mm

        pads_out.append({
            "pin": pad_name,               
            "x_mm": round(lx_mm, 6),           
            "y_mm": round(ly_mm, 6),
        })


    
    val_norm = (value or "").strip()
    fp_norm  = (footprint_full or "").strip()
    key = (val_norm.lower(), fp_norm.lower())
    if key in seen:
        continue 
    seen.add(key)
    update_amount += 1

    comp = {
        "value": value,                
        "footprint": footprint_full,          
        "pin_count": len(pads_out),          
        "pads": pads_out,
    }
    data["components"].append(comp)

print(f"Updated {update_amount} components")


# --------- Simple YAML output ---------
def yaml_escape(s):
    if s is None:
        return "''"
    s = str(s)
    if any(c in s for c in [":","{","}","[","]",",","#","&","*","!","|",">","'","\"","%","@","`"]):
        return "'" + s.replace("'", "''") + "'"
    return s


def dump_items(components):
    lines = []
    for c in components:
        lines.append(f"- value: {yaml_escape(c['value'])}")
        lines.append(f"  footprint: {yaml_escape(c['footprint'])}")
        lines.append(f"  pin_count: {c['pin_count']}")
        lines.append(f"  pads:")
        for p in c["pads"]:
            lines.append(f"    - pin: {yaml_escape(p['pin'])}")
            lines.append(f"      x_mm: {p['x_mm']}")
            lines.append(f"      y_mm: {p['y_mm']}")
    return lines


items = dump_items(data["components"])



# File writing
if not os.path.exists(out_path):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("components:\n")
        f.write("\n".join(items))
        f.write("\n")
else:
    with open(out_path, "a", encoding="utf-8") as f:
        f.write(("" if open(out_path, "rb").read().endswith(b"\n") else "\n"))
        f.write("\n".join(items))
        f.write("\n")


print(f"[OK] YAML exported to:{out_path}")