# Use KiCad bundled python.exe
# Export each net in a KiCad PCB (.kicad_pcb) and its pins (ref, pin) to YAML.

import sys, os
import pcbnew

PCB_PATH = r"REPLACE_WITH_YOUR_PCB_PATH\XXX.kicad_pcb"
board = pcbnew.LoadBoard(PCB_PATH)
if not board:
    raise RuntimeError("No PCB (.kicad_pcb) loaded.")



# Output file path: same directory as the .kicad_pcb file
pcb_path = board.GetFileName()
base_dir  = os.path.dirname(pcb_path)
yaml_dir  = os.path.join(base_dir, "yaml")
os.makedirs(yaml_dir, exist_ok=True)
out_path = os.path.join(yaml_dir, "boardNets.yaml")



# Helper functions

def get_pad_name(pad):
    return pad.GetName()

# --------- Simple YAML output ---------
def yaml_escape(s):
    s = "" if s is None else str(s)
    if s == "" or any(c in s for c in [":","{","}","[","]",",","#","&","*","!","|",">","'","\"","%","@","`"]):
        return "'" + s.replace("'", "''") + "'"
    return s


def get_footprint_from_pad(pad):
    try:
        fp = pcbnew.Cast_to_FOOTPRINT(pad.GetParent())
        if fp:
            return fp
    except Exception:
        pass

    parent = pad.GetParent() if hasattr(pad, "GetParent") else None
    while parent is not None:
        if hasattr(parent, "GetReference"):
            return parent
        parent = parent.GetParent() if hasattr(parent, "GetParent") else None

    return None



# === Collect pins (ref, pin) for each net ===
nets = {}     # code -> {"name": name, "code": code, "pins":[{"ref":..,"pin":..}, ...]}


# Iterate through all pads on the board (covers all footprints)
for pad in board.GetPads():

    net = pad.GetNet()
    if not net:
      # Skip pads that are not connected to any net
      continue
    
    net_code = net.GetNetCode()     
    net_name = net.GetNetname()     

    fp = get_footprint_from_pad(pad)
    if fp is None:
      continue
    
    ref = fp.GetReference()   
    pin = get_pad_name(pad)   

    if net_code not in nets:
      nets[net_code] = {"name": net_name, "code": net_code, "pins": [], "seen": set()}
    
    # Deduplication: remove duplicate pins
    key = (ref, pin)
    if key not in nets[net_code]["seen"]:
        nets[net_code]["pins"].append({"ref": ref, "pin": pin})
        nets[net_code]["seen"].add(key)



# Convert to list and sort by name, then by code
nets_list = sorted(nets.values(), key=lambda x: (x["name"], x["code"]))
# Sort pins by ref, then by pin
for n in nets_list:
    n["pins"].sort(key=lambda p: (p["ref"], p["pin"]))

print(f"\n[OK] {len(nets_list)} nets will be exported to: {out_path}")



def dump_items(nets_list):
    lines = []
    for n in nets_list:
      lines.append(f"- name: {yaml_escape(n['name'])}")
      lines.append(f"  code: {n['code']}")
      lines.append(f"  pin_count: {len(n['pins'])}")   
      lines.append(f"  pins:")
      for p in n["pins"]:
          lines.append(f"    - ref: {yaml_escape(p['ref'])}")   
          lines.append(f"      pin: {yaml_escape(p['pin'])}")   
    return lines


items = dump_items(nets_list)


# Write output file
with open(out_path, "w", encoding="utf-8") as f: 
        f.write("nets:\n")
        f.write("\n".join(items))
        f.write("\n")

print(f"[OK] YAML exported to:{out_path}")
