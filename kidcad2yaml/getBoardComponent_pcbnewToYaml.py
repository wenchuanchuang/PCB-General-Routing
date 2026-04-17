# Use KiCad bundled python.exe
# Extract components (including reference) from pcbnew and export to YAML
# "ref" = unique component identifier on the PCB (e.g., R1, C5, U2)
import pcbnew, math, os

# ---- KiCad 9 specific ----

PCB_PATH = r"REPLACE_WITH_YOUR_PCB_PATH\XXX.kicad_pcb"
board = pcbnew.LoadBoard(PCB_PATH)
if not board:
    raise RuntimeError("No PCB (.kicad_pcb) loaded.")


# Output YAML path (same directory as PCB file)
pcb_path = board.GetFileName()
base_dir  = os.path.dirname(pcb_path)
yaml_dir = os.path.join(base_dir, "yaml")
if not os.path.exists(yaml_dir):
    os.makedirs(yaml_dir)
out_path = os.path.join(yaml_dir, "boardComponents.yaml")


# Deduplication set
seen = set()  # Stores unique (value, footprint) pairs to avoid duplicates

data = {"components": []}

# Iterate all footprints
update_amount = 0
for fp in board.GetFootprints():

    ref = fp.GetReference()
    value = fp.Value().GetText()

    fpid   = fp.GetFPID()
    fp_name = fpid.GetLibItemName()
    libnick = fpid.GetLibNickname()
    footprint_full = f"{libnick}:{fp_name}" if libnick else fp_name

    # Deduplication
    if ref in seen:
        continue
    seen.add(ref)
    update_amount += 1

    comp = {
        "ref": ref, 
        "value": value,   
        "footprint": footprint_full,
    }
    data["components"].append(comp)

print(f"\nUpdated {update_amount}  board components")



# ---- Simple YAML writer ----
def yaml_escape(s):
    if s is None:
        return "''"
    s = str(s)
    if any(c in s for c in [":","{","}","[","]",",","#","&","*","!","|",">","'","\"","%","@","`"]):
        return "'" + s.replace("'", "''") + "'"
    return s

# Convert component list into YAML lines (without header)
def dump_items(components):
    lines = []
    for c in components:
        lines.append(f"- ref: {yaml_escape(c['ref'])}")
        lines.append(f"  value: {yaml_escape(c['value'])}")
        lines.append(f"  footprint: {yaml_escape(c['footprint'])}")
    return lines

items = dump_items(data["components"])




# ---- Write YAML file ----
with open(out_path, "w", encoding="utf-8") as f:
        f.write("components:\n")
        f.write("\n".join(items))
        f.write("\n")

print(f"[OK] YAML exported to:{out_path}")