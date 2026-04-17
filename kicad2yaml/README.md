(This Python script is used to convert a .kicad_pcb file into a YAML format required by the program.)


Run the script inside KiCad: **Tools → Scripting Console (KiPython)**

Then execute:

```python
exec(open(r"PATH\TO\SCRIPT.py", encoding="utf-8").read())
```