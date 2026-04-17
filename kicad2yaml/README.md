## Description

This Python script is used to convert a `.kicad_pcb` file into the YAML format required by the program.

The converted YAML files are already provided in this GitHub repository.  
These Python scripts are only needed if you want to regenerate the YAML files from the original `.kicad_pcb` file.

Run the script inside KiCad:  **Tools → Scripting Console (KiPython)**

Then execute:

```python
exec(open(r"PATH\TO\SCRIPT.py", encoding="utf-8").read())
```