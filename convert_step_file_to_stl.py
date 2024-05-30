import FreeCAD
import Part
import sys

step_file = sys.argv[1]
stl_file = f"{step_file.rsplit('.', 1)[0]}.stl"

doc = FreeCAD.open(step_file)
objs = doc.Objects
for obj in objs:
    shape = obj.Shape
    shape.exportStl(stl_file)
FreeCAD.closeDocument(doc.Name)