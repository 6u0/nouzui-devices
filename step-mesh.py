# 1/3: STEP -> raw.json (tessellated meshes + assembly tree), same format as occt-import-js.
# Uses native OpenCascade (pip install cadquery-ocp) because occt-import-js (WASM) returns empty
# meshes once the model contains the ~30k-face brain body.
# Colours and solid names come straight from the XCAF document (instance colour > part colour >
# parent colour, face colours on top), so to-glb.js no longer has to guess them by face count.
# Usage: python step-mesh.py ["source/brain device2.step"]
import sys, json
sys.stdout.reconfigure(encoding='utf-8', errors='replace')  # part names are not always cp932-safe
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.Interface import Interface_Static
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

STEP = sys.argv[1] if len(sys.argv) > 1 else 'source/brain device2.step'
# linear deflection = ratio * solid bbox size (as occt-import-js bounding_box_ratio), never below MIN_DEFL mm.
# Defaults suit the brain device; a model folder can override them in step-mesh.config.json,
# e.g. headgear/ uses a coarser floor so hundreds of tiny SMD parts stay light.
import os
_cfg = json.load(open("step-mesh.config.json")) if os.path.exists("step-mesh.config.json") else {}
RATIO, ANGLE, MIN_DEFL = _cfg.get("ratio", 0.002), _cfg.get("angle", 0.8), _cfg.get("min_deflection", 1e-4)

doc = TDocStd_Document(TCollection_ExtendedString('doc'))
Interface_Static.SetIVal_s("read.stepcaf.subshapes.name", 1)  # keep solid names (brain, shell…)
r = STEPCAFControl_Reader(); r.SetNameMode(True); r.SetColorMode(True)
r.ReadFile(STEP); r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
KINDS = (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen)

def name_of(label):
    if label.IsNull() or not label.IsAttribute(TDataStd_Name.GetID_s()): return ''
    a = TDataStd_Name()
    return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else ''

def srgb(c):  # STEP stores sRGB values; OCCT keeps them linear internally
    v = c.Values(Quantity_TypeOfColor.Quantity_TOC_sRGB)
    return [round(x, 6) for x in v]

def color_of(target):  # target: TDF_Label or TopoDS_Shape (a sub-shape of a shape in the document)
    get = ct.GetColor_s if isinstance(target, TDF_Label) else ct.GetColor
    for k in KINDS:
        c = Quantity_Color()
        if get(target, k, c): return srgb(c)
    return None

meshes = []

def mesh_solid(orig, moved, name, colour):
    b = Bnd_Box(); BRepBndLib.Add_s(moved, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    size = max(hi.X() - lo.X(), hi.Y() - lo.Y(), hi.Z() - lo.Z())
    BRepMesh_IncrementalMesh(moved, max(size * RATIO, MIN_DEFL), False, ANGLE, False)
    pos, nrm, idx, faces = [], [], [], []
    fe, fo = TopExp_Explorer(moved, TopAbs_FACE), TopExp_Explorer(orig, TopAbs_FACE)  # same order
    while fe.More():
        f = TopoDS.Face(fe.Current())
        loc = TopLoc_Location()
        T = BRep_Tool.Triangulation_s(f, loc)
        first = len(idx) // 3
        if T is not None and T.NbTriangles() > 0:
            if not T.HasNormals(): BRepLib_ToolTriangulatedShape.ComputeNormals_s(f, T)
            tr = loc.Transformation(); rev = f.Orientation() == TopAbs_REVERSED
            base = len(pos) // 3
            for i in range(1, T.NbNodes() + 1):
                p = T.Node(i).Transformed(tr); pos += [p.X(), p.Y(), p.Z()]
                n = T.Normal(i).Transformed(tr)
                s = -1 if rev else 1; nrm += [s * n.X(), s * n.Y(), s * n.Z()]
            for i in range(1, T.NbTriangles() + 1):
                a, b2, c = T.Triangle(i).Get()
                if rev: b2, c = c, b2
                idx += [base + a - 1, base + b2 - 1, base + c - 1]
        face = {'first': first, 'last': len(idx) // 3 - 1}
        fc = color_of(fo.Current())
        if fc: face['color'] = fc
        faces.append(face)
        fe.Next(); fo.Next()
    m = {'name': name, 'attributes': {'position': {'array': pos}, 'normal': {'array': nrm}},
         'index': {'array': idx}, 'brep_faces': faces}
    if colour: m['color'] = colour
    meshes.append(m)
    return len(meshes) - 1

def walk(label, loc, inherited):
    """Returns a tree node; mirrors occt-import-js: one mesh per solid, in assembly order."""
    node = {'name': name_of(label), 'meshes': [], 'children': []}
    own = color_of(label) or inherited
    if st.IsAssembly_s(label):
        comps = TDF_LabelSequence(); st.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            c = comps.Value(i); ref = TDF_Label()
            st.GetReferredShape_s(c, ref)
            # an instance colour wins over the part's own colour, which wins over the parent's
            inst = color_of(c)
            child = walk(ref, loc.Multiplied(st.GetLocation_s(c)), own)
            if inst: recolour(child, inst)
            node['children'].append(child)
        return node
    shape = st.GetShape_s(label)
    subs = TDF_LabelSequence(); st.GetSubShapes_s(label, subs)  # named solids inside a part (brain, shell…)
    subs = [(st.GetShape_s(subs.Value(i)), name_of(subs.Value(i))) for i in range(1, subs.Length() + 1)]
    eo, em = TopExp_Explorer(shape, TopAbs_SOLID), TopExp_Explorer(shape.Moved(loc), TopAbs_SOLID)
    while eo.More():
        sname = next((n for s, n in subs if s.IsSame(eo.Current()) and n), '')
        node['meshes'].append(mesh_solid(eo.Current(), em.Current(), sname or node['name'], color_of(eo.Current()) or own))
        eo.Next(); em.Next()
    return node

def recolour(node, colour):  # instance colour: replaces part-level colours, keeps face colours
    for i in node['meshes']: meshes[i]['color'] = colour
    for c in node['children']: recolour(c, colour)

roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
root = {'name': '', 'meshes': [], 'children': [walk(roots.Value(i), TopLoc_Location(), None) for i in range(1, roots.Length() + 1)]}

def show(n, d):
    print('  ' * d + (n['name'] or '(noname)') + ' meshes=' + json.dumps(n['meshes']))
    for c in n['children']: show(c, d + 1)
with open('raw.json', 'w') as fp: json.dump({'success': True, 'root': root, 'meshes': meshes}, fp)
show(root, 0)
hexc = lambda c: ''.join('%02x' % round(v * 255) for v in c) if c else '-'
for i, m in enumerate(meshes):
    p = m['attributes']['position']['array']
    mn = [min(p[a::3]) for a in range(3)] if p else [0] * 3
    mx = [max(p[a::3]) for a in range(3)] if p else [0] * 3
    print(i, m['name'], 'tris', len(m['index']['array']) // 3, 'faces', len(m['brep_faces']), 'color', hexc(m.get('color')),
          'min', ','.join('%.1f' % v for v in mn), 'max', ','.join('%.1f' % v for v in mx))
