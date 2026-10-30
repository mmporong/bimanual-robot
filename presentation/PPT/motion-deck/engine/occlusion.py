"""Flag small objects that a later, filled object partly covers (z-order bugs such as connectors under the next card).

usage: occlusion.py deck.pptx   -> prints each hit and 'partial occlusions: N' (aim for 0)
"""
import re
import sys
import zipfile

from lxml import etree

P = '{http://schemas.openxmlformats.org/presentationml/2006/main}'
A = '{http://schemas.openxmlformats.org/drawingml/2006/main}'
EMU = 914400
z = zipfile.ZipFile(sys.argv[1])
parts = sorted((int(m.group(1)), n) for n in z.namelist() if (m := re.fullmatch(r'ppt/slides/slide(\d+)\.xml', n)))
hits = 0
for n, part in parts:
    root = etree.fromstring(z.read(part))
    objs = []
    for el in root.find(f'{P}cSld/{P}spTree'):
        name = el.find(f'.//{P}cNvPr')
        off, ext = el.find(f'.//{A}off'), el.find(f'.//{A}ext')
        if name is None or off is None or ext is None:
            continue
        nm = name.get('name')
        x, y, w, h = (int(off.get('x')) / EMU, int(off.get('y')) / EMU, int(ext.get('cx')) / EMU, int(ext.get('cy')) / EMU)
        sppr = el.find(f'{P}spPr')
        filled = el.tag == f'{P}pic' or (sppr is not None and sppr.find(f'{A}solidFill') is not None)
        objs.append((nm, x, y, w, h, filled, el.tag.split('}')[1]))
    for i, (nm, x, y, w, h, f, t) in enumerate(objs):
        if nm.startswith('!!orb') or nm.endswith('_cover') or w * h > 1.0 or w * h < 0.005:
            continue
        for nm2, x2, y2, w2, h2, f2, t2 in objs[i + 1:]:
            # orbs sit behind everything; *_cover boxes hide a late video on purpose and fade out
            if not f2 or nm2.startswith('!!orb') or nm2.endswith('_cover') or w2 * h2 <= w * h:
                continue
            ix = max(0, min(x + w, x2 + w2) - max(x, x2))
            iy = max(0, min(y + h, y2 + h2) - max(y, y2))
            frac = ix * iy / (w * h)
            fully_inside = x >= x2 and y >= y2 and x + w <= x2 + w2 and y + h <= y2 + h2
            if 0.08 < frac and not fully_inside:
                hits += 1
                print(f'slide {n}: {nm} ({t}) {frac:.0%} covered by later {nm2} ({t2})')
print('partial occlusions:', hits)
