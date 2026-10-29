"""Post-process the pptxgenjs deck: unique shape ids, native animations (ppt-master timing writer),
Morph transitions, auto-playing muted looping videos, embedded Pretendard (EOT), Korean theme fonts.

usage: post.py deck_raw.pptx deck_raw.anim.json deck.pptx
env:   MOTION_DECK_FONTS   dir with Pretendard-{Regular,SemiBold,Bold}.ttf  (default ~/.cache/motion-deck/fonts)
       FONT_SUBSET=1       embed only the characters the deck uses (upload size limits; new text typed later falls back)
       PPT_MASTER_SCRIPTS  ppt-master skills/ppt-master/scripts dir         (default: setup.sh's pinned checkout)
"""
import json
import os
import re
import sys
import zipfile
from pathlib import Path

from lxml import etree

HERE = Path(__file__).resolve().parent
CACHE = Path(os.environ.get('MOTION_DECK_HOME', Path.home() / '.cache/motion-deck'))
PM_SCRIPTS = Path(os.environ.get('PPT_MASTER_SCRIPTS', CACHE / 'ppt-master/skills/ppt-master/scripts'))
if not (PM_SCRIPTS / 'pptx_animations.py').exists():
    raise SystemExit(f'ppt-master not found at {PM_SCRIPTS}: run scripts/setup.sh')
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PM_SCRIPTS))
from eot import eot_from_bytes  # noqa: E402
from pptx_animations import create_sequence_timing_xml  # noqa: E402
from pptx_transitions import create_transition_xml  # noqa: E402

P = 'http://schemas.openxmlformats.org/presentationml/2006/main'
A_ = 'http://schemas.openxmlformats.org/drawingml/2006/main'
R = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
NS = {'p': P, 'a': A_, 'r': R}
FONT_DIR = Path(os.environ.get('MOTION_DECK_FONTS', CACHE / 'fonts'))
FONTS = [('Pretendard', 'regular', 'Pretendard-Regular.ttf'), ('Pretendard', 'bold', 'Pretendard-Bold.ttf'),
         ('Pretendard SemiBold', 'regular', 'Pretendard-SemiBold.ttf')]


def panose_hex(path):
    from fontTools.ttLib import TTFont
    pn = TTFont(path)['OS/2'].panose
    vals = [pn.bFamilyType, pn.bSerifStyle, pn.bWeight, pn.bProportion, pn.bContrast,
            pn.bStrokeVariation, pn.bArmStyle, pn.bLetterForm, pn.bMidline, pn.bXHeight]
    return ''.join('%02X' % v for v in vals)


PANOSE = {face: panose_hex(FONT_DIR / fn) for face, slot, fn in FONTS if slot == 'regular'}

EFX = {
    'fade': ('entrance_fade', {}), 'up': ('entrance_ascend', {}), 'down': ('entrance_descend', {}),
    'zoom': ('entrance_faded_zoom', {}), 'grow': ('entrance_zoom', {}),
    # PowerPoint names a wipe by the edge it starts from (presetSubtype and filter alike: wipe(left) = From Left),
    # verified against ppt-master's PowerPoint-authored default (From Bottom = subtype 4 + wipe(down)).
    'wipeR': ('entrance_wipe', {'direction': 'left'}), 'wipeL': ('entrance_wipe', {'direction': 'right'}),
    'wipeU': ('entrance_wipe', {'direction': 'down'}), 'wipeD': ('entrance_wipe', {'direction': 'up'}),
    'path': ('path_right', {}), 'pulse': ('emphasis_grow_shrink', {}), 'out': ('exit_fade', {}),
}
MORPH_SEC = 0.9


def q(tag):
    pre, local = tag.split(':')
    return '{%s}%s' % (NS[pre], local)


def renumber_ids(root):
    """Give every drawing object a unique id (pptxgenjs reuses ids for media/tables)."""
    names = {}
    nxt = 2
    for c in root.iter(q('p:cNvPr')):
        parent = c.getparent().getparent()
        if parent.tag == q('p:grpSpPr') or c.getparent().tag == q('p:nvGrpSpPr') and parent.getparent().tag == q('p:cSld'):
            c.set('id', '1')
            continue
        c.set('id', str(nxt))
        nm = c.get('name')
        if nm in names:
            raise SystemExit(f'duplicate object name {nm}')
        names[nm] = nxt
        nxt += 1
    return names


def parse_fragment(xml):
    wrap = f'<w xmlns:p="{P}" xmlns:a="{A_}" xmlns:r="{R}">{xml}</w>'
    return etree.fromstring(wrap.encode('utf-8'))[0]


def check_starts(timing, expected, n):
    """Each effect must start at its designed absolute time (time-group delay + effect delay)."""
    got = []
    for grp in timing.find('.//p:cTn[@nodeType="mainSeq"]', NS).find('p:childTnLst', NS).findall('p:par', NS):
        for tg in grp.find('p:cTn/p:childTnLst', NS).findall('p:par', NS):
            base = int(tg.find('p:cTn/p:stCondLst/p:cond', NS).get('delay'))
            for eff in tg.find('p:cTn/p:childTnLst', NS).findall('p:par', NS):
                ctn = eff.find('p:cTn', NS)
                spid = int(ctn.find('.//p:spTgt', NS).get('spid'))
                got.append((spid, base + int(ctn.find('p:stCondLst/p:cond', NS).get('delay'))))
    if got != expected:
        raise SystemExit(f'slide {n}: effect start times differ from design: {list(zip(expected, got))[:6]}')


def build_timing(items, ids, n):
    rows = []
    prev = 0
    for it in sorted(items, key=lambda t: t['d']):
        eff, opts = EFX[it['e']]
        opts = dict(opts)
        # ppt-master reads delay_ms as the gap after the previous row's start, so pass gaps, not absolute times
        gap = int(it['d']) - prev
        prev = int(it['d'])
        tgt = {'shape_id': ids[it['name']], 'delay_ms': gap, 'effect': eff,
               'duration': float(it.get('t', 0.5)), 'trigger': 'with-previous'}
        if it['e'] == 'pulse':
            opts['size'] = float(it.get('size', 108))
            tgt['auto_reverse'] = True
        if it.get('repeat'):
            tgt['repeat_count'] = float(it['repeat'])
        if it['e'] in ('up', 'down', 'zoom', 'grow'):
            tgt['decelerate'] = 0.6
        if opts:
            tgt['effect_options'] = opts
        rows.append((it, tgt))
    xml = create_sequence_timing_xml([t for _, t in rows], duration=0.5, trigger='with-previous')
    timing = parse_fragment(xml)
    check_starts(timing, [(t['shape_id'], int(it['d'])) for it, t in rows], n)
    # custom motion-path lengths
    for it, tgt in rows:
        if it['e'] != 'path':
            continue
        sid = str(tgt['shape_id'])
        hits = [m for m in timing.iter(q('p:animMotion'))
                if m.find('.//p:spTgt', NS) is not None and m.find('.//p:spTgt', NS).get('spid') == sid]
        if len(hits) != 1:
            raise SystemExit(f'slide {n}: expected one animMotion for {it["name"]}, got {len(hits)}')
        hits[0].set('path', f'M 0 0 L {it["dx"]:.4f} {it.get("dy", 0):.4f} E')  # only the length differs from the preset
        if it.get('linear'):
            # constant speed (timers, playheads): drop the preset's 50% ease-in/ease-out
            eff = hits[0].getparent().getparent()
            for k in ('accel', 'decel'):
                eff.attrib.pop(k, None)
    return timing


def add_video(timing, vid, dur_ms, delay_ms, mute=True):
    """Auto-play (looping, muted unless mute=False) once the entrance build has run; click toggles pause."""
    root_ctn = timing.find('p:tnLst/p:par/p:cTn', NS)
    root_children = root_ctn.find('p:childTnLst', NS)
    main = root_children.find('p:seq/p:cTn[@nodeType="mainSeq"]', NS)
    first_group = main.find('p:childTnLst/p:par/p:cTn/p:childTnLst', NS)
    play = parse_fragment(
        f'<p:par><p:cTn id="0" fill="hold"><p:stCondLst><p:cond delay="{delay_ms}"/></p:stCondLst><p:childTnLst>'
        f'<p:par><p:cTn id="0" presetID="1" presetClass="mediacall" presetSubtype="0" fill="hold" nodeType="withEffect">'
        f'<p:stCondLst><p:cond delay="0"/></p:stCondLst><p:childTnLst>'
        f'<p:cmd type="call" cmd="playFrom(0.0)"><p:cBhvr><p:cTn id="0" dur="{dur_ms}" fill="hold"/>'
        f'<p:tgtEl><p:spTgt spid="{vid}"/></p:tgtEl></p:cBhvr></p:cmd>'
        f'</p:childTnLst></p:cTn></p:par></p:childTnLst></p:cTn></p:par>')
    first_group.append(play)
    toggle = parse_fragment(
        f'<p:seq concurrent="1" nextAc="seek"><p:cTn id="0" restart="whenNotActive" fill="hold" evtFilter="cancelBubble" nodeType="interactiveSeq">'
        f'<p:stCondLst><p:cond evt="onClick" delay="0"><p:tgtEl><p:spTgt spid="{vid}"/></p:tgtEl></p:cond></p:stCondLst>'
        f'<p:endSync evt="end" delay="0"><p:rtn val="all"/></p:endSync><p:childTnLst>'
        f'<p:par><p:cTn id="0" fill="hold"><p:stCondLst><p:cond delay="0"/></p:stCondLst><p:childTnLst>'
        f'<p:par><p:cTn id="0" fill="hold"><p:stCondLst><p:cond delay="0"/></p:stCondLst><p:childTnLst>'
        f'<p:par><p:cTn id="0" presetID="2" presetClass="mediacall" presetSubtype="0" fill="hold" nodeType="clickEffect">'
        f'<p:stCondLst><p:cond delay="0"/></p:stCondLst><p:childTnLst>'
        f'<p:cmd type="call" cmd="togglePause"><p:cBhvr><p:cTn id="0" dur="1" fill="hold"/>'
        f'<p:tgtEl><p:spTgt spid="{vid}"/></p:tgtEl></p:cBhvr></p:cmd>'
        f'</p:childTnLst></p:cTn></p:par></p:childTnLst></p:cTn></p:par></p:childTnLst></p:cTn></p:par>'
        f'</p:childTnLst></p:cTn><p:nextCondLst><p:cond evt="onClick" delay="0"><p:tgtEl><p:spTgt spid="{vid}"/></p:tgtEl></p:cond></p:nextCondLst></p:seq>')
    root_children.append(toggle)
    media = parse_fragment(
        f'<p:video><p:cMediaNode vol="80000"{" mute=\"1\"" if mute else ""}><p:cTn id="0" repeatCount="indefinite" fill="hold" display="0">'
        f'<p:stCondLst><p:cond delay="indefinite"/></p:stCondLst></p:cTn>'
        f'<p:tgtEl><p:spTgt spid="{vid}"/></p:tgtEl></p:cMediaNode></p:video>')
    root_children.append(media)


def round_video_frames(root, radius_in=0.14):
    """Clip videos to a rounded rectangle (PowerPoint honours the picture geometry while playing)."""
    for pic in root.iter(q('p:pic')):
        if pic.find('.//a:videoFile', NS) is None:
            continue
        ext = pic.find('p:spPr/a:xfrm/a:ext', NS)
        geom = pic.find('p:spPr/a:prstGeom', NS)
        adj = round(radius_in * 914400 / min(int(ext.get('cx')), int(ext.get('cy'))) * 100000)
        geom.set('prst', 'roundRect')
        av = geom.find('a:avLst', NS)
        if av is None:
            av = etree.SubElement(geom, q('a:avLst'))
        for g in list(av):
            av.remove(g)
        gd = etree.SubElement(av, q('a:gd')); gd.set('name', 'adj'); gd.set('fmla', f'val {adj}')


def renumber_ctn(timing):
    for i, ctn in enumerate(timing.iter(q('p:cTn')), start=1):
        ctn.set('id', str(i))


def insert_after_clrmap(root, elems):
    anchor = root.find('p:clrMapOvr', NS)
    if anchor is None:
        anchor = root.find('p:cSld', NS)
    idx = list(root).index(anchor) + 1
    for k, e in enumerate(elems):
        root.insert(idx + k, e)


def process_slide(xml, n, spec):
    root = etree.fromstring(xml)
    for old in root.findall('p:timing', NS) + root.findall('p:transition', NS):
        root.remove(old)
    ids = renumber_ids(root)
    round_video_frames(root)
    elems = []
    tr = create_transition_xml('morph', duration=MORPH_SEC) if spec['transition'] == 'morph' else create_transition_xml('fade', duration=0.7)
    elems.append(parse_fragment(tr))
    stats = {'anims': len(spec['items']), 'video': bool(spec.get('video'))}
    if spec['items']:
        missing = [it['name'] for it in spec['items'] if it['name'] not in ids]
        if missing:
            raise SystemExit(f'slide {n}: animation targets not found {missing}')
        timing = build_timing(spec['items'], ids, n)
        if spec.get('video'):
            v = spec['video']
            add_video(timing, ids[v['name']], v['durMs'], v['delay'], v.get('mute', True))
        renumber_ctn(timing)
        elems.append(timing)
    insert_after_clrmap(root, elems)
    out = etree.tostring(root, xml_declaration=True, encoding='UTF-8', standalone=True)
    # Korean charset/lang hints for East Asian text runs
    out = out.replace(b'charset="-122"', b'charset="-127"').replace(b'lang="en-US"', b'lang="ko-KR" altLang="en-US"')
    return out, stats


def used_chars(files):
    """Every character that can appear in the deck: slide/chart/notes text plus printable ASCII and common marks."""
    chars = set(chr(c) for c in range(0x20, 0x7F)) | set('·—–→←↔“”‘’…≥≤×°%↗↘✓')
    for name, data in files.items():
        if re.match(r'ppt/(slides|charts|notesSlides)/[^/]+\.xml$', name):
            for m in re.finditer(rb'<(?:a:t|c:v)>([^<]*)</(?:a:t|c:v)>', data):
                chars |= set(m.group(1).decode('utf-8'))
    return ''.join(sorted(chars))


def subset_font(data, text):
    """Keep only the glyphs the deck uses (FONT_SUBSET=1): ~2.7 MB per Pretendard weight -> ~100-200 KB."""
    import io
    from fontTools import subset
    from fontTools.ttLib import TTFont
    opts = subset.Options()
    opts.layout_features = ['*']
    opts.name_IDs = ['*']
    opts.name_languages = ['*']
    opts.name_legacy = True
    opts.notdef_outline = True
    opts.glyph_names = False
    font = TTFont(io.BytesIO(data))
    sub = subset.Subsetter(opts)
    sub.populate(text=text)
    sub.subset(font)
    out = io.BytesIO()
    font.save(out)
    return out.getvalue()


def embed_fonts(files):
    pres = files['ppt/presentation.xml'].decode('utf-8')
    text = used_chars(files) if os.environ.get('FONT_SUBSET') else None
    rels = files['ppt/_rels/presentation.xml.rels'].decode('utf-8')
    ct = files['[Content_Types].xml'].decode('utf-8')
    entries = {}
    for i, (face, slot, fn) in enumerate(FONTS, start=1):
        part = f'ppt/fonts/font{i}.fntdata'
        raw = (FONT_DIR / fn).read_bytes()
        files[part] = eot_from_bytes(subset_font(raw, text) if text else raw)
        rid = f'rIdFont{i}'
        rels = rels.replace('</Relationships>', f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/font" Target="fonts/font{i}.fntdata"/></Relationships>')
        entries.setdefault(face, []).append((slot, rid))
    if 'Extension="fntdata"' not in ct:
        ct = ct.replace('<Default Extension="xml"', '<Default Extension="fntdata" ContentType="application/x-fontdata"/><Default Extension="xml"', 1)
    lst = '<p:embeddedFontLst>'
    for face, slots in entries.items():
        lst += f'<p:embeddedFont><p:font typeface="{face}" panose="{PANOSE[face]}" pitchFamily="2" charset="-127"/>'
        for slot in ('regular', 'bold'):
            for s, rid in slots:
                if s == slot:
                    lst += f'<p:{slot} r:id="{rid}"/>'
        lst += '</p:embeddedFont>'
    lst += '</p:embeddedFontLst>'
    pres = re.sub(r'(<p:notesSz[^>]*/>)', r'\1' + lst, pres, count=1)
    pres = re.sub(r'<p:presentation ', '<p:presentation embedTrueTypeFonts="1" ', pres, count=1)
    files['ppt/presentation.xml'] = pres.encode('utf-8')
    files['ppt/_rels/presentation.xml.rels'] = rels.encode('utf-8')
    files['[Content_Types].xml'] = ct.encode('utf-8')


def theme_fonts(files):
    for name in [k for k in files if re.match(r'ppt/theme/theme\d+\.xml', k)]:
        t = files[name].decode('utf-8')
        for blk in ('majorFont', 'minorFont'):
            def fix(m, blk=blk):
                s = m.group(0)
                s = re.sub(r'<a:latin typeface="[^"]*"', '<a:latin typeface="Pretendard"', s)
                s = re.sub(r'<a:ea typeface="[^"]*"', '<a:ea typeface="Pretendard"', s)
                s = re.sub(r'<a:font script="Hang" typeface="[^"]*"/>', '', s)
                return s.replace(f'</a:{blk}>', f'<a:font script="Hang" typeface="Pretendard"/></a:{blk}>')
            t = re.sub(rf'<a:{blk}>.*?</a:{blk}>', fix, t, flags=re.S)
        files[name] = t.encode('utf-8')


def main(src, spec_path, dst):
    spec = json.loads(Path(spec_path).read_text())
    zin = zipfile.ZipFile(src)
    order = zin.namelist()
    files = {k: zin.read(k) for k in order}
    report = {}
    for n_str, sp in spec.items():
        part = f'ppt/slides/slide{n_str}.xml'
        files[part], report[int(n_str)] = process_slide(files[part], int(n_str), sp)
    embed_fonts(files)
    theme_fonts(files)
    order = ['[Content_Types].xml'] + [k for k in order if k != '[Content_Types].xml'] + [k for k in files if k not in order]
    with zipfile.ZipFile(dst, 'w', zipfile.ZIP_DEFLATED) as z:
        for k in order:
            if k.endswith('/'):
                continue
            ctype = zipfile.ZIP_STORED if k.endswith(('.mp4', '.fntdata')) else zipfile.ZIP_DEFLATED
            z.writestr(zipfile.ZipInfo(k, date_time=(1980, 1, 1, 0, 0, 0)), files[k], compress_type=ctype)
    total = sum(r['anims'] for r in report.values())
    print(f'wrote {dst}: {len(report)} slides, {total} animation rows, videos on {[k for k, r in report.items() if r["video"]]}')


if __name__ == '__main__':
    main(*sys.argv[1:4])
