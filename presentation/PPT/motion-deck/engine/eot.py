"""TTF/OTF -> EOT v2.2 (uncompressed, unobfuscated) — the format PowerPoint/Canva use for ppt/fonts/*.fntdata."""
import struct, io
from fontTools.ttLib import TTFont

def _name(font, nid):
    for rec in font['name'].names:
        if rec.nameID == nid and rec.platformID == 3 and rec.platEncID in (1, 10) and rec.langID == 0x409:
            return rec.toUnicode()
    for rec in font['name'].names:
        if rec.nameID == nid and rec.platformID == 3:
            return rec.toUnicode()
    return ''

def _str(s):
    b = s.encode('utf-16le')
    return struct.pack('<HH', 0, len(b)) + b   # padding(2) + size(2) + bytes

def eot_from_bytes(data: bytes) -> bytes:
    f = TTFont(io.BytesIO(data))
    os2, head = f['OS/2'], f['head']
    p = os2.panose
    panose = bytes([p.bFamilyType, p.bSerifStyle, p.bWeight, p.bProportion, p.bContrast,
                    p.bStrokeVariation, p.bArmStyle, p.bLetterForm, p.bMidline, p.bXHeight])
    italic = 1 if (os2.fsSelection & 1) else 0
    hdr = panose + struct.pack('<BB', 1, italic)                     # charset=DEFAULT_CHARSET(1)
    hdr += struct.pack('<IHH', os2.usWeightClass, os2.fsType, 0x504C)
    hdr += struct.pack('<IIII', os2.ulUnicodeRange1, os2.ulUnicodeRange2, os2.ulUnicodeRange3, os2.ulUnicodeRange4)
    hdr += struct.pack('<II', getattr(os2, 'ulCodePageRange1', 0), getattr(os2, 'ulCodePageRange2', 0))
    hdr += struct.pack('<I', head.checkSumAdjustment) + b'\0' * 16    # Reserved1-4
    hdr += _str(_name(f, 1)) + _str(_name(f, 2)) + _str(_name(f, 5)) + _str(_name(f, 4))
    hdr += struct.pack('<HH', 0, 0)                                   # Padding5 + RootStringSize(0)
    hdr += struct.pack('<IIHH', 0x50475342, 0, 0, 0)                  # RootStringCheckSum, EUDCCodePage, Padding6, SignatureSize
    hdr += struct.pack('<II', 0, 0)                                   # EUDCFlags, EUDCFontSize
    total = 16 + len(hdr) + len(data)
    return struct.pack('<IIII', total, len(data), 0x00020002, 0) + hdr + data

if __name__ == '__main__':
    import sys
    src, dst = sys.argv[1], sys.argv[2]
    open(dst, 'wb').write(eot_from_bytes(open(src, 'rb').read()))
