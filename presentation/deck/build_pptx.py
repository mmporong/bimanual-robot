# -*- coding: utf-8 -*-
"""HOLD THE FLOW 출정식 발표자료를 PPT.pdf 템플릿 스타일로 생성한다.

스타일 특징(PPT.pdf에서 확인): 남색 헤더 바 + 흰 굵은 제목, 크림색 배경,
중앙 정렬된 검은 굵은 헤드라인 문장, 하단 "주요 인사이트" 바(남색 칩 + 연한 파랑 박스).
슬라이드 하나에 아이디어 하나만 담아 단순하게 구성한다.

사용법:
    python build_pptx.py
"""
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pathlib import Path

HERE = Path(__file__).resolve().parent
MEDIA = HERE.parent / "media"
OUT = HERE / "HOLD_THE_FLOW_출정식.pptx"

FONT = "맑은 고딕"

def rgb(h):
    return RGBColor.from_string(h)

NAVY = rgb("1A2E66")
NAVY_LIGHT = rgb("3B54A0")
CREAM = rgb("F2EFE7")
WHITE = rgb("FFFFFF")
ORANGE = rgb("DD8341")
INK = rgb("1C1C1C")
GRAY = rgb("6E6E6E")
LIGHT_BLUE = rgb("DCE6F6")
BORDER = rgb("D9D4C6")
GOOD = rgb("2E8B57")
GOOD_SOFT = rgb("E1F0E6")
AMBER = rgb("B9721A")
AMBER_SOFT = rgb("F6E9D3")
SIMB = rgb("2F6FB0")
SIMB_SOFT = rgb("DCE9F6")
TODOB = rgb("7A7A7A")
TODOB_SOFT = rgb("EAEAEA")

SLIDE_W_IN = 13.333
SLIDE_H_IN = 7.5

prs = Presentation()
prs.slide_width = Inches(SLIDE_W_IN)
prs.slide_height = Inches(SLIDE_H_IN)
BLANK = prs.slide_layouts[6]


def no_line(shape):
    shape.line.fill.background()
    shape.shadow.inherit = False


def set_text(tf, text, size=12, bold=False, color=INK, align=PP_ALIGN.LEFT, bullet=False, font=FONT):
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.alignment = align
    run = p.add_run()
    run.text = ("• " + text) if bullet else text
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = color
    run.font.name = font
    return p


def add_para(tf, text, size=12, bold=False, color=INK, align=PP_ALIGN.LEFT, bullet=False, font=FONT):
    p = tf.add_paragraph()
    p.alignment = align
    run = p.add_run()
    run.text = ("• " + text) if bullet else text
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = color
    run.font.name = font
    return p


def new_slide():
    slide = prs.slides.add_slide(BLANK)
    bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg.fill.solid()
    bg.fill.fore_color.rgb = CREAM
    no_line(bg)
    return slide


def add_header(slide, title, eyebrow=None):
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, Inches(1.05))
    bar.fill.solid()
    bar.fill.fore_color.rgb = NAVY
    no_line(bar)
    dots = [(11.55, 0.20, 0.16), (12.15, 0.42, 0.10), (12.55, 0.16, 0.08), (12.85, 0.55, 0.06), (11.95, 0.72, 0.07)]
    for x, y, s in dots:
        c = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(s), Inches(s))
        c.fill.solid()
        c.fill.fore_color.rgb = NAVY_LIGHT
        no_line(c)
    tb = slide.shapes.add_textbox(Inches(0.55), Inches(0.10), Inches(10.6), Inches(0.85))
    tf = tb.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.word_wrap = True
    if eyebrow:
        set_text(tf, eyebrow, size=11, bold=True, color=rgb("9FB2E0"))
        add_para(tf, title, size=24, bold=True, color=WHITE)
    else:
        set_text(tf, title, size=25, bold=True, color=WHITE)
    return bar


def add_headline(slide, text, top=1.28, size=20, height=0.85):
    tb = slide.shapes.add_textbox(Inches(0.6), Inches(top), Inches(SLIDE_W_IN - 1.2), Inches(height))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    set_text(tf, text, size=size, bold=True, color=INK, align=PP_ALIGN.CENTER)
    return tb


def add_insight(slide, label, text, top=6.42, height=0.78):
    left = Inches(0.55)
    total_w = SLIDE_W_IN - 1.1
    chip_w = 1.55
    chip = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, Inches(top), Inches(chip_w), Inches(height))
    chip.fill.solid()
    chip.fill.fore_color.rgb = NAVY
    no_line(chip)
    tf = chip.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    set_text(tf, label, size=12.5, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
    box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.55 + chip_w), Inches(top), Inches(total_w - chip_w), Inches(height))
    box.fill.solid()
    box.fill.fore_color.rgb = LIGHT_BLUE
    no_line(box)
    tf2 = box.text_frame
    tf2.word_wrap = True
    tf2.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf2.margin_left = Inches(0.22)
    tf2.margin_right = Inches(0.22)
    if isinstance(text, list):
        set_text(tf2, text[0], size=11.5, color=INK)
        for t in text[1:]:
            add_para(tf2, t, size=11.5, color=INK)
    else:
        set_text(tf2, text, size=11.5, color=INK)


def add_page_number(slide, n):
    tb = slide.shapes.add_textbox(Inches(SLIDE_W_IN / 2 - 0.4), Inches(SLIDE_H_IN - 0.34), Inches(0.8), Inches(0.28))
    tf = tb.text_frame
    set_text(tf, str(n), size=10, color=GRAY, align=PP_ALIGN.CENTER)


BADGES = {
    "real": (GOOD, GOOD_SOFT, "실차·실물"),
    "sim": (SIMB, SIMB_SOFT, "시뮬"),
    "code": (TODOB, TODOB_SOFT, "코드·단위시험"),
    "plan": (AMBER, AMBER_SOFT, "계획"),
}


def add_badge(slide, x, y, kind, custom_text=None):
    color, soft, default_text = BADGES[kind]
    text = custom_text or default_text
    w = 0.28 + 0.098 * len(text)
    box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(0.32))
    box.fill.solid()
    box.fill.fore_color.rgb = soft
    box.line.color.rgb = color
    box.line.width = Pt(0.75)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.margin_left = Inches(0.05)
    tf.margin_right = Inches(0.05)
    set_text(tf, text, size=9.5, bold=True, color=color, align=PP_ALIGN.CENTER)
    return w


# ---------------------------------------------------------------- flow (pipeline)

def add_flow(slide, steps, top=2.5, height=1.05, box_font=12):
    n = len(steps)
    margin = 0.6
    arrow_w = 0.34
    total_w = SLIDE_W_IN - margin * 2
    box_w = (total_w - arrow_w * (n - 1)) / n
    x = margin
    for i, label in enumerate(steps):
        box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(top), Inches(box_w), Inches(height))
        box.fill.solid()
        box.fill.fore_color.rgb = NAVY
        no_line(box)
        tf = box.text_frame
        tf.word_wrap = True
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf.margin_left = Inches(0.06)
        tf.margin_right = Inches(0.06)
        set_text(tf, f"{i + 1:02d}", size=9.5, color=rgb("9FB2E0"), align=PP_ALIGN.CENTER)
        add_para(tf, label, size=box_font, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
        x += box_w
        if i < n - 1:
            ar = slide.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x), Inches(top + height / 2 - 0.11), Inches(arrow_w), Inches(0.22))
            ar.fill.solid()
            ar.fill.fore_color.rgb = ORANGE
            no_line(ar)
            x += arrow_w


def add_flow_labeled(slide, steps, top=2.5, height=1.05):
    """steps: list of (top_label, main_label, navy_bool)"""
    n = len(steps)
    margin = 0.6
    arrow_w = 0.34
    total_w = SLIDE_W_IN - margin * 2
    box_w = (total_w - arrow_w * (n - 1)) / n
    x = margin
    for i, (tag, label, is_robot) in enumerate(steps):
        fill = NAVY if is_robot else rgb("C97A2E")
        box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(top), Inches(box_w), Inches(height))
        box.fill.solid()
        box.fill.fore_color.rgb = fill
        no_line(box)
        tf = box.text_frame
        tf.word_wrap = True
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf.margin_left = Inches(0.05)
        tf.margin_right = Inches(0.05)
        set_text(tf, tag, size=9.5, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
        add_para(tf, label, size=11.5, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
        x += box_w
        if i < n - 1:
            ar = slide.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x), Inches(top + height / 2 - 0.11), Inches(arrow_w), Inches(0.22))
            ar.fill.solid()
            ar.fill.fore_color.rgb = BORDER
            no_line(ar)
            x += arrow_w


# ---------------------------------------------------------------- numbered columns

def add_columns(slide, items, top=2.35, bottom=6.25):
    n = len(items)
    margin = 0.6
    total_w = SLIDE_W_IN - margin * 2
    col_w = total_w / n
    for i, item in enumerate(items):
        title = item["title"]
        desc = item["desc"]
        cx = margin + i * col_w
        icon = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(cx + col_w / 2 - 0.32), Inches(top), Inches(0.64), Inches(0.64))
        icon.fill.solid()
        icon.fill.fore_color.rgb = ORANGE
        no_line(icon)
        tf = icon.text_frame
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        set_text(tf, str(i + 1), size=20, bold=True, color=WHITE, align=PP_ALIGN.CENTER)
        tb = slide.shapes.add_textbox(Inches(cx + 0.12), Inches(top + 0.82), Inches(col_w - 0.24), Inches(0.55))
        tf2 = tb.text_frame
        tf2.word_wrap = True
        set_text(tf2, title, size=14, bold=True, color=NAVY, align=PP_ALIGN.CENTER)
        tb2 = slide.shapes.add_textbox(Inches(cx + 0.12), Inches(top + 1.35), Inches(col_w - 0.24), Inches(bottom - top - 1.4))
        tf3 = tb2.text_frame
        tf3.word_wrap = True
        set_text(tf3, desc, size=11.5, color=INK, align=PP_ALIGN.CENTER)
        if i > 0:
            ln = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(cx), Inches(top - 0.05), Inches(cx), Inches(bottom))
            ln.line.color.rgb = BORDER
            ln.line.width = Pt(1)


# ---------------------------------------------------------------- card grid

def add_cards_grid(slide, cards, cols=2, top=2.15, bottom=6.3, gap=0.22):
    n = len(cards)
    rows = (n + cols - 1) // cols
    margin = 0.6
    total_w = SLIDE_W_IN - margin * 2
    card_w = (total_w - gap * (cols - 1)) / cols
    total_h = bottom - top
    card_h = (total_h - gap * (rows - 1)) / rows
    for idx, card in enumerate(cards):
        r = idx // cols
        c = idx % cols
        x = margin + c * (card_w + gap)
        y = top + r * (card_h + gap)
        accent = card.get("accent", NAVY)
        box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(x), Inches(y), Inches(card_w), Inches(card_h))
        box.fill.solid()
        box.fill.fore_color.rgb = WHITE
        box.line.color.rgb = BORDER
        box.line.width = Pt(1)
        box.shadow.inherit = False
        accbar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(0.07), Inches(card_h))
        accbar.fill.solid()
        accbar.fill.fore_color.rgb = accent
        no_line(accbar)
        tf = box.text_frame
        tf.word_wrap = True
        tf.margin_left = Inches(0.22)
        tf.margin_right = Inches(0.18)
        tf.margin_top = Inches(0.14)
        first = True
        if card.get("kicker"):
            set_text(tf, card["kicker"], size=10.5, bold=True, color=ORANGE)
            first = False
        title_p = (set_text if first else add_para)(tf, card["title"], size=14.5, bold=True, color=NAVY)
        if isinstance(card["desc"], list):
            for d in card["desc"]:
                add_para(tf, d, size=11, color=INK, bullet=True)
        else:
            add_para(tf, card["desc"], size=11.5, color=INK)
        if card.get("badge"):
            bx = x + card_w - 1.55
            by = y + 0.14
            add_badge(slide, bx, by, card["badge"])


# ---------------------------------------------------------------- table

def add_table(slide, headers, rows, top=2.2, left=0.6, width=None, height=None, highlight_col=None, col_widths=None, font_size=11):
    width = width if width else (SLIDE_W_IN - 1.2)
    nrows = len(rows) + 1
    ncols = len(headers)
    height = height if height else min(0.52 * nrows, 4.4)
    shape = slide.shapes.add_table(nrows, ncols, Inches(left), Inches(top), Inches(width), Inches(height))
    table = shape.table
    if col_widths:
        for i, w in enumerate(col_widths):
            table.columns[i].width = Inches(w)
    for j, h in enumerate(headers):
        cell = table.cell(0, j)
        cell.text = ""
        cell.fill.solid()
        cell.fill.fore_color.rgb = NAVY
        cell.margin_left = Inches(0.1)
        cell.margin_right = Inches(0.1)
        cell.vertical_anchor = MSO_ANCHOR.MIDDLE
        p = cell.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        run = p.add_run()
        run.text = h
        run.font.bold = True
        run.font.size = Pt(font_size + 0.5)
        run.font.color.rgb = WHITE
        run.font.name = FONT
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            cell = table.cell(i + 1, j)
            cell.text = ""
            is_hl = highlight_col is not None and j == highlight_col
            cell.fill.solid()
            cell.fill.fore_color.rgb = LIGHT_BLUE if is_hl else WHITE
            cell.margin_left = Inches(0.1)
            cell.margin_right = Inches(0.1)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            p = cell.text_frame.paragraphs[0]
            run = p.add_run()
            run.text = str(val)
            run.font.size = Pt(font_size)
            run.font.bold = is_hl
            run.font.color.rgb = INK
            run.font.name = FONT
    return table


# ---------------------------------------------------------------- bullet box (plain text block)

def add_bullet_box(slide, x, y, w, h, title, bullets, title_color=NAVY, title_size=14, bullet_size=12.5):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    if title:
        set_text(tf, title, size=title_size, bold=True, color=title_color)
        first = False
    else:
        first = True
    for b in bullets:
        (set_text if first else add_para)(tf, b, size=bullet_size, color=INK, bullet=True)
        first = False
    return tb


def add_picture_fit(slide, path, x, y, max_w, max_h):
    from PIL import Image
    with Image.open(path) as im:
        iw, ih = im.size
    ratio = min(max_w / iw, max_h / ih)
    w, h = iw * ratio, ih * ratio
    px = x + (max_w - w) / 2
    py = y + (max_h - h) / 2
    slide.shapes.add_picture(str(path), Inches(px), Inches(py), Inches(w), Inches(h))


def add_movie(slide, path, x, y, w, h, poster_frame_image=None):
    """영상을 슬라이드에 임베드한다. poster_frame_image가 주어지면 그 이미지의
    가로세로비에 맞춰 (x,y,w,h)를 최대 경계 상자로 취급해 add_picture_fit과
    같은 방식으로 종횡비를 유지한다. 파일이 없으면 조용히 스킵한다."""
    path = Path(path)
    if not path.exists():
        print(f"WARNING: missing media {path}, 영상 스킵")
        return None
    px, py, pw, ph = x, y, w, h
    poster = Path(poster_frame_image) if poster_frame_image else None
    if poster and poster.exists():
        from PIL import Image
        with Image.open(poster) as im:
            iw, ih = im.size
        ratio = min(w / iw, h / ih)
        pw, ph = iw * ratio, ih * ratio
        px, py = x + (w - pw) / 2, y + (h - ph) / 2
    kwargs = {"mime_type": "video/mp4"}
    if poster and poster.exists():
        kwargs["poster_frame_image"] = str(poster)
    movie = slide.shapes.add_movie(str(path), Inches(px), Inches(py), Inches(pw), Inches(ph), **kwargs)
    return movie


def add_quote_block(slide, quote, note, source=None, top=2.3, height=2.7):
    box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.1), Inches(top), Inches(SLIDE_W_IN - 2.2), Inches(height))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    accbar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.1), Inches(top), Inches(0.09), Inches(height))
    accbar.fill.solid()
    accbar.fill.fore_color.rgb = ORANGE
    no_line(accbar)
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.margin_left = Inches(0.45)
    tf.margin_right = Inches(0.45)
    set_text(tf, quote, size=19, bold=True, color=NAVY, align=PP_ALIGN.CENTER)
    if source:
        add_para(tf, source, size=11, color=GRAY, align=PP_ALIGN.CENTER)
    add_para(tf, " ", size=6, align=PP_ALIGN.CENTER)
    add_para(tf, note, size=13, color=INK, align=PP_ALIGN.CENTER)


# =====================================================================
# 슬라이드 빌드
# =====================================================================

def build_title():
    slide = prs.slides.add_slide(BLANK)
    bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bg.fill.solid()
    bg.fill.fore_color.rgb = WHITE
    no_line(bg)

    tb = slide.shapes.add_textbox(Inches(1.5), Inches(1.55), Inches(2.5), Inches(0.5))
    set_text(tb.text_frame, "2026", size=22, bold=True, color=INK)

    rule = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.5), Inches(2.05), Inches(10.3), Pt(2.4))
    rule.fill.solid()
    rule.fill.fore_color.rgb = INK
    no_line(rule)

    icon_path = MEDIA / "01_완성품_콘셉트_ImageGen.png"
    if icon_path.exists():
        add_picture_fit(slide, icon_path, 1.5, 2.35, 1.15, 1.15)

    tb2 = slide.shapes.add_textbox(Inches(2.9), Inches(2.35), Inches(9.0), Inches(1.65))
    tf2 = tb2.text_frame
    tf2.word_wrap = True
    tf2.vertical_anchor = MSO_ANCHOR.MIDDLE
    set_text(tf2, "HOLD THE FLOW", size=46, bold=True, color=NAVY)
    add_para(tf2, "이동형 양팔 물 서빙 로봇 · 프로젝트 출정식", size=20, bold=True, color=NAVY)

    rule2 = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.5), Inches(4.15), Inches(10.3), Pt(2.4))
    rule2.fill.solid()
    rule2.fill.fore_color.rgb = INK
    no_line(rule2)

    tb3 = slide.shapes.add_textbox(Inches(1.5), Inches(4.32), Inches(10.3), Inches(0.5))
    set_text(tb3.text_frame, "앞으로 5주의 실행 계획", size=16, color=GRAY, align=PP_ALIGN.CENTER)

    tb4 = slide.shapes.add_textbox(Inches(1.5), Inches(6.75), Inches(6.0), Inches(0.4))
    set_text(tb4.text_frame, "github.com/mmporong/bimanual-robot", size=12, color=GRAY)

    tb5 = slide.shapes.add_textbox(Inches(8.5), Inches(6.75), Inches(3.3), Inches(0.4))
    set_text(tb5.text_frame, "2026.10.02(금) · bimanual-robot 팀", size=12, color=GRAY, align=PP_ALIGN.RIGHT)


def build_s02():
    s = new_slide()
    add_header(s, "개요 — 미션 파이프라인", eyebrow="01 · 개요")
    add_headline(s, "물 한 잔을, 준비부터 테이블 위까지")
    add_flow(s, ["웹 주문", "주방 이동", "작업대 정렬", "양팔 물 따르기", "상판 적재 후 이동", "테이블에 컵 배치", "충전소 복귀·충전"], top=2.6, height=1.15, box_font=10.5)
    add_insight(s, "미션", "충전소에서 대기하던 로봇이 웹 요청을 받아 주방으로 이동하고, 왼팔 컵·오른팔 물통으로 물을 따른 뒤 테이블에 배치하고 충전소로 복귀한다.")
    add_page_number(s, len(prs.slides))


def build_s03():
    s = new_slide()
    add_header(s, "개요 — 운용 원칙", eyebrow="01 · 개요")
    add_headline(s, "세 가지 원칙으로 미션을 통제한다")
    add_cards_grid(s, [
        {"kicker": "운반 원칙", "title": "컵은 상판 위에", "desc": "주행 중에는 컵을 팔로 들지 않는다."},
        {"kicker": "정지 원칙", "title": "조작 중 베이스 정지", "desc": "팔이 움직일 때 바퀴는 정지한다."},
        {"kicker": "판정 원칙", "title": "기록이 곧 성공", "desc": "성공은 버튼 응답이 아니라, 한 주문의 전 과정이 기록으로 이어지는 것이다."},
    ], cols=3, top=2.4, bottom=6.25)
    add_insight(s, "왜 중요한가", "세 원칙은 이동·조작 역할 분리와 실패 원인 추적의 전제가 된다.")
    add_page_number(s, len(prs.slides))


def build_s04():
    s = new_slide()
    add_header(s, "기술 개발 배경[1] — 기술적 필요성", eyebrow="02 · 배경")
    add_headline(s, "서빙로봇은 '나르기'까지 — 마지막 1 m는 사람 몫")
    add_quote_block(
        s,
        quote="“로봇이 자리에 도착하면 음식을 테이블로 내려야 한다”",
        source="외부 E2-1 · ZDNet Korea, 2023.2",
        note="로봇이 테이블에 도착해도 내려놓는 동작은 사람이 한다. 준비부터 테이블 위 전달까지 가능한 양팔 조작 기술이 다음 단계다.",
    )
    add_insight(s, "기술적 필요성", "마지막 1 m를 채우는 양팔 조작 기술이 필요하다.")
    add_page_number(s, len(prs.slides))


def build_s05():
    s = new_slide()
    add_header(s, "기술 개발 배경[2] — 시장적 필요성", eyebrow="02 · 배경")
    add_headline(s, "보급은 느는데, 활용 범위는 그대로")
    add_quote_block(
        s,
        quote="“한 대가 한 가지 일만 하는 구조로 비용 상쇄 어려움”",
        source="업계가 꼽은 확산 걸림돌 · 외부 E2-2 · 서울경제, 2026.9.21",
        note="국내 서빙로봇 보급은 2021년 3,000대 → 2023년 11,000대(예상)로 늘었다 — 외부 E2-3 · 식품외식경제, 2023.7",
    )
    add_insight(s, "시장적 필요성", "보급 확대의 다음 과제는 '한 대가 맡는 일의 범위'다.")
    add_page_number(s, len(prs.slides))


def build_s06():
    s = new_slide()
    add_header(s, "기술 개발 배경[3] — 사회적 필요성", eyebrow="02 · 배경")
    add_headline(s, "일할 사람은 줄고, 대우는 낮다")
    add_cards_grid(s, [
        {"kicker": "외식업 임금 수준", "title": "전 산업 평균 대비 58%", "desc": "외부 E3-1 · 한국농촌경제연구원 인용, 2025"},
        {"kicker": "핵심 노동 연령층 전망", "title": "2033년까지 약 182만 명 감소", "desc": "30~59세 인구 기준 · 외부 E3-2 · 한국농촌경제연구원 인용, 2025"},
    ], cols=2, top=2.5, bottom=6.25)
    add_insight(s, "사회적 필요성", "인구 구조 변화가 자동화 수요의 근본 배경이다.")
    add_page_number(s, len(prs.slides))


def build_s07():
    s = new_slide()
    add_header(s, "기술 개발 배경[4] — 정책적 필요성", eyebrow="02 · 배경")
    add_headline(s, "업계가 1순위로 요청한 것도 자동화")
    add_quote_block(
        s,
        quote="자동화·로봇 기술 지원 68.8%",
        source="외식업체가 1순위로 요청한 정책 · 외부 E4-1 · 한국농촌경제연구원 인용, 2025",
        note="규제가 아니라 현장의 요청이 먼저다.",
    )
    add_insight(s, "정직한 전제", "2025년 3분기 미충원율은 8.4%로 오히려 낮아졌다 — 근거는 단기 구인난이 아니라 인구 구조와 '한 대 한 기능' 구조의 활용도 문제다.")
    add_page_number(s, len(prs.slides))


def build_s08():
    s = new_slide()
    add_header(s, "핵심 문제", eyebrow="03 · 문제와 솔루션")
    add_headline(s, "한 대가 한 가지 일만 한다")
    add_quote_block(
        s,
        quote="준비와 테이블 위 전달은 여전히 사람 몫이다",
        note="로봇 한 대의 활용도가 낮고, 비용을 상쇄하기 어렵다.",
        source=None,
    )
    add_insight(s, "문제", "범용 로봇이 아니라, 이 한 가지 구조적 공백을 메우는 것이 출발점이다.")
    add_page_number(s, len(prs.slides))


def build_s09():
    s = new_slide()
    add_header(s, "솔루션", eyebrow="03 · 문제와 솔루션")
    add_headline(s, "양팔과 이동을 한 미션으로 묶는다")
    add_flow(s, [
        "준비: 왼팔 컵·오른팔 물통 붓기",
        "운반: 컵을 상판에 싣고 자율주행",
        "전달: 재파지 후 테이블 배치",
        "복귀: 충전소 복귀·충전 확인",
    ], top=2.7, height=1.35, box_font=11.5)
    add_insight(s, "아직 가설", "여러 업무를 한 대에 묶으면 활용도가 오른다는 것은 가설이다 — 5주 뒤 통합 리허설(G6)의 end-to-end 기록으로 확인한다.")
    add_page_number(s, len(prs.slides))


def build_s10():
    s = new_slide()
    add_header(s, "환경 분석[1] — 시중 서빙로봇 대비", eyebrow="04 · 환경 분석")
    add_headline(s, "시중 서빙로봇과 무엇이 다른가", size=19)
    add_table(s, ["업무", "일반 서빙로봇", "HOLD THE FLOW"], [
        ["주문 연결", "직원이 음식을 올리고 목적지 입력", "웹 주문이 미션으로 바로 연결"],
        ["음료 준비", "사람", "양팔 물 따르기"],
        ["운반", "트레이 운반", "상판 적재 운반"],
        ["테이블 위 전달", "손님·직원이 내려놓음", "컵을 다시 잡아 테이블에 배치"],
        ["복귀", "대기 위치 복귀", "충전소 복귀·충전 시작 확인"],
        ["기록", "주행 로그", "주문 단위 phase 기록·실패 원인 기록"],
    ], top=2.15, height=3.9, col_widths=[2.3, 4.3, 5.1], highlight_col=2, font_size=11.5)
    add_insight(s, "정직한 긴장 관계", "팔을 더하면 가격·복잡도도 늘어난다 — 기능 대비 비용은 오늘 멘토링에서 여쭙고 싶은 질문이다. (외부 E2-1·E2-2)")
    add_page_number(s, len(prs.slides))


def build_s11():
    s = new_slide()
    add_header(s, "환경 분석[2] — 오픈소스 프로젝트 대비", eyebrow="04 · 환경 분석")
    add_headline(s, "정책을 만드는 도구 위에, 서비스를 운영하는 계층을", size=18)
    add_table(s, ["프로젝트", "구성·비용", "공개 범위", "공개 자료에 없는 부분"], [
        ["ALOHA", "탁상 고정 양팔, $20k 미만", "저가 양팔 텔레옵, ACT 정책", "이동, 미션 운영"],
        ["Mobile ALOHA", "이동 양팔, $32k", "전신 텔레옵, 과제당 시연 50회", "단일 과제 모방학습, 고정 팔 높이"],
        ["XLeRobot", "SO-101×2 + 이동 베이스, 약 $660", "텔레옵, MuJoCo, RL·VLA 학습", "주문 연결, 충전 복귀, 실패 기록, Nav2 연동"],
        ["LeRobot bi-SO101", "SO-101 양팔", "양팔 시연 기록·정책 학습 프레임워크", "이동 베이스, 미션 운영"],
    ], top=2.15, height=3.2, col_widths=[2.0, 2.5, 3.5, 3.5], font_size=10.5)
    add_insight(s, "포지셔닝", "우리는 XLeRobot과 같은 SO-101 저가 양팔 계열 위에 서비스 미션 계층을 더하는 위치다.")
    add_page_number(s, len(prs.slides))


def build_s12():
    s = new_slide()
    add_header(s, "개발 창의성 및 혁신성", eyebrow="05 · 창의성·혁신성")
    add_headline(s, "우리가 더하는 것 네 가지")
    add_cards_grid(s, [
        {"kicker": "01", "title": "이동·조작 역할 분리", "desc": "주행 중에는 상판 운반, 조작 중에는 베이스 정지 — 같은 로봇이 두 역할을 겹치지 않고 수행"},
        {"kicker": "02", "title": "주문 단위 기록", "desc": "웹 주문 → phase별 결과 → 도크 복귀까지 하나의 request_id/run_id로 연결"},
        {"kicker": "03", "title": "phase별 공정 비교", "desc": "같은 조건에서 계획 기반(PLANNED)과 학습 정책(ACT)을 phase 단위로 비교, 더 나은 쪽만 채택(HYBRID)"},
        {"kicker": "04", "title": "실패 데이터 규약", "desc": "증상 → 원인 가설·기각 근거 → 확인된 원인 → 같은 조건 재시험 → 학습 데이터 사용 결정"},
    ], cols=2, top=2.15, bottom=6.3)
    add_insight(s, "근거", "산업용 피킹 조사 결과, 상용 제품은 AI를 인식·그립 판단에 쓰고 움직임은 검증된 궤적과 결합한다 — R33.")
    add_page_number(s, len(prs.slides))


def build_s13():
    s = new_slide()
    add_header(s, "시뮬레이션 데모", eyebrow="06 · 데모")
    add_headline(s, "그래서, 실제로 되나요?")
    poster = MEDIA / "05_시뮬_서빙시연_마지막장면.png"
    add_movie(s, MEDIA / "05_시뮬_서빙시연_59s.mp4", 0.6, 2.25, 6.1, 3.45, poster_frame_image=poster)
    tb_order = s.shapes.add_textbox(Inches(0.6), Inches(5.75), Inches(6.1), Inches(0.55))
    set_text(tb_order.text_frame, "영상 순서 1 컵·병 파지 → 2 물따르기 → 3 상판 적재 → 4 이동 → 5 컵 재파지 → 6 테이블 배치", size=10, color=GRAY)
    bx, by, bw = 7.0, 2.3, 5.7
    add_badge(s, bx, by, "sim", custom_text="시뮬 · Isaac Sim 5.1 · 1회")
    add_table(s, ["항목", "값"], [
        ["완주 시간", "473.9초 (도크 복귀까지)"],
        ["최종 물 입자", "컵 687 · 병 227 · 외부 4"],
        ["운반 중 추가 손실", "0"],
        ["phase 통과", "12/12 (dock_roundtrip02)"],
    ], top=by + 0.5, left=bx, width=bw, height=2.3, col_widths=[1.9, 3.8], font_size=11.5)
    tb = s.shapes.add_textbox(Inches(bx), Inches(by + 2.95), Inches(bw), Inches(0.9))
    set_text(tb.text_frame, "왼손은 이전 후보(80mm 강체 패드) 형상 가정 — 현행 순정 그리퍼 재실행은 로드맵 G5.", size=10.5, color=GRAY)
    add_insight(s, "확인 범위", "Isaac Sim 5.1 시뮬레이션 1회 결과다 (구간별 2~8배속, 15컷 편집본, 배경음악 생성).")
    add_page_number(s, len(prs.slides))


def build_s14():
    s = new_slide()
    add_header(s, "시스템 구조", eyebrow="07 · 시스템 구조")
    add_headline(s, "인지는 판단만, 팔 명령은 한 통로로")
    img = MEDIA / "03_시스템구조_도식.png"
    if img.exists():
        add_picture_fit(s, img, 0.6, 2.25, 6.3, 3.9)
    bx, by, bw = 7.15, 2.35, 5.6
    tb = s.shapes.add_textbox(Inches(bx), Inches(by), Inches(bw), Inches(1.1))
    set_text(tb.text_frame, "카메라·인식 모델은 판단만 하고 모터를 움직이지 않는다. 계획 기반·학습 정책·텔레옵 중 한 번에 하나만 팔 명령권(lease)을 갖는다.", size=12.5, color=INK)
    add_table(s, ["위치", "맡는 일"], [
        ["로봇 Raspberry Pi 4B", "센서·SLAM/AMCL·Nav2·미션 상태기계·계획 기반 조작·명령 중재·watchdog"],
        ["외부 노트북 RTX 5050 8GB", "웹 UI·학습 정책 추론·YOLO·학습"],
        ["Wi-Fi", "노트북은 서보 포트를 열지 않고, 명령은 Pi를 거친다"],
    ], top=by + 1.25, left=bx, width=bw, height=2.0, col_widths=[2.1, 3.5], font_size=10.5)
    add_insight(s, "설계 원칙", "모든 명령은 같은 중재기와 안전 게이트를 지난다 — lease 전환(PLANNED↔ACT↔TELEOP) 시험은 G6에서 진행한다.")
    add_page_number(s, len(prs.slides))


def build_s15():
    s = new_slide()
    add_header(s, "조작 전략 — 세 가지 방식", eyebrow="08 · 조작 전략")
    add_headline(s, "phase마다 계획 기반과 학습 정책을 같은 조건에서 비교")
    add_cards_grid(s, [
        {"kicker": "PLANNED_ALL", "title": "산업형 기준선", "desc": "RGB-D·그립 계산·IK·검증 궤적·그리퍼 피드백·액면 폐루프"},
        {"kicker": "ACT_ALL", "title": "전체 IL 가능성 측정", "desc": "모든 조작 phase를 학습 정책(ACT checkpoint)으로 실행"},
        {"kicker": "HYBRID", "title": "실이점 구간에만 학습", "desc": "phase별로 더 나은 방식을 선택 — 첫 후보는 POUR만 ACT"},
    ], cols=3, top=2.4, bottom=6.25)
    add_insight(s, "첫 후보", "파지·접근·반환·배치는 PLANNED, POUR(붓기)만 ACT로 실행한다.")
    add_page_number(s, len(prs.slides))


def build_s16():
    s = new_slide()
    add_header(s, "조작 전략 — phase별 적용", eyebrow="08 · 조작 전략")
    add_headline(s, "여섯 phase 중 지금은 POUR만 학습 정책 후보")
    add_flow_labeled(s, [
        ("01", "CUP_PICK · PLANNED", True),
        ("02", "JUG_PICK · PLANNED", True),
        ("03", "PREPOUR · PLANNED", True),
        ("04", "POUR · ACT", False),
        ("05", "JUG_RETURN · PLANNED", True),
        ("06", "SHELF_PLACE · PLANNED", True),
    ], top=2.7, height=1.3)
    add_movie(s, MEDIA / "06_시뮬_물붓기확대_30s.mp4", 4.67, 4.25, 4.0, 2.05,
              poster_frame_image=MEDIA / "06_시뮬_물붓기확대_포스터.png")
    add_insight(s, "범례", "남색 = PLANNED(계획 기반) · 주황 = ACT(학습 정책) — 위 영상은 POUR 구간 확대(Isaac Sim 5.1, 왼손 이전 후보 그리퍼 가정)")
    add_page_number(s, len(prs.slides))


def build_s17():
    s = new_slide()
    add_header(s, "기술개발 목표[1] — 핵심 정량 지표", eyebrow="09 · 목표")
    add_headline(s, "무엇을 셀 것인가")
    box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.4), Inches(2.3), Inches(SLIDE_W_IN - 2.8), Inches(3.9))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.5)
    tf.margin_top = Inches(0.3)
    items = [
        "phase별 성공 횟수 / 전체 횟수",
        "목표 물 양 오차(mL), UNDER/OVER/SPILL/UNKNOWN 비율",
        "사람 개입·복구 횟수",
        "1회 소요 시간(cycle time)",
        "관측→명령 지연 p95, 제어 지터",
        "위치·조명·초기 잔량 변화에서의 결과",
    ]
    first = True
    for it in items:
        (set_text if first else add_para)(tf, it, size=15, color=INK, bullet=True)
        first = False
    add_insight(s, "계획", "목표값은 팀 합의 후 확정한다 — 여기서는 무엇을 측정할지만 먼저 고정한다.")
    add_page_number(s, len(prs.slides))


def build_s18():
    s = new_slide()
    add_header(s, "기술개발 목표[2] — 기간별 게이트", eyebrow="09 · 목표")
    add_headline(s, "5주 안에 통과해야 할 여섯 게이트", size=19)
    add_table(s, ["게이트", "내용", "기간", "통과 기준"], [
        ["G1", "새 차체 구동계 이식·기본 주행·새 지도", "10/6~10/10", "지도 저장 뒤 같은 지도로 재기동"],
        ["G2", "station 교시·반복 접근", "10/13~10/17", "0.05m·3°·정지 1초, 반복 N회를 성공/전체로 기록"],
        ["G3", "웹 주문→Nav2 실차 이동+조작 Action", "10/20~10/24", "한 주문 ID로 이동·조작 호출·복귀 기록 연결"],
        ["G4", "무선 두절·도킹·충전 신호", "10/20~10/24", "두절 시 정지·재연결 기록, 충전 신호 확인"],
        ["G5", "순정 그리퍼 시뮬 재실행·IK 실물 건식", "10/27~10/31", "시뮬 1회 이상 통과 + 실물 건식 기록"],
        ["G6", "전 lane 연결·통합 리허설", "11/2~11/3", "최종 발표 전 end-to-end 기록 1건 이상"],
    ], top=2.1, height=3.95, col_widths=[1.1, 5.4, 2.1, 3.5], font_size=10.5)
    add_insight(s, "사람별", "조작 정책 = 10회 롤아웃 평가 → 시작 위치 다양화 → 실패 지점 재수집. 기구·인지 lane 목표는 담당자 확인 후 확정.")
    add_page_number(s, len(prs.slides))


def build_s19():
    s = new_slide()
    add_header(s, "핵심 포인트", eyebrow="10 · 핵심 포인트")
    add_headline(s, "오늘 꼭 기억해 주셨으면 하는 네 가지")
    add_cards_grid(s, [
        {"kicker": None, "title": "웹 주문 한 건, 12 phase 완주", "desc": "473.9초 만에 도크 복귀까지 · 운반 중 물 손실 0", "badge": "sim"},
        {"kicker": None, "title": "복도 77 m 왕복", "desc": "20/20 waypoint · recovery 0회 (선행 JD-AMR 구동계)", "badge": "real"},
        {"kicker": None, "title": "SmolVLA 정책 학습", "desc": "eval_loss 38.8%↓(20→60 에피소드) · 실물 롤아웃 태스크 완주 관찰", "badge": "real"},
        {"kicker": None, "title": "오픈소스 팔 + 서비스 계층", "desc": "역할분리·주문기록·phase 비교·실패 규약 네 가지를 더한다", "badge": "plan"},
    ], cols=2, top=2.15, bottom=6.3)
    add_insight(s, "핵심 포인트", "시뮬 1회·실차 1회·실물 1회의 근거 위에, 서비스 계층이라는 포지셔닝을 더한다.")
    add_page_number(s, len(prs.slides))


def build_s20():
    s = new_slide()
    add_header(s, "개발 추진 계획 — 추진 체계", eyebrow="11 · 추진 계획")
    add_headline(s, "네 lane이 하나의 Action 계약에서 만난다", size=19)
    add_cards_grid(s, [
        {"kicker": "이동·연결", "title": "담당 · 임주영", "desc": ["SLAM/Nav2·장애물 회피", "station 반복 접근", "넘기는 것 → 조작 Action 호출 계약"]},
        {"kicker": "기구·하드웨어", "title": "담당 · 확인 후 표기", "desc": ["차체·상판·그리퍼·배선", "넘기는 것 → 실측 치수·장착 좌표"]},
        {"kicker": "조작 정책", "title": "담당 · 장준서", "desc": ["양팔 물 따르기 모방학습", "넘기는 것 → phase별 정책 checkpoint"]},
        {"kicker": "인지·물 양", "title": "담당 · 확인 후 표기", "desc": ["컵·물통 위치, 물 양 판정", "넘기는 것 → 물 양 판정 결과"]},
    ], cols=4, top=2.3, bottom=6.25)
    add_insight(s, "만나는 지점", "ExecuteManipulationSkill ROS 2 Action 계약(요청·결과·취소·timeout) — 각 lane의 결과물이 같은 형식으로 들어오고 나간다.")
    add_page_number(s, len(prs.slides))


def build_s21():
    s = new_slide()
    add_header(s, "개발 추진 계획 — 수행 일정", eyebrow="11 · 추진 계획")
    add_headline(s, "5주의 시나리오")
    add_cards_grid(s, [
        {"kicker": "1주차 · 10/6", "title": "새 차체가 굴러간다", "desc": ["구동계 이식·기본 주행", "식당 공간 새 지도 생성"]},
        {"kicker": "2주차 · 10/13", "title": "정해진 자리에 정확히 선다", "desc": ["station 교시", "반복 접근 성공률 기록"]},
        {"kicker": "3주차 · 10/20", "title": "웹 주문이 실제 주행을 움직인다", "desc": ["Nav2 실차 이동 + 조작 Action", "무선·도킹·충전 신호 시험"]},
        {"kicker": "4주차 · 10/27", "title": "순정 그리퍼로 다시 붓는다", "desc": ["시뮬 전체 재실행", "IK 컵 파지 실물 건식 시험"]},
        {"kicker": "5주차 · 11/2", "title": "네 lane이 하나로 만난다", "desc": ["정책·인지 결과물 연결", "통합 리허설"], "accent": ORANGE},
    ], cols=5, top=2.35, bottom=6.25)
    add_insight(s, "기준일 확인 필요", "부트캠프 수료 11/4 전후 일정과 실기체 사용 가능 기간에 맞춰 최종 확정한다.")
    add_page_number(s, len(prs.slides))


def build_s22():
    s = new_slide()
    add_header(s, "예상 난제와 해결 방안", eyebrow="12 · 난제와 대응")
    add_headline(s, "막힐 것으로 예상되는 지점, 지금부터 대응", size=19)
    add_table(s, ["분야", "예상 난제", "해결 방안"], [
        ["주행", "팔·상판이 올라간 새 차체의 정지거리·footprint 변화", "실제 외곽으로 footprint 재작성, 실측값 반영"],
        ["온보드", "Pi 4B USB 포트 수·단일 컨트롤러 대역폭", "센서 재배치, 팔 버스 통합 검토, Pi 5 전환 조건 측정"],
        ["무선", "추론 지연·두절 중 오래된 명령 실행", "관측 ID·유효기간 계약, 두절 시 안전 hold"],
        ["조작", "SO-101 페이로드와 물통 무게", "무게 상한 실측, 컵 몸통 중간 수평 파지"],
        ["물 양", "투명 컵·반사로 액면 인식 어려움", "컵 한 종류·고정 배경부터, 컵별 보정표, 저울 대조"],
        ["통합", "lane별 결과물의 입출력 불일치", "ROS 2 Action 계약 선확정, mock으로 먼저 연결"],
    ], top=2.15, height=3.9, col_widths=[1.6, 5.1, 5.0], highlight_col=2, font_size=11)
    add_insight(s, "원칙", "난제는 숨기지 않고 먼저 꺼내, 대응 방안과 함께 보여준다.")
    add_page_number(s, len(prs.slides))


def build_s23():
    s = new_slide()
    add_header(s, "팀 우수성 및 수행실적[1] — 이동·연결", eyebrow="13 · 팀")
    add_headline(s, "기계공학에서 로봇 주행으로", size=19)

    box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.6), Inches(2.2), Inches(7.0), Inches(3.85))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.32)
    tf.margin_top = Inches(0.22)
    tf.margin_right = Inches(0.32)
    set_text(tf, "담당 · 임주영", size=12.5, bold=True, color=ORANGE)
    add_para(tf, "기계공학 학사(FEA·위상최적화) → Unity 개발 약 5년(모바일 게임 1인 개발·과학실험 시뮬 80개) → DAPIER 부트캠프(ROS 2·SLAM/Nav2, Isaac Lab) · 기업 AI 활용 교육 보조강사 3개 과정", size=10.5, color=INK)
    add_para(tf, "ROS 2 Jazzy · Nav2 · Cartographer · AMCL · Keepout · DDS 진단 · URDF · Isaac Sim/Gazebo · Python · CI", size=9.5, color=GRAY)
    add_para(tf, " ", size=5)
    add_para(tf, "선행 JD-AMR 복도 77.090m 왕복 — 20/20 waypoint 통과, Nav2 recovery 0회 (9/4)", size=11, color=INK, bullet=True)
    add_para(tf, "장애물 배치 주행 20/20 · AMCL 78.498m · 정지 6회 후 재개 · recovery 최대 3회 (9/10)", size=11, color=INK, bullet=True)
    add_para(tf, "원격 통신 분리로 LiDAR 공백 10.750초 → 0.112초", size=11, color=INK, bullet=True)
    add_para(tf, "시뮬·Gazebo: Cartographer 선택 — ATE 0.645m(SLAM Toolbox 4.355m) · 2.5D 재현 경유점 20/20", size=11, color=INK, bullet=True)
    add_para(tf, "코드·단위시험: 식당 station 상태기계(충전소→주방→테이블→복귀) · 도착판정 0.05m·3°·정지1초 · Keepout 안전여유 0.55m", size=11, color=INK, bullet=True)

    w_real = add_badge(s, 0.6, 6.12, "real", custom_text="실차 · 1회")
    add_badge(s, 0.6 + w_real + 0.15, 6.12, "code", custom_text="코드 · 단위시험")

    rx, rw = 7.85, 4.883
    vid_w = (rw - 0.15) / 2
    vid_h = vid_w * 9 / 16
    add_movie(s, MEDIA / "09_JDAMR_실차_박스우회_11s.mp4", rx, 2.2, vid_w, vid_h,
              poster_frame_image=MEDIA / "09_JDAMR_실차_박스우회_포스터.png")
    add_movie(s, MEDIA / "10_JDAMR_실차_장애물앞정지_11s.mp4", rx + vid_w + 0.15, 2.2, vid_w, vid_h,
              poster_frame_image=MEDIA / "10_JDAMR_실차_장애물앞정지_포스터.png")
    cap1 = s.shapes.add_textbox(Inches(rx), Inches(2.2 + vid_h + 0.04), Inches(vid_w), Inches(0.28))
    set_text(cap1.text_frame, "박스 우회 · 2배속", size=9, color=GRAY, align=PP_ALIGN.CENTER)
    cap2 = s.shapes.add_textbox(Inches(rx + vid_w + 0.15), Inches(2.2 + vid_h + 0.04), Inches(vid_w), Inches(0.28))
    set_text(cap2.text_frame, "장애물 앞 정지 · 2배속", size=9, color=GRAY, align=PP_ALIGN.CENTER)

    still_y = 2.2 + vid_h + 0.36
    cap3 = s.shapes.add_textbox(Inches(rx), Inches(still_y), Inches(rw), Inches(0.25))
    set_text(cap3.text_frame, "실차 경로 기록(2.5D)", size=9, color=GRAY, align=PP_ALIGN.CENTER)
    still = MEDIA / "11_JDAMR_실차_경로기록.png"
    if still.exists():
        add_picture_fit(s, still, rx, still_y + 0.27, rw, 6.05 - (still_y + 0.27))

    add_insight(s, "실차 근거", "선행 JD-AMR 구동계를 새 차체가 그대로 재사용한다 — JD-AMR 검증이 이 팀의 출발점이다.")
    add_page_number(s, len(prs.slides))


def build_s24():
    s = new_slide()
    add_header(s, "팀 우수성 및 수행실적[2] — 조작 정책", eyebrow="13 · 팀")
    add_headline(s, "텔레옵 시연에서 학습 정책까지", size=19)
    box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.0), Inches(2.25), Inches(SLIDE_W_IN - 2.0), Inches(4.0))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.4)
    tf.margin_top = Inches(0.3)
    tf.margin_right = Inches(0.4)
    set_text(tf, "담당 · 장준서", size=13, bold=True, color=ORANGE)
    add_para(tf, "왼팔 컵·오른팔 물통 물 따르기를 텔레옵 시연 기반 모방학습으로 구현", size=13, color=INK)
    add_para(tf, " ", size=6)
    add_para(tf, "텔레옵 에피소드 60개 수집(20⊂40⊂60 부분집합으로 데이터 양 효과 비교)", size=13.5, color=INK, bullet=True)
    add_para(tf, "모델 4종 시도(ACT·SmolVLA·pi0·GR00T), 학습까지 진행한 3종 비교 → SmolVLA 선택", size=13.5, color=INK, bullet=True)
    add_para(tf, "SmolVLA eval_loss: 20개 0.2549 → 60개 0.1559 (38.8%↓)", size=13.5, color=INK, bullet=True)
    add_para(tf, "SmolVLA-40 실물 롤아웃 1회에서 태스크 완주 관찰", size=13.5, color=INK, bullet=True)
    add_para(tf, "pi0 원격 추론 지연 7.4초 → 130ms대 단축", size=13.5, color=INK, bullet=True)
    add_para(tf, "그리퍼 서보 레지스터 문제 2건 원인 규명·복구", size=13.5, color=INK, bullet=True)
    add_badge(s, 1.3, 2.55, "real", custom_text="실물 · 1회")
    add_insight(s, "다음 목표", "고정 프로토콜 10회 롤아웃 정량 평가 → 시작 위치 다양화 데이터 → 실패 지점 재수집.")
    add_page_number(s, len(prs.slides))


def build_s24b():
    s = new_slide()
    add_header(s, "팀 우수성 및 수행실적[3] — 연결·시뮬검증", eyebrow="13 · 팀")
    add_headline(s, "웹 주문 한 건을 12개 phase로 이어 도크 복귀까지", size=18)
    add_flow(s, ["웹 주문", "미션 관제", "ROS 2 조작 Action", "Isaac Sim 물리 실행", "결과 회신"], top=2.25, height=0.95, box_font=11)
    add_badge(s, 0.6, 3.34, "code", custom_text="코드 · 단위시험")
    bx, by, bw, bh = 0.6, 3.78, 7.3, 2.55
    add_bullet_box(s, bx, by, bw, bh, None, [
        "웹 관제: 4개 테이블 냉수·온수 주문, 주문 ID 중복 방지, 우선순위 큐, 배터리 부족 시 충전 선행, SQLite 저장·재시작 복구",
        "ROS 2 ExecuteManipulationSkill Action: 성공·취소·timeout 구분, 늦게 온 결과 무효 처리",
        "Isaac Sim(Python 3.11) ↔ ROS 2 Jazzy(3.12)를 소켓 IPC로 분리, phase 사이 물리 상태 유지",
    ], bullet_size=11.5)
    img = MEDIA / "07_웹관제_dryrun_캡처.png"
    if img.exists():
        add_picture_fit(s, img, 8.15, by, 4.55, bh)
    add_insight(s, "확인 범위", "CPU dry-run(실물·시뮬레이터 미접속) 1회 결과다 — 12개 phase 모두 성공, 시뮬 기준 473.9초.")
    add_page_number(s, len(prs.slides))


def build_s25a():
    s = new_slide()
    add_header(s, "팀 우수성 및 수행실적[4] — 기구·하드웨어", eyebrow="13 · 팀")
    add_headline(s, "담당자 확인 후 이력·실적 기입 예정", size=19)
    box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.6), Inches(2.2), Inches(7.3), Inches(4.05))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.32)
    tf.margin_top = Inches(0.2)
    tf.margin_right = Inches(0.32)
    set_text(tf, "담당 · 확인 후 표기", size=13, bold=True, color=ORANGE)
    add_para(tf, "개발 목적", size=10.5, bold=True, color=NAVY)
    add_para(tf, "양팔·RGB-D·LiDAR를 싣고 컵을 운반할 차체와 상판", size=11, color=INK)
    add_para(tf, "보유 기술·이력", size=10.5, bold=True, color=NAVY)
    add_para(tf, "[팀원 기입]", size=11, color=GRAY)
    add_para(tf, "한 일(후보)", size=10.5, bold=True, color=NAVY)
    add_para(tf, "상·하부 프레임 450×340mm, 상판 높이 720mm, 바퀴 포함 폭 540mm(설계·URDF 기준)", size=10.5, color=INK, bullet=True)
    add_para(tf, "추가 적재 5kg 수동 평지 주행 예비 확인(9/16, 정격 아님)", size=10.5, color=INK, bullet=True)
    add_para(tf, "그리퍼 선정: 왼손 SO-101 순정 죠, 오른손 ggao50 평행 그리퍼", size=10.5, color=INK, bullet=True)
    add_para(tf, "개인 목표·수행일", size=10.5, bold=True, color=NAVY)
    add_para(tf, "[팀원 기입]", size=11, color=GRAY)
    add_para(tf, "주고받는 것", size=10.5, bold=True, color=NAVY)
    add_para(tf, "실측 치수·장착 좌표 → URDF·Isaac 모델(이동·연결 lane)", size=11, color=INK)
    img = MEDIA / "02_URDF_축척렌더.png"
    if img.exists():
        add_picture_fit(s, img, 8.15, 2.2, 4.55, 4.05)
    add_insight(s, "정직한 상태", "팀원 확인 전까지는 지어내지 않고 비워둔다.")
    add_page_number(s, len(prs.slides))


def build_s25b():
    s = new_slide()
    add_header(s, "팀 우수성 및 수행실적[5] — 인지·물 양", eyebrow="13 · 팀")
    add_headline(s, "담당자 확인 후 이력·실적 기입 예정", size=19)
    add_flow(s, ["YOLO segmentation", "원근 보정", "액면 높이 비율", "컵별 보정표", "판정"], top=2.15, height=0.8, box_font=9)
    box = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.6), Inches(3.2), Inches(SLIDE_W_IN - 1.2), Inches(3.0))
    box.fill.solid()
    box.fill.fore_color.rgb = WHITE
    box.line.color.rgb = BORDER
    box.line.width = Pt(1)
    box.shadow.inherit = False
    tf = box.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.35)
    tf.margin_top = Inches(0.2)
    tf.margin_right = Inches(0.35)
    set_text(tf, "담당 · 확인 후 표기", size=13, bold=True, color=ORANGE)
    add_para(tf, "개발 목적", size=10.5, bold=True, color=NAVY)
    add_para(tf, "컵·물통 위치 인식과 물 양·흘림 판정, 붓는 동안 무게 변화 대응", size=11, color=INK)
    add_para(tf, "보유 기술·이력", size=10.5, bold=True, color=NAVY)
    add_para(tf, "[팀원 기입]", size=11, color=GRAY)
    add_para(tf, "계획된 방법", size=10.5, bold=True, color=NAVY)
    add_para(tf, "YOLO segmentation으로 컵 내부·액체·흘림 영역 분리", size=10.5, color=INK, bullet=True)
    add_para(tf, "원근 보정 → 액면 높이 비율 → 컵별 실측 보정표", size=10.5, color=INK, bullet=True)
    add_para(tf, "초과·흘림·판정 불가면 추가 기울임 금지, 최종 평가는 저울·눈금 대조", size=10.5, color=INK, bullet=True)
    add_para(tf, "개인 목표·수행일 · [팀원 기입]  ·  주고받는 것 · 컵·물통 위치·물 양 판정 → 조작 정책 lane·미션 상태기계", size=10, color=GRAY)
    add_insight(s, "정직한 상태", "팀원 확인 전까지는 지어내지 않고 비워둔다.")
    add_page_number(s, len(prs.slides))


def build_s26():
    s = new_slide()
    add_header(s, "기대 효과[1] — 적용 범위·후속 연계", eyebrow="14 · 기대 효과")
    add_headline(s, "식당에서 만든 미션 뼈대, 그 다음은 실험실", size=19)
    add_table(s, ["물 서빙", "실험실 적용"], [
        ["웹 주문", "실험 요청(시료 ID·목표량)"],
        ["주방·테이블 station", "시료 보관대·분석 장비 앞"],
        ["왼팔 컵·오른팔 물통 따르기", "한 팔 용기 고정·한 팔 시료 따르기"],
        ["물 양 판정(YOLO·보정표·저울)", "눈금·중량 기반 정량 확인"],
        ["주문 단위 기록·실패 규약", "실험 추적성(누가·언제·얼마나)"],
    ], top=2.15, height=3.5, col_widths=[5.6, 5.9], highlight_col=1, font_size=12)
    add_insight(s, "선행 사례", "리버풀대 이동형 화학 로봇 8일간 688회 실험(Nature, 2020) · 이동형 매니퓰레이터+양팔 로봇 3대의 12단계 워크플로(Chemical Science, 2024)")
    add_page_number(s, len(prs.slides))


def build_s27():
    s = new_slide()
    add_header(s, "기대 효과[2] — 실용성 관점", eyebrow="14 · 기대 효과")
    add_headline(s, "저비용 오픈소스 위의 서비스 계층")
    add_quote_block(
        s,
        quote="약 $660대 오픈소스 양팔 + 서비스 미션 계층",
        note="저가 오픈소스 양팔 위에 쌓은 서비스 계층이라, 초기 도입 비용 장벽이 상대적으로 낮다.",
        source=None,
    )
    add_insight(s, "열린 질문", "팔이 늘어난 만큼 비용·복잡도도 늘어난다 — 사업화 판단은 비용 대비 활용도 검증(G6) 이후로 열어둔다.")
    add_page_number(s, len(prs.slides))


def build_s28():
    s = new_slide()
    add_header(s, "보안·안전성 검증 및 조치", eyebrow="15 · 안전·보안")
    add_headline(s, "명령은 한 통로로, 실기체는 확인 후에만", size=19)
    add_cards_grid(s, [
        {"kicker": "명령 중재", "title": "단일 lease + 안전 게이트", "desc": "스톨·관절 한계·충돌 위험·통신 timeout·컵 이탈·흘림 위험은 실제 hold/stop으로 연결한다."},
        {"kicker": "무선 계약", "title": "유효기간 지난 명령은 폐기", "desc": "지연·중복·재연결 후 남은 행동은 Pi에서 폐기하고 안전 hold로 전환한다."},
        {"kicker": "데이터·운영", "title": "실물은 확인 후, 데이터는 비공개", "desc": "팔을 움직이는 모든 동작은 사용자 확인 후 실행하며, 데이터셋·모델은 HuggingFace Hub(private)에 둔다."},
    ], cols=3, top=2.4, bottom=6.25)
    add_insight(s, "원칙", "팔을 움직이는 모든 동작은 사용자 확인 후 실행한다.")
    add_page_number(s, len(prs.slides))


def build_s29():
    s = new_slide()
    add_header(s, "마무리", eyebrow="16 · 마무리")
    add_headline(s, "오늘 멘토링에서 검증받고 싶은 두 가지")
    add_cards_grid(s, [
        {"kicker": "궁금한 점 1", "title": "에이전트 계층의 경계", "desc": "자연어 주문 해석·예외 상황 설명을 에이전트에 맡길 때, 허용된 스킬 목록 안에서만 고르게 하는 설계가 현장에서 충분할지 검증받고 싶습니다."},
        {"kicker": "궁금한 점 2", "title": "모바일 매니퓰레이터의 현장 운용", "desc": "팔을 더한 서빙 로봇의 기능 대비 비용 판단과 매장 안전 운용 기준에 대한 조언을 구하고 싶습니다."},
    ], cols=2, top=2.2, bottom=5.15)
    tb = s.shapes.add_textbox(Inches(1.2), Inches(5.35), Inches(SLIDE_W_IN - 2.4), Inches(0.75))
    set_text(tb.text_frame, "물 한 잔을, 준비부터 테이블 위까지. 한 주문의 기록이 끝까지 이어지는 로봇을 만듭니다.", size=15, bold=True, color=NAVY, align=PP_ALIGN.CENTER)
    footer = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(6.55), prs.slide_width, Inches(0.95))
    footer.fill.solid()
    footer.fill.fore_color.rgb = NAVY
    no_line(footer)
    tf = footer.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    set_text(tf, "github.com/mmporong/bimanual-robot   ·   감사합니다", size=16, bold=True, color=WHITE, align=PP_ALIGN.CENTER)


def build_appendix_a():
    s = new_slide()
    add_header(s, "부록 A — 안전·실패데이터 규약·무선 계약", eyebrow="부록")
    add_headline(s, "본문 요약을 프로토콜 수준까지 풀어쓴 Q&A 백업", size=18)
    add_cards_grid(s, [
        {"kicker": "안전·명령 중재", "title": "lease 하나 → 중재기 → 안전 게이트 → 하드웨어", "desc": [
            "스톨·관절 한계·충돌 위험·통신 timeout·컵 이탈·흘림 위험은 hold/stop으로 연결",
            "정지가 항상 토크 OFF는 아니다 — 물체를 든 자세를 고려한다",
        ]},
        {"kicker": "실패 데이터 규약", "title": "L1→L5 순서로 원인을 배제한다", "desc": [
            "L1 하드웨어·연결 → L2 좌표 → L3 로직·파라미터 → L4 데이터·분포 → L5 측정",
            "실패 1건으로 고치지 않는다 — 같은 조건 3회 + 다른 조건 3회",
            "원인이 확인되지 않은 실패는 학습 데이터에 넣지 않는다",
        ]},
        {"kicker": "무선 계약", "title": "관측 ID·유효기간이 있는 명령만 실행", "desc": [
            "Pi → 노트북: 요청·관측 ID, 관절 상태·시각",
            "노트북 → Pi: 대응 관측 ID, checkpoint 버전, 관절 순서·단위·유효기간",
            "지연·중복·재연결 후 남은 행동은 폐기하고 안전 hold로 전환",
        ]},
    ], cols=3, top=2.4, bottom=6.25)
    add_insight(s, "이 자료는", "본문 15장(안전·보안)의 요약을 프로토콜 수준까지 풀어쓴 Q&A 백업이다 — 본문을 대체하지 않는다.")
    add_page_number(s, len(prs.slides))


def build_appendix_b():
    s = new_slide()
    add_header(s, "부록 B — 시뮬 수치 원본·예상 난제", eyebrow="부록")
    add_headline(s, "발표 중 세부 수치를 물으면 여기서 답한다", size=19)
    add_badge(s, 0.6, 2.15, "sim", custom_text="시뮬 · Isaac Sim 5.1 · 1회")
    add_table(s, ["실행", "조건", "결과"], [
        ["plate_service02", "물따르기→상판적재→주행→재파지→테이블배치 1회", "282.9초 · 컵684·병230·바깥4 · 팔·환경접촉 0"],
        ["dock_roundtrip02", "12 phase, 충전소 왕복 포함", "473.9초 · 컵687·병227·바깥4"],
        ["flared03", "입구 90mm 컵 양팔 물붓기", "유출 3/918 입자"],
        ["순정 SO-101 왼손 컵 접촉", "10g·210g 조건 파지·상승·내려놓기·손 분리", "각 1회 통과"],
    ], top=2.6, height=2.1, col_widths=[3.0, 4.8, 4.3], font_size=10.5)
    tb = s.shapes.add_textbox(Inches(0.6), Inches(4.85), Inches(SLIDE_W_IN - 1.2), Inches(0.5))
    set_text(tb.text_frame, "공통: 시뮬 정답 좌표 사용 · dock_roundtrip02/plate_service02는 왼손 80mm 강체 패드 형상 가정", size=10, color=GRAY)
    add_insight(s, "참고", "예상 난제·해결 방안 표는 본문 12장(22번 슬라이드)과 동일하다 — 여기서는 세부 시뮬 수치만 보탠다.")
    add_page_number(s, len(prs.slides))


def build_appendix_c():
    s = new_slide()
    add_header(s, "부록 C — 주제 변천·저장소 운영", eyebrow="부록")
    add_headline(s, "왜 지금 이 미션인가, 그리고 어떻게 운영하는가", size=18)
    add_columns(s, [
        {"title": "8/21", "desc": "양팔 커넥터 체결 셀"},
        {"title": "8/28", "desc": "이동형 용기 붓기·봉지 열기 병행"},
        {"title": "9/7", "desc": "물 서빙 단일 미션 확정 — 양팔 필요성·하드웨어 한계·데이터 수집 가능성·이동 통합 기준"},
    ], top=2.3, bottom=4.05)
    add_bullet_box(s, 0.6, 4.3, SLIDE_W_IN - 1.2, 2.0, "저장소 운영", [
        "Issue → 브랜치 → PR → CI 검사 → 병합",
        "데이터·모델은 HuggingFace Hub(private), 저장소에는 인덱스만",
        "실패 원인 귀속 순서와 재시험 규칙을 저장소 규칙으로 공유",
    ], bullet_size=13)
    add_insight(s, "운영 원칙", "결정 기준과 저장소 규칙을 투명하게 공개해 재현성을 높인다.")
    add_page_number(s, len(prs.slides))


def main():
    build_title()
    build_s02()
    build_s03()
    build_s04()
    build_s05()
    build_s06()
    build_s07()
    build_s08()
    build_s09()
    build_s10()
    build_s11()
    build_s12()
    build_s13()
    build_s14()
    build_s15()
    build_s16()
    build_s17()
    build_s18()
    build_s19()
    build_s20()
    build_s21()
    build_s22()
    build_s23()
    build_s24()
    build_s24b()
    build_s25a()
    build_s25b()
    build_s26()
    build_s27()
    build_s28()
    build_s29()
    build_appendix_a()
    build_appendix_b()
    build_appendix_c()
    prs.save(OUT)
    print(f"done: {len(prs.slides)} slides -> {OUT}")


if __name__ == "__main__":
    main()
