'use strict';
// MS팀 출정식 slides 1–10 (white+sky, delivered 2026-10-02)
const L = require('motion-deck');
const { pres, C, FR, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, cover, rounded, contain, dataUri, header, callout, newSlide, setVideo, tagPill, videoCard, seqLine, coverUntil } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;

async function s1() {
  newSlide({ transition: 'fade' });
  L.orbs('t', false, 'DDF1FC', 'CDEBFB');
  // white plate + ring framing the robot render
  circle(10.15, 3.72, 5.5, { fill: C.white, shadow: 'soft' }, { e: 'zoom', d: 0, t: 0.7 });
  circle(10.15, 3.72, 6.05, { line: C.mainM, lw: 1.5 }, { e: 'zoom', d: 250, t: 0.8 });
  const r = await contain('robot_w.png', 10.15 - 2.15, 3.72 - 1.62, 4.3, 3.24);
  img(A('robot_w.png'), r.x, r.y, r.w, r.h, { alt: '이동형 양팔 물 서빙 로봇 렌더' }, { e: 'up', d: 350, t: 0.8 });
  txt('피지컬 AI 부트캠프 DAPIER · 최종 프로젝트 출정식', 0.75, 1.5, 6.3, 0.36, { size: 14, font: FSB, color: C.mainD }, { e: 'fade', d: 300, t: 0.5 });
  txt('이동형 양팔\n물 서빙 로봇', 0.75, 1.95, 6.4, 2.25, { size: 58, bold: true, color: C.ink, ls: 1.08 }, { e: 'wipeR', d: 450, t: 0.7 });
  txt('물 한 잔을, 준비부터 테이블 위까지', 0.75, 4.35, 6.4, 0.5, { size: 22, font: FSB, color: C.main }, { e: 'fade', d: 1000, t: 0.6 });
  txt('2026.10.02(금)', 0.75, 5.0, 4, 0.4, { size: 16, color: C.muted }, { e: 'fade', d: 1200, t: 0.5 });
  img(A('logo_h_light_sky.png'), 0.75, 6.12, 2.68, 0.55, { name: '!!logo', alt: 'MS Team 로고' }, { e: 'fade', d: 1350, t: 0.5 });
}

async function s2() {
  newSlide();
  header({ num: '01', label: '개요 — 파이프라인', title: '웹 요청 한 번으로, 물 한 잔을 준비부터 테이블 위까지', orb: 'b' });
  const steps = ['웹 주문', '주방 이동', '작업대 정렬', '양팔\n물 따르기', '상판 적재 후\n이동', '테이블에\n컵 배치', '충전소 복귀\n· 충전'];
  const x0 = 1.25, x1 = 12.08, cy = 3.6, dx = (x1 - x0) / 6;
  line(x0, cy, x1, cy, { color: 'D0E3EF', w: 3 }, { e: 'wipeR', d: 0, t: 0.8 });
  line(x0, cy, x1, cy, { color: C.main, w: 3 }, { e: 'wipeR', d: 300, t: 2.4 });
  // traveller: robot badge rides above the track while steps light up
  const tx = x0, ty = 2.62;
  circle(tx, ty, 0.66, { fill: C.white, line: C.main, lw: 2, shadow: 'soft', name: 's2_bot' }, [{ e: 'fade', d: 0, t: 0.3 }, { e: 'path', d: 300, t: 2.4, dx: (x1 - x0) / 13.333, dy: 0 }]);
  await iconImg('LuBot', C.main, tx - 0.19, ty - 0.19, 0.38, [{ e: 'fade', d: 0, t: 0.3 }, { e: 'path', d: 300, t: 2.4, dx: (x1 - x0) / 13.333, dy: 0 }], { name: 's2_boticon' });
  steps.forEach((s, i) => {
    const cx = x0 + i * dx, orange = i === 3 || i === 5;
    const d = 300 + Math.round((i / 6) * 2400);
    txt(String(i + 1), cx - 0.42, cy - 0.42, 0.84, 0.84, { shape: pres.shapes.OVAL, fill: orange ? C.acc : C.main, color: C.white, bold: true, size: 20, align: 'center', valign: 'middle', line: C.white, lw: 3, check: false }, { e: 'grow', d, t: 0.35 });
    txt(s, cx - 0.85, 4.18, 1.7, 0.75, { size: 15, font: FSB, color: orange ? C.accD : C.ink, align: 'center', ls: 1.1 }, { e: 'fade', d: d + 100, t: 0.4 });
  });
  // return path: every order ends back at the dock
  const yb = 5.12;
  line(x1, 4.98, x1, yb, { color: C.mainM, w: 1.5, dash: 'dash' }, { e: 'fade', d: 2800, t: 0.3 });
  line(x1, yb, x0, yb, { color: C.mainM, w: 1.5, dash: 'dash' }, { e: 'wipeL', d: 2900, t: 0.8 });
  line(x0, yb, x0, 4.98, { color: C.mainM, w: 1.5, dash: 'dash', ea: 'triangle' }, { e: 'fade', d: 3600, t: 0.3 });
  circle(ML + 0.08, 5.52, 0.16, { fill: C.acc }, { e: 'fade', d: 2900, t: 0.4 });
  txt('주황 = 일반 서빙로봇에 없는 단계', ML + 0.26, 5.39, 5, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 2900, t: 0.4 });
  const rules = [['LuLock', '이동 중 컵은 상판 위 고정'], ['LuPause', '팔이 움직일 때 바퀴는 정지']];
  for (let i = 0; i < 2; i++) {
    const x = ML + i * (5.95 + 0.233), y = 5.9, a = { e: 'up', d: 3100 + i * 150, t: 0.5 };
    box(x, y, 5.95, 0.82, { r: 0.12, fill: C.panel }, a);
    await iconDot(rules[i][0], x + 0.52, y + 0.41, 0.5, C.mainT, C.main, a);
    txt(rules[i][1], x + 0.98, y + 0.12, 4.8, 0.58, { size: 16.5, font: FSB, color: C.ink, valign: 'middle' }, a);
  }
}

async function s3() {
  newSlide();
  header({ num: '02', label: '문제 인식 · 핵심 문제', title: "서빙로봇은 '나르기'까지, 마지막 1m는 사람 몫", orb: 'c' });
  const cats = ['2022', '2023', '2025', '2026(추정)'];
  L.S.slide.addChart(pres.charts.BAR, [
    { name: '보급 대수', labels: cats, values: [5000, 11000, 15000, 0] },
    { name: '추정치', labels: cats, values: [0, 0, 0, 17000] },
  ], {
    x: ML, y: 1.92, w: 5.75, h: 3.78, objectName: 's3_chart', barDir: 'col', barGrouping: 'stacked', overlap: 100, barGapWidthPct: 55,
    chartColors: [C.main, C.acc], showLegend: false,
    showTitle: true, title: '국내 서빙로봇 보급 대수(대)', titleFontFace: FR, titleFontSize: 13, titleColor: C.ink, titleBold: true,
    showValue: true, dataLabelPosition: 'inEnd', dataLabelColor: C.white, dataLabelFontFace: FR, dataLabelFontSize: 12, dataLabelFontBold: true, dataLabelFormatCode: '#,##0;;;',
    catAxisLabelFontFace: FR, catAxisLabelFontSize: 11.5, catAxisLabelColor: C.muted, catAxisLineShow: true, catAxisLineColor: 'B9CCDA',
    valAxisLabelFontFace: FR, valAxisLabelFontSize: 10, valAxisLabelColor: C.faint, valAxisLineShow: false, valAxisMaxVal: 18000, valAxisMajorUnit: 3000, valAxisLabelFormatCode: '#,##0',
    valGridLine: { color: 'E6F0F7', size: 0.75 }, catGridLine: { style: 'none' },
  });
  L.anim('s3_chart', { e: 'wipeU', d: 0, t: 0.9 });
  txt('* 2026년은 2023→2025 증가 추세를 근거로 한 추정치(실측 아님)', ML, 5.74, 5.75, 0.28, { size: 10.5, color: C.faint }, { e: 'fade', d: 700, t: 0.4 });
  const rows = [['음료 준비', '사람', 'h'], ['트레이에 올리기', '사람', 'h'], ['테이블까지 운반', '로봇', 'r'], ['테이블에 내려놓기', '손님·직원', 'h'], ['복귀', '로봇', 'r']];
  const x = 6.75, w = CR - x;
  for (let i = 0; i < rows.length; i++) {
    const [step, who, k] = rows[i];
    const y = 1.98 + i * 0.75, h = 0.64, hum = k === 'h', a = { e: 'up', d: 300 + i * 170, t: 0.45 };
    box(x, y, w, h, { r: 0.1, fill: hum ? C.accT : C.mainT, line: i === 3 ? C.acc : undefined, lw: 2 }, a);
    await iconDot(hum ? 'LuUserRound' : 'LuBot', x + 0.42, y + h / 2, 0.42, C.white, hum ? C.acc : C.main, a);
    txt(String(i + 1).padStart(2, '0'), x + 0.78, y + 0.13, 0.4, 0.38, { size: 12, bold: true, color: hum ? C.accD : C.main, valign: 'middle', check: false }, a);
    txt(step, x + 1.18, y + 0.1, 3.0, 0.44, { size: 16.5, font: FSB, color: C.ink, valign: 'middle' }, a);
    txt(who, x + w - 1.35, y + 0.15, 1.2, 0.34, { shape: RR, r: 0.17, fill: hum ? C.acc : C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a);
  }
  callout('문제 정의', '보급은 늘었고, 다음 과제는 한 대가 맡는 일의 범위', 6.15, {}, { e: 'up', d: 1350, t: 0.5 });
}

async function s4() {
  newSlide();
  header({ num: '02', label: '문제 인식 · 사회적 필요성', title: '왜 로봇 한 대당 일의 범위는 늘어야 하는가?', orb: 'a' });
  const cw = 3.85, gap = (CW - 3 * cw) / 2;
  const heads = ['일할 사람은 줄고, 대우는 낮다', '업계가 요청한 것은 자동화', '보급은 느는데, 활용 범위는 그대로'];
  for (let i = 0; i < 3; i++) {
    const x = ML + i * (cw + gap), d0 = i * 500;
    txt(String(i + 1).padStart(2, '0'), x, 2.0, 0.46, 0.46, { shape: pres.shapes.OVAL, fill: i === 2 ? C.acc : C.main, color: C.white, bold: true, size: 13, align: 'center', valign: 'middle', check: false }, { e: 'grow', d: d0, t: 0.35 });
    txt(heads[i], x + 0.58, 1.9, cw - 0.58, 0.68, { size: 16, bold: true, color: C.ink, valign: 'middle', ls: 1.1 }, { e: 'fade', d: d0 + 80, t: 0.4 });
    box(x, 2.72, cw, 4.0, { r: 0.12, fill: C.panel }, { e: 'up', d: d0 + 150, t: 0.5 });
    if (i < 2) await iconImg('LuChevronRight', C.mainM, x + cw + gap / 2 - 0.16, 2.06, 0.32, { e: 'fade', d: d0 + 350, t: 0.3 });
  }
  // column 1: two statistics
  let x = ML + 0.3;
  const kick = (t, xx, y, a) => txt(t, xx, y, 3.3, 0.3, { size: 12.5, font: FSB, color: C.muted }, a);
  kick('핵심 노동 연령층 전망', x, 2.98, { e: 'fade', d: 250, t: 0.4 });
  txt([{ text: '182', options: { fontSize: 46, bold: true, color: C.main } }, { text: '만 명', options: { fontSize: 24, bold: true, color: C.main } }], x, 3.28, 3.3, 0.78, { valign: 'bottom', check: false }, { e: 'zoom', d: 350, t: 0.5 });
  txt('2033년까지 약 182만 명 감소', x, 4.08, 3.3, 0.32, { size: 14, color: C.text }, { e: 'fade', d: 400, t: 0.4 });
  kick('외식업 임금 수준', x, 4.72, { e: 'fade', d: 450, t: 0.4 });
  txt([{ text: '58', options: { fontSize: 46, bold: true, color: C.main } }, { text: '%', options: { fontSize: 24, bold: true, color: C.main } }], x, 5.02, 3.3, 0.78, { valign: 'bottom', check: false }, { e: 'zoom', d: 550, t: 0.5 });
  txt('전 산업 평균 대비 58%', x, 5.82, 3.3, 0.32, { size: 14, color: C.text }, { e: 'fade', d: 600, t: 0.4 });
  txt('한국농촌경제연구원 인용, 2025', x, 6.3, 3.3, 0.26, { size: 10.5, color: C.faint }, { e: 'fade', d: 650, t: 0.4 });
  // column 2: one big statistic with a filled bar
  x = ML + (cw + gap) + 0.3;
  kick('외식업체가 1순위로 요청한 정책', x, 2.98, { e: 'fade', d: 700, t: 0.4 });
  txt([{ text: '68.8', options: { fontSize: 64, bold: true, color: C.main } }, { text: '%', options: { fontSize: 30, bold: true, color: C.main } }], x, 3.35, 3.3, 1.05, { valign: 'bottom', check: false }, { e: 'zoom', d: 850, t: 0.6 });
  txt('자동화·로봇 기술 지원 68.8%', x, 4.5, 3.3, 0.34, { size: 15, font: FSB, color: C.text }, { e: 'fade', d: 950, t: 0.4 });
  box(x, 5.1, 3.25, 0.22, { r: 0.11, fill: 'DCEBF4' }, { e: 'fade', d: 950, t: 0.3 });
  box(x, 5.1, 3.25 * 0.688, 0.22, { r: 0.11, fill: C.main }, { e: 'wipeR', d: 1050, t: 0.9 });
  txt('한국농촌경제연구원 인용, 2025', x, 6.3, 3.3, 0.26, { size: 10.5, color: C.faint }, { e: 'fade', d: 1100, t: 0.4 });
  // column 3: quote
  x = ML + 2 * (cw + gap) + 0.3;
  kick('업계가 꼽은 확산 걸림돌', x, 2.98, { e: 'fade', d: 1200, t: 0.4 });
  await iconImg('LuQuote', C.acc, x, 3.42, 0.5, { e: 'grow', d: 1300, t: 0.4 });
  txt('한 대가 한 가지 일만 하는 구조로 비용 상쇄 어려움', x, 4.1, 3.3, 1.9, { size: 24, bold: true, color: C.ink, ls: 1.2 }, { e: 'fade', d: 1400, t: 0.6 });
  txt('서울경제, 2026.9.21', x, 6.3, 3.3, 0.26, { size: 10.5, color: C.faint }, { e: 'fade', d: 1500, t: 0.4 });
}

async function s5() {
  newSlide();
  header({ num: '03', label: '솔루션', title: '준비 · 운반 · 전달 · 복귀를 모두 수행한다', orb: 'd' });
  const cards = [['01', '준비', '양팔로 물 따르기', 'LuGlassWater', true], ['02', '운반', '컵을 상판에 싣고\n자율주행', 'LuRoute', false], ['03', '전달', '테이블 앞 정렬 뒤\n컵을 테이블 위에 전달', 'LuHandPlatter', true], ['04', '복귀', '충전소 복귀', 'LuBatteryCharging', false]];
  const w = 2.86, g = (CW - 4 * w) / 3, y = 2.1, h = 2.98;
  for (let i = 0; i < 4; i++) {
    const [no, t, desc, ic, orange] = cards[i];
    const x = ML + i * (w + g), a = { e: 'up', d: i * 220, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, lw: 1, shadow: 'soft' }, a);
    await iconDot(ic, x + 0.75, y + 0.78, 0.92, orange ? C.accT : C.mainT, orange ? C.acc : C.main, a);
    txt(no, x + w - 1.05, y + 0.38, 0.8, 0.6, { size: 30, bold: true, color: orange ? 'F6C69E' : C.mainT2, align: 'right', check: false }, a);
    txt(t, x + 0.3, y + 1.5, w - 0.6, 0.62, { size: 28, bold: true, color: orange ? C.accD : C.ink }, a);
    txt(desc, x + 0.3, y + 2.15, w - 0.5, 0.85, { size: 15, color: C.muted, ls: 1.15 }, a);
  }
  // connectors go on top of every card so the next card never hides them
  for (let i = 0; i < 3; i++) {
    const cx = ML + i * (w + g) + w + g / 2, ac = { e: 'grow', d: i * 220 + 260, t: 0.3 };
    circle(cx, y + h / 2, 0.42, { fill: C.white, line: C.line, lw: 1 }, ac);
    await iconImg('LuChevronRight', C.main, cx - 0.13, y + h / 2 - 0.13, 0.26, ac);
  }
  circle(ML + 0.08, 5.6, 0.16, { fill: C.acc }, { e: 'fade', d: 1000, t: 0.4 });
  txt('주황 = 일반 서빙로봇에 없는 단계', ML + 0.26, 5.47, 5, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 1000, t: 0.4 });
  callout('가설', '여러 업무를 한 대에 묶으면 활용도가 올라간다', 6.12, {}, { e: 'up', d: 1150, t: 0.5 });
}

async function s6() {
  newSlide();
  header({ num: '04', label: '업계 현황 · 시중 서빙로봇 대비', title: '우리 로봇은 무엇이 다른가', orb: 'e' });
  img(await rounded('bear.jpg', 2.15, 3.7, 0.14), ML, 1.95, 2.15, 3.7, { alt: '일반 서빙로봇 사진' }, { e: 'fade', d: 0, t: 0.6 });
  txt('일반 서빙로봇', ML, 5.7, 2.15, 0.3, { size: 12, color: C.muted, align: 'center' }, { e: 'fade', d: 0, t: 0.6 });
  const o = await contain('ours.png', 10.58, 1.95, 2.15, 3.7);
  img(A('ours.png'), o.x, o.y, o.w, o.h, { alt: '이동형 양팔 서빙 로봇 렌더' }, { e: 'fade', d: 100, t: 0.6 });
  txt('이동형 양팔 서빙 로봇', 10.38, 5.7, 2.55, 0.3, { size: 12, color: C.main, bold: true, align: 'center' }, { e: 'fade', d: 100, t: 0.6 });
  const x0 = 3.0, cw = [1.42, 2.78, 3.0], g = 0.06;
  const xs = [x0, x0 + cw[0] + g, x0 + cw[0] + cw[1] + 2 * g];
  const hA = { e: 'fade', d: 200, t: 0.4 };
  txt('업무', xs[0], 1.95, cw[0], 0.48, { size: 12.5, bold: true, color: C.muted, valign: 'middle', margin: 6 }, hA);
  txt('일반 서빙로봇', xs[1], 1.95, cw[1], 0.48, { shape: RR, r: 0.08, fill: 'E6EEF3', size: 13.5, bold: true, color: C.muted, valign: 'middle', align: 'center' }, hA);
  txt('이동형 양팔 서빙 로봇', xs[2], 1.95, cw[2], 0.48, { shape: RR, r: 0.08, fill: C.main, size: 13.5, bold: true, color: C.white, valign: 'middle', align: 'center' }, hA);
  const rows = [['주문 연결', '직원이 음식을 올리고 목적지 입력', '웹 주문이 TASK로 바로 연결'], ['음료 준비', '사람', '양팔 물 따르기', 1], ['운반', '트레이 운반', '상판 적재 운반'], ['테이블 위 전달', '손님·직원이 내려놓음', '컵을 다시 잡아 테이블에 배치', 1], ['복귀', '대기 위치 복귀', '충전소 복귀 · 충전 시작 확인'], ['기록', '주행 로그', '주문 단위 phase 기록 · 실패 원인 기록']];
  rows.forEach((r, i) => {
    const y = 2.5 + i * 0.56, h = 0.51, a = { e: 'wipeR', d: 350 + i * 160, t: 0.5 }, hi = r[3];
    txt(r[0], xs[0], y, cw[0], h, { size: 13.5, bold: true, color: C.ink, valign: 'middle', margin: 6 }, a);
    txt(r[1], xs[1], y, cw[1], h, { fill: C.panel, size: 12.5, color: C.muted, valign: 'middle', margin: 7, ls: 1.05 }, a);
    txt(r[2], xs[2], y, cw[2], h, { fill: hi ? C.accT : C.mainT, size: 12.5, font: FSB, color: hi ? C.accD : C.ink, valign: 'middle', margin: 7, ls: 1.05 }, a);
  });
  callout('질문', '팔을 더하면 가격과 복잡도도 늘어난다\n→ 기능 대비 비용은 멘토링에서 검증받고 싶은 질문', 6.12, { h: 0.74, size: 14 }, { e: 'up', d: 1400, t: 0.5 });
}

async function s7() {
  newSlide();
  header({ num: '05', label: '목표 · 개발 목표[1] — 구현 목표', title: '한번의 주문이 요구하는 모든 과정을 로봇이 직접 수행하는 것', orb: 'b' });
  const ph = ['대기 · 충전', '주문 접수', '도크 이탈', '주방 이동', '주방 정렬', '주방 조작', '물 양 · 적재 확인', '테이블 이동 · 정렬', '테이블 배치 · 확인', '도크 이동 · 도킹', '충전'];
  const x0 = 1.0, x1 = 12.33, cy = 2.32, dx = (x1 - x0) / 10;
  line(x0, cy, x1, cy, { color: C.mainT2, w: 2.5 }, { e: 'wipeR', d: 0, t: 1.6 });
  ph.forEach((p, i) => {
    const cx = x0 + i * dx, hi = i === 5 || i === 8, d = 150 + i * 130;
    txt(String(i + 1), cx - 0.25, cy - 0.25, 0.5, 0.5, { shape: pres.shapes.OVAL, fill: hi ? C.acc : C.main, color: C.white, bold: true, size: 13, align: 'center', valign: 'middle', line: C.white, lw: 2.5, check: false }, { e: 'grow', d, t: 0.3 });
    txt(p.replace(' · ', ' ·\n'), cx - 0.56, 2.7, 1.12, 0.62, { size: 12.5, font: FSB, color: hi ? C.accD : C.ink, align: 'center', ls: 1.05 }, { e: 'fade', d: d + 80, t: 0.35 });
  });
  // expansion of phase 6 (kitchen manipulation)
  const cx6 = x0 + 5 * dx;
  line(cx6, 3.38, cx6, 3.62, { color: C.acc, w: 1.5 }, { e: 'wipeD', d: 1700, t: 0.25 });
  txt('컵 파지 · 물통 파지 · 붓기 준비 · 붓기 · 물통 반환 · 상판 적재', cx6 - 3.6, 3.62, 7.2, 0.5, { shape: RR, r: 0.25, fill: C.accT, line: 'F4B27A', lw: 1, size: 14.5, font: FSB, color: C.accD, align: 'center', valign: 'middle' }, { e: 'zoom', d: 1800, t: 0.45 });
  // media row: restaurant scene + pouring clip (3x, smooth 60 fps edit)
  const my = 4.42, mh = 2.4, vw = mh * 16 / 9, vx = CR - vw, iw = CW - vw - 0.22;
  img(await rounded('restaurant.png', iw, mh, 0.14), ML, my, iw, mh, { alt: '식당 서빙 장면 이미지' }, { e: 'fade', d: 2100, t: 0.6 });
  L.S.slide.addMedia({ type: 'video', path: A('v7n.mp4'), cover: await dataUri('poster7n.jpg'), x: vx, y: my, w: vw, h: mh, objectName: 's7_video' });
  L.anim('s7_video', { e: 'fade', d: 2250, t: 0.6 });
  // LibreOffice ignores entrance effects on media, so a background-coloured cover hides the clip until it starts
  coverUntil(vx - 0.02, my - 0.02, vw + 0.04, mh + 0.04, 2250, { name: 's7_cover' });
  setVideo('s7_video', 10000, 2250);
  txt('3배속 편집 영상', 10.45, 4.14, CR - 10.45, 0.24, { size: 10.5, color: C.faint, align: 'right', check: false }, { e: 'fade', d: 2300, t: 0.4 });
}

async function s8() {
  newSlide();
  header({ num: '05', label: '목표 · 개발 목표[2] — 핵심 정량 지표', title: '성공의 정량적 기준은 무엇인가', orb: 'c' });
  const g1 = { e: 'fade', d: 0, t: 0.4 };
  txt('성공 판정 지표', ML, 1.95, 4, 0.36, { size: 16.5, bold: true, color: C.main }, g1);
  txt('시스템 성능 지표', 9.05, 1.95, 3.6, 0.36, { size: 16.5, bold: true, color: C.subD }, { e: 'fade', d: 900, t: 0.4 });
  const cards = [['LuCircleCheck', 'phase 성공', 'phase별 전체 시도 대비 성공 횟수'], ['LuDroplet', '물 양 오차', '목표 물 양 오차(mL),\nUnder/Over/Spill/Unknown 비율'], ['LuHand', '사람 개입', '사람 개입·복구 횟수'], ['LuTimer', 'cycle time', '1회 소요 시간(cycle time)']];
  const cw = 4.05, ch = 1.6, gx = 0.2, gy = 0.22;
  for (let i = 0; i < 4; i++) {
    const x = ML + (i % 2) * (cw + gx), y = 2.45 + Math.floor(i / 2) * (ch + gy), a = { e: 'zoom', d: 150 + i * 170, t: 0.45 };
    box(x, y, cw, ch, { r: 0.12, fill: C.white, line: C.line, shadow: 'soft' }, a);
    await iconDot(cards[i][0], x + 0.58, y + 0.6, 0.66, C.mainT, C.main, a);
    txt(cards[i][1], x + 1.08, y + 0.33, cw - 1.2, 0.5, { size: 21, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(cards[i][2], x + 0.32, y + 0.95, cw - 0.55, 0.58, { size: 13.5, color: C.muted, ls: 1.1 }, a);
  }
  const x = 9.05, y = 2.45, w = CR - x, h = 2 * ch + gy, a = { e: 'zoom', d: 1000, t: 0.5 };
  box(x, y, w, h, { r: 0.12, fill: C.subT, line: 'CDE6F6' }, a);
  await iconDot('LuActivity', x + 0.62, y + 0.66, 0.74, C.white, C.subD, a);
  txt('지연 · 지터', x + 0.32, y + 1.35, w - 0.6, 0.5, { size: 22, bold: true, color: C.ink }, a);
  txt('명령 지연·제어 지터\n(ms 단위)', x + 0.32, y + 1.9, w - 0.6, 0.7, { size: 14.5, color: C.muted, ls: 1.1 }, a);
  // small jitter trace: animated as a draw-on
  const pts = [0, 0.18, -0.12, 0.25, -0.2, 0.1, -0.05, 0.22, -0.16, 0.08];
  const bx = x + 0.32, by = y + 3.05, step = (w - 0.64) / (pts.length - 1);
  for (let i = 0; i < pts.length - 1; i++) line(bx + i * step, by - pts[i], bx + (i + 1) * step, by - pts[i + 1], { color: C.sub, w: 2 }, { e: 'wipeR', d: 1250 + i * 60, t: 0.12 });
  callout('계획', '목표값은 추후 팀 합의 후 확정', 6.12, {}, { e: 'up', d: 1500, t: 0.5 });
}

async function s9() {
  newSlide();
  header({ num: '06', label: '데모 · 시뮬레이션 데모', title: '그래서, 실제로 되나요?', orb: 'a' });
  const vw = 7.75, vh = vw * 9 / 16;
  await videoCard('s9_video', 'v9n.mp4', 'poster9n.jpg', ML, 1.9, vw, vh, 19883, 600);
  seqLine(['컵·병 파지', '물따르기', '상판 적재', '이동', '컵 재파지', '테이블 배치'], ML, 1.9 + vh + 0.16, vw, 400);
  const x = 8.75, w = CR - x;
  const t1 = tagPill('시뮬 · Isaac Sim 5.1 · 1회', x, 1.9, 300);
  tagPill('영상 3배속', x + t1 + 0.12, 1.9, 300, { fill: C.panel, color: C.muted });
  txt([{ text: '473.9', options: { fontSize: 60, bold: true, color: C.main } }, { text: '초', options: { fontSize: 28, bold: true, color: C.mainD } }], x, 2.42, w, 1.05, { valign: 'bottom', check: false }, { e: 'zoom', d: 450, t: 0.6 });
  txt('완주 시간 · 도크 복귀까지', x, 3.5, w, 0.32, { size: 13.5, color: C.muted }, { e: 'fade', d: 550, t: 0.4 });
  const rows = [['phase 통과', [{ text: '12/12', options: { fontSize: 24, bold: true, color: C.main } }, { text: '  dock_roundtrip02', options: { fontSize: 11.5, color: C.faint } }]], ['운반 중 추가 손실', [{ text: '0', options: { fontSize: 24, bold: true, color: C.main } }]], ['이동·정렬', [{ text: '주방·테이블 앞 정렬', options: { fontSize: 16, bold: true, color: C.ink } }]]];
  rows.forEach((r, i) => {
    const y = 4.05 + i * 0.66, a = { e: 'up', d: 700 + i * 150, t: 0.4 };
    line(x, y, CR, y, { color: C.line, w: 1 }, a);
    txt(r[0], x, y + 0.08, 1.55, 0.5, { size: 12.5, color: C.muted, valign: 'middle' }, a);
    txt(r[1], x + 1.55, y + 0.06, w - 1.55, 0.54, { valign: 'middle', check: false }, a);
  });
  txt('왼손은 이전 후보(80mm 강체 패드) 형상 가정 — 현행 순정 그리퍼 재실행은 로드맵', x, 6.05, w, 0.66, { size: 11.5, color: C.muted, ls: 1.15 }, { e: 'fade', d: 1200, t: 0.4 });
}

async function s10() {
  newSlide();
  header({ num: '06', label: '데모 · 실물 데모 (물 따르기)', title: '실물로도 검증되었나요?', orb: 'b' });
  const vw = 7.75, vh = vw * 9 / 16, vx = CR - vw;
  await videoCard('s10_video', 'v10.mp4', 'poster10.jpg', vx, 1.9, vw, vh, 38633, 600);
  seqLine(['컵·병 파지', '물따르기', '컵 상판 적재', '원 위치 복귀'], vx, 1.9 + vh + 0.16, vw, 400);
  const x = ML, w = vx - ML - 0.35;
  tagPill('smolVLA · 에피소드 40개 학습', x, 1.9, 300);
  const rows = [['실행 방식', 'IK 없이 학습 정책 단독 실행(전 구간 IL)'], ['수행 모델', 'SmolVLA (40 에피소드 학습)'], ['롤아웃 결과', '파지→물따르기→상판적재→원위치 복귀까지 완주', 1], ['정량 평가', '고정 프로토콜 10회 평가 진행 예정']];
  rows.forEach((r, i) => {
    const y = 2.45 + i * 0.78, a = { e: 'up', d: 450 + i * 150, t: 0.4 };
    line(x, y, x + w, y, { color: C.line, w: 1 }, a);
    txt(r[0], x, y + 0.08, w, 0.26, { size: 11.5, color: C.muted }, a);
    txt(r[1], x, y + 0.33, w, 0.42, { size: 14.5, bold: true, color: r[2] ? C.main : C.ink, ls: 1.05 }, a);
  });
  const cy = 5.6, ch = 1.12, a = { e: 'up', d: 1150, t: 0.5 };
  box(x, cy, w, ch, { r: 0.1, fill: C.panel }, a);
  txt('검증', x + 0.18, cy + 0.15, 0.78, 0.34, { shape: RR, r: 0.17, fill: C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a);
  txt('IK 없이 모방학습만으로 물따르기 TASK를 원활히 수행함을 확인.', x + 0.18, cy + 0.56, w - 0.36, 0.5, { size: 13, font: FSB, color: C.ink, ls: 1.1 }, a);
}

module.exports = [s1, s2, s3, s4, s5, s6, s7, s8, s9, s10];
