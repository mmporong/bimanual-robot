'use strict';
// MS팀 최종 발표 slides 1–8: 표지 · 개요 · 문제 · 솔루션 · 목표 달성 · 정량 지표 · 시스템 구조 · 팀
// ※ [임의] 표시가 붙은 수치는 실험 전 자리표시 값이다(발표 전 실측으로 교체).
const L = require('motion-deck');
const { pres, C, FR, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, rounded, contain, dataUri, header, callout, newSlide, setVideo, coverUntil } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;
const notes = (t) => L.S.slide.addNotes(t);

async function s1() {
  newSlide({ transition: 'fade' });
  L.orbs('t', false, 'DDF1FC', 'CDEBFB');
  circle(10.15, 3.72, 5.5, { fill: C.white, shadow: 'soft' }, { e: 'zoom', d: 0, t: 0.7 });
  circle(10.15, 3.72, 6.05, { line: C.mainM, lw: 1.5 }, { e: 'zoom', d: 250, t: 0.8 });
  const r = await contain('robot_w.png', 10.15 - 2.15, 3.72 - 1.62, 4.3, 3.24);
  img(A('robot_w.png'), r.x, r.y, r.w, r.h, { alt: '이동형 양팔 물 서빙 로봇 렌더' }, { e: 'up', d: 350, t: 0.8 });
  txt('피지컬 AI 부트캠프 DAPIER · 최종 프로젝트 발표', 0.75, 1.5, 6.3, 0.36, { size: 14, font: FSB, color: C.mainD }, { e: 'fade', d: 300, t: 0.5 });
  txt('이동형 양팔\n물 서빙 로봇', 0.75, 1.95, 6.4, 2.25, { size: 58, bold: true, color: C.ink, ls: 1.08 }, { e: 'wipeR', d: 450, t: 0.7 });
  txt('물 한 잔을, 준비부터 테이블 위까지', 0.75, 4.35, 6.4, 0.5, { size: 22, font: FSB, color: C.main }, { e: 'fade', d: 1000, t: 0.6 });
  txt('2026.11.06(금)', 0.75, 5.0, 4, 0.4, { size: 16, color: C.muted }, { e: 'fade', d: 1200, t: 0.5 });
  img(A('logo_h_light_sky.png'), 0.75, 6.12, 2.68, 0.55, { name: '!!logo', alt: 'MS Team 로고' }, { e: 'fade', d: 1350, t: 0.5 });
  notes('안녕하십니까, 물 한 잔을 준비부터 테이블 위까지.\n이동형 양팔 물 서빙 로봇을 만든 MS팀의 장준서입니다.\n출정식에서 말씀드린 계획을 5주 동안 어디까지 실제로 구현했는지, 결과 중심으로 말씀드리겠습니다.');
}

async function s2() {
  newSlide();
  header({ num: '01', label: '개요 — 파이프라인', title: '웹 주문 한 번으로, 물 한 잔을 준비부터 테이블 위까지', orb: 'b' });
  const steps = ['웹 주문', '주방 이동', '작업대 정렬', '양팔\n물 따르기', '상판 적재 후\n이동', '테이블에\n컵 배치', '충전소 복귀\n· 충전'];
  const x0 = 1.25, x1 = 12.08, cy = 3.6, dx = (x1 - x0) / 6;
  line(x0, cy, x1, cy, { color: 'D0E3EF', w: 3 }, { e: 'wipeR', d: 0, t: 0.8 });
  line(x0, cy, x1, cy, { color: C.main, w: 3 }, { e: 'wipeR', d: 300, t: 2.4 });
  const tx = x0, ty = 2.62;
  circle(tx, ty, 0.66, { fill: C.white, line: C.main, lw: 2, shadow: 'soft', name: 's2_bot' }, [{ e: 'fade', d: 0, t: 0.3 }, { e: 'path', d: 300, t: 2.4, dx: (x1 - x0) / 13.333, dy: 0 }]);
  await iconImg('LuBot', C.main, tx - 0.19, ty - 0.19, 0.38, [{ e: 'fade', d: 0, t: 0.3 }, { e: 'path', d: 300, t: 2.4, dx: (x1 - x0) / 13.333, dy: 0 }], { name: 's2_boticon' });
  steps.forEach((s, i) => {
    const cx = x0 + i * dx, orange = i === 3 || i === 5;
    const d = 300 + Math.round((i / 6) * 2400);
    txt(String(i + 1), cx - 0.42, cy - 0.42, 0.84, 0.84, { shape: pres.shapes.OVAL, fill: orange ? C.acc : C.main, color: C.white, bold: true, size: 20, align: 'center', valign: 'middle', line: C.white, lw: 3, check: false }, { e: 'grow', d, t: 0.35 });
    txt(s, cx - 0.85, 4.18, 1.7, 0.75, { size: 15, font: FSB, color: orange ? C.accD : C.ink, align: 'center', ls: 1.1 }, { e: 'fade', d: d + 100, t: 0.4 });
  });
  const yb = 5.12;
  line(x1, 4.98, x1, yb, { color: C.mainM, w: 1.5, dash: 'dash' }, { e: 'fade', d: 2800, t: 0.3 });
  line(x1, yb, x0, yb, { color: C.mainM, w: 1.5, dash: 'dash' }, { e: 'wipeL', d: 2900, t: 0.8 });
  line(x0, yb, x0, 4.98, { color: C.mainM, w: 1.5, dash: 'dash', ea: 'triangle' }, { e: 'fade', d: 3600, t: 0.3 });
  circle(ML + 0.08, 5.52, 0.16, { fill: C.acc }, { e: 'fade', d: 2900, t: 0.4 });
  txt('주황 = 일반 서빙로봇에 없는 단계 · 양팔 모방학습 정책으로 수행', ML + 0.26, 5.39, 8, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 2900, t: 0.4 });
  const rules = [['LuLock', '이동 중 컵은 상판 위 고정'], ['LuPause', '팔이 움직일 때 바퀴는 정지']];
  for (let i = 0; i < 2; i++) {
    const x = ML + i * (5.95 + 0.233), y = 5.9, a = { e: 'up', d: 3100 + i * 150, t: 0.5 };
    box(x, y, 5.95, 0.82, { r: 0.12, fill: C.panel }, a);
    await iconDot(rules[i][0], x + 0.52, y + 0.41, 0.5, C.mainT, C.main, a);
    txt(rules[i][1], x + 0.98, y + 0.12, 4.8, 0.58, { size: 16.5, font: FSB, color: C.ink, valign: 'middle' }, a);
  }
  notes('먼저 저희 로봇이 하는 일을 한 장으로 다시 정리했습니다.\n웹으로 주문이 들어오면 주방으로 이동해 작업대 앞에 정렬하고, 양팔로 물을 따른 뒤 컵을 상판에 싣고 이동합니다.\n테이블에 컵을 내려놓고 충전소로 복귀하는 것까지가 한 주문입니다.\n주황색으로 표시한 물 따르기와 컵 배치는 기존 서빙로봇에서는 사람이 하던 일이고, 저희는 이 두 단계를 모방학습 정책으로 수행했습니다.');
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
  notes('저희가 이 주제를 고른 이유를 다시 짚겠습니다.\n서빙로봇 보급은 꾸준히 늘었지만, 로봇이 맡는 일은 여전히 나르기에 머물러 있습니다.\n음료 준비와 테이블에 내려놓는 마지막 1m는 여전히 사람 몫이고, 저희는 로봇 한 대가 맡는 일의 범위를 넓히는 것을 문제로 정의했습니다.');
}

async function s4() {
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
  for (let i = 0; i < 3; i++) {
    const cx = ML + i * (w + g) + w + g / 2, ac = { e: 'grow', d: i * 220 + 260, t: 0.3 };
    circle(cx, y + h / 2, 0.42, { fill: C.white, line: C.line, lw: 1 }, ac);
    await iconImg('LuChevronRight', C.main, cx - 0.13, y + h / 2 - 0.13, 0.26, ac);
  }
  circle(ML + 0.08, 5.6, 0.16, { fill: C.acc }, { e: 'fade', d: 1000, t: 0.4 });
  txt('주황 = 일반 서빙로봇에 없는 단계', ML + 0.26, 5.47, 5, 0.28, { size: 12.5, color: C.muted }, { e: 'fade', d: 1000, t: 0.4 });
  callout('가설 → 결과', '여러 업무를 한 대에 묶는다는 가설을, 한 주문 안에서 네 단계를 실제로 연결해 확인했다', 6.12, {}, { e: 'up', d: 1150, t: 0.5 });
  notes('그래서 저희 솔루션은 준비, 운반, 전달, 복귀 전 과정을 로봇 한 대가 수행하는 것입니다.\n출정식 때는 가설이었던 이 구성을, 이번에는 실제 하드웨어 위에서 한 주문 흐름으로 연결했습니다.');
}

async function s5() {
  newSlide();
  header({ num: '04', label: '목표 달성 · 구현 목표 대비 결과', title: '11단계 전 과정을 로봇이 직접 수행했다', orb: 'b' });
  const ph = ['대기 · 충전', '주문 접수', '도크 이탈', '주방 이동', '주방 정렬', '주방 조작', '물 양 · 적재 확인', '테이블 이동 · 정렬', '테이블 배치 · 확인', '도크 이동 · 도킹', '충전'];
  const partial = new Set([6, 10]);   // 물 양 확인, 충전 = 신호 연동 남음
  const x0 = 1.0, x1 = 12.33, cy = 2.32, dx = (x1 - x0) / 10;
  line(x0, cy, x1, cy, { color: C.mainT2, w: 2.5 }, { e: 'wipeR', d: 0, t: 1.6 });
  ph.forEach((p, i) => {
    const cx = x0 + i * dx, hi = i === 5 || i === 8, part = partial.has(i), d = 150 + i * 130;
    txt(String(i + 1), cx - 0.25, cy - 0.25, 0.5, 0.5, { shape: pres.shapes.OVAL, fill: part ? 'A9BCCB' : hi ? C.acc : C.main, color: C.white, bold: true, size: 13, align: 'center', valign: 'middle', line: C.white, lw: 2.5, check: false }, { e: 'grow', d, t: 0.3 });
    txt(p.replace(' · ', ' ·\n'), cx - 0.56, 2.7, 1.12, 0.62, { size: 12.5, font: FSB, color: part ? C.muted : hi ? C.accD : C.ink, align: 'center', ls: 1.05 }, { e: 'fade', d: d + 80, t: 0.35 });
  });
  const cx6 = x0 + 5 * dx;
  line(cx6, 3.38, cx6, 3.62, { color: C.acc, w: 1.5 }, { e: 'wipeD', d: 1700, t: 0.25 });
  txt('주방 조작 = 컵·물통 파지 · 붓기 · 물통 반환 · 상판 적재 (모방학습)', cx6 - 3.6, 3.62, 7.2, 0.5, { shape: RR, r: 0.25, fill: C.accT, line: 'F4B27A', lw: 1, size: 14.5, font: FSB, color: C.accD, align: 'center', valign: 'middle' }, { e: 'zoom', d: 1800, t: 0.45 });
  const lg = { e: 'fade', d: 1950, t: 0.4 };
  circle(ML + 0.08, 4.32, 0.16, { fill: C.main }, lg);
  txt('실물 검증 9단계', ML + 0.26, 4.19, 1.7, 0.28, { size: 12, font: FSB, color: C.ink }, lg);
  circle(ML + 2.1, 4.32, 0.16, { fill: 'A9BCCB' }, lg);
  txt('부분 검증 2단계 — 물 양 신호·충전 시작 신호 연동 남음', ML + 2.28, 4.19, 5.6, 0.28, { size: 12, color: C.muted }, lg);
  const my = 4.62, mh = 2.2, vw = mh * 16 / 9, vx = CR - vw, iw = CW - vw - 0.22;
  img(await rounded('restaurant.png', iw, mh, 0.14), ML, my, iw, mh, { alt: '식당 서빙 장면 이미지' }, { e: 'fade', d: 2100, t: 0.6 });
  L.S.slide.addMedia({ type: 'video', path: A('v7n.mp4'), cover: await dataUri('poster7n.jpg'), x: vx, y: my, w: vw, h: mh, objectName: 's5_video' });
  L.anim('s5_video', { e: 'fade', d: 2250, t: 0.6 });
  coverUntil(vx - 0.02, my - 0.02, vw + 0.04, mh + 0.04, 2250, { name: 's5_cover' });
  setVideo('s5_video', 10000, 2250);
  txt('3배속 편집 영상', 10.45, 4.36, CR - 10.45, 0.24, { size: 10.5, color: C.faint, align: 'right', check: false }, { e: 'fade', d: 2300, t: 0.4 });
  notes('출정식에서 목표로 잡았던 11단계를 기준으로 결과를 정리했습니다.\n파란색 9단계는 실물에서 검증을 마쳤고, 회색 두 단계는 기능은 있지만 신호 연동이 남은 부분 검증 단계입니다.\n물 양 확인은 판정 모델은 완성했지만 붓기 정지와의 직접 연결이 남았고, 충전은 도크 도착까지는 되지만 충전 시작 신호를 아직 받지 못합니다.\n가운데 주방 조작 다섯 동작은 모두 모방학습 정책 하나로 수행합니다.');
}

async function s6() {
  newSlide();
  header({ num: '04', label: '목표 달성 · 핵심 정량 지표 결과', title: '정량 지표로 본 최종 성능', orb: 'c' });
  txt('성공 판정 지표', ML, 1.95, 4, 0.36, { size: 16.5, bold: true, color: C.main }, { e: 'fade', d: 0, t: 0.4 });
  txt('시스템 성능 지표', 9.05, 1.95, 3.6, 0.36, { size: 16.5, bold: true, color: C.subD }, { e: 'fade', d: 900, t: 0.4 });
  // [임의] 아래 결과값은 모두 발표 전 실측으로 교체
  const cards = [['LuCircleCheck', 'phase 성공', '9/10 · 8/10 · 7/10', '물따르기 · 컵배치 · 전체 미션 (각 10회)'], ['LuDroplet', '물 양 오차', '±12 mL', '목표 150 mL 기준 평균 오차 · 흘림 1/10회'], ['LuHand', '사람 개입', '0.3회 / 주문', '10회 중 개입 3회 (컵 재정렬 2 · 재시작 1)'], ['LuTimer', 'cycle time', '6분 48초', '주문 접수 → 도크 복귀 평균 (시뮬 473.9초)']];
  const cw = 4.05, ch = 1.6, gx = 0.2, gy = 0.22;
  for (let i = 0; i < 4; i++) {
    const x = ML + (i % 2) * (cw + gx), y = 2.45 + Math.floor(i / 2) * (ch + gy), a = { e: 'zoom', d: 150 + i * 170, t: 0.45 };
    box(x, y, cw, ch, { r: 0.12, fill: C.white, line: C.line, shadow: 'soft' }, a);
    await iconDot(cards[i][0], x + 0.5, y + 0.5, 0.56, C.mainT, C.main, a);
    txt(cards[i][1], x + 0.95, y + 0.27, cw - 1.1, 0.46, { size: 18, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(cards[i][2], x + 0.3, y + 0.82, cw - 0.5, 0.42, { size: 22, bold: true, color: C.main, valign: 'middle', check: false }, { e: 'zoom', d: 350 + i * 170, t: 0.45 });
    txt(cards[i][3], x + 0.3, y + 1.22, cw - 0.45, 0.3, { size: 11.5, color: C.muted }, a);
  }
  const x = 9.05, y = 2.45, w = CR - x, h = 2 * ch + gy, a = { e: 'zoom', d: 1000, t: 0.5 };
  box(x, y, w, h, { r: 0.12, fill: C.subT, line: 'CDE6F6' }, a);
  await iconDot('LuActivity', x + 0.62, y + 0.62, 0.7, C.white, C.subD, a);
  txt('지연 · 지터', x + 0.32, y + 1.12, w - 0.6, 0.42, { size: 20, bold: true, color: C.ink }, a);
  txt('92 ms · ±4 ms', x + 0.32, y + 1.55, w - 0.6, 0.48, { size: 22, bold: true, color: C.subD, check: false }, { e: 'zoom', d: 1150, t: 0.45 });
  txt('정책 추론 지연 · 30 Hz 제어 지터\nπ0 원격 추론 7.4 s → 130 ms', x + 0.32, y + 2.05, w - 0.6, 0.6, { size: 12, color: C.muted, ls: 1.1 }, a);
  const pts = [0, 0.12, -0.08, 0.15, -0.12, 0.06, -0.04, 0.13, -0.1, 0.05];
  const bx = x + 0.32, by = y + 3.0, step = (w - 0.64) / (pts.length - 1);
  for (let i = 0; i < pts.length - 1; i++) line(bx + i * step, by - pts[i], bx + (i + 1) * step, by - pts[i + 1], { color: C.sub, w: 2 }, { e: 'wipeR', d: 1250 + i * 60, t: 0.12 });
  callout('결과', '전체 미션 성공률 70% · 물 양 오차는 허용오차 ±15 mL 이내 · 사람 개입은 주문당 1회 미만', 6.12, {}, { e: 'up', d: 1500, t: 0.5 });
  notes('출정식에서 정한 다섯 가지 정량 지표의 결과입니다.\n고정된 시작 위치에서 10회씩 시험했을 때 물따르기는 9회, 컵배치는 8회, 주문부터 복귀까지 전체 미션은 7회 성공했습니다.\n물 양은 150 mL 목표에 평균 12 mL 오차였고, 사람 개입은 주문당 0.3회였습니다.\n한 주문의 평균 소요 시간은 6분 48초였고, 정책 추론 지연은 92 ms로 30 Hz 제어 주기 안에서 동작했습니다.\n(이 쪽의 결과 수치는 발표 전 실측으로 교체할 자리표시 값)');
}

async function s7() {
  newSlide();
  header({ num: '05', label: '시스템 구조', title: '컴퓨터 3대가 역할을 나눠 로봇 한 대를 움직입니다', orb: 'c', sub: '무거운 AI 계산은 로봇 밖 노트북이, 가볍지만 빠른 판단·실행은 로봇 안의 컴퓨터가 나눠 맡습니다.' });
  const y = 2.32, h = 2.25, nw = 3.45, px = 5.2, ex = CR - nw;
  box(ML - 0.08, y - 0.3, nw + 0.16, h + 0.42, { r: 0.12, line: C.mainT3, lw: 1.25, dash: 'dash' }, { e: 'fade', d: 0, t: 0.4 });
  txt('로봇 밖', ML + 0.1, y - 0.42, 0.85, 0.26, { size: 11, bold: true, color: C.main, fill: C.white, align: 'center', valign: 'middle', check: false }, { e: 'fade', d: 0, t: 0.4 });
  box(px - 0.12, y - 0.3, CR - px + 0.2, h + 0.42, { r: 0.12, line: C.mainT3, lw: 1.25, dash: 'dash' }, { e: 'fade', d: 100, t: 0.4 });
  txt('로봇 안', px + 0.06, y - 0.42, 0.85, 0.26, { size: 11, bold: true, color: C.main, fill: C.white, align: 'center', valign: 'middle', check: false }, { e: 'fade', d: 100, t: 0.4 });
  const nodes = [
    [ML, 'LuLaptop', '노트북', '로봇의 대뇌', '웹 주문 화면을 띄웁니다\n학습한 AI로 잡기 · 붓기 동작을 계산합니다\n카메라 영상으로 컵 속 물 양을 봅니다', 'GPU 노트북'],
    [px, 'LuCpu', '라즈베리파이', '로봇의 소뇌', '주문을 정해진 순서대로 진행합니다\n지도를 보고 길을 찾아 이동합니다\n주차·도킹을 라이다·뎁스로 맞춥니다', 'ROS 2 · SLAM · Nav2'],
    [ex, 'LuMicrochip', 'ESP32 보드', '로봇의 말초신경', '받은 속도대로 바퀴를 돌립니다\n바퀴가 얼마나 굴렀는지 알려 줍니다\n배터리가 얼마나 남았는지 알려 줍니다', '초당 50번 주고받음'],
  ];
  for (let i = 0; i < 3; i++) {
    const [x, ic, nmz, meta, body, tag] = nodes[i], a = { e: 'up', d: 200 + i * 200, t: 0.45 };
    box(x, y, nw, h, { r: 0.12, fill: i === 0 ? C.white : C.mainT, line: i === 0 ? C.line : undefined, shadow: i === 0 ? 'soft' : undefined }, a);
    await iconDot(ic, x + 0.45, y + 0.42, 0.52, i === 0 ? C.mainT : C.white, C.main, a);
    txt(nmz, x + 0.82, y + 0.2, nw - 0.95, 0.42, { size: 19, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(meta, x + 0.82, y + 0.6, nw - 0.95, 0.26, { size: 12, bold: true, color: C.main }, a);
    txt(body.split('\n').map((t, j, arr) => ({ text: t, options: { bullet: { indent: 10 }, breakLine: j < arr.length - 1 } })), x + 0.2, y + 0.98, nw - 0.3, 0.86, { size: 11.5, color: C.text, ls: 1.08, check: false }, a);
    txt(tag, x + 0.2, y + h - 0.4, Math.min(nw - 0.4, L.tw(tag, 11, 'sb') + 0.36), 0.28, { shape: RR, r: 0.14, fill: i === 0 ? C.panel : C.white, color: C.mainD, font: FSB, size: 11, align: 'center', valign: 'middle', check: false }, a);
  }
  const ly = y + h / 2;
  const links = [[ML + nw, px, '무선'], [px + nw, ex, '유선']];
  links.forEach((lk, i) => {
    const [a1, a2, lab] = lk, d = 800 + i * 200;
    line(a1 + 0.05, ly, a2 - 0.05, ly, { color: C.main, w: 2, ba: 'triangle', ea: 'triangle' }, { e: 'wipeR', d, t: 0.4 });
    txt(lab, (a1 + a2) / 2 - 0.4, ly - 0.42, 0.8, 0.26, { size: 11.5, bold: true, color: C.main, align: 'center', check: false }, { e: 'fade', d, t: 0.4 });
    const span = a2 - a1 - 0.5, per = i === 0 ? 1.4 : 0.7;
    circle(a1 + 0.25, ly, 0.13, { fill: C.sub, name: `s7_pk${i}a` }, [{ e: 'fade', d: d + 300, t: 0.2 }, { e: 'path', d: d + 300, t: per, dx: span / 13.333, dy: 0, repeat: 60 }]);
    circle(a2 - 0.25, ly, 0.13, { fill: C.acc, name: `s7_pk${i}b` }, [{ e: 'fade', d: d + 300 + per * 500, t: 0.2 }, { e: 'path', d: d + 300 + per * 500, t: per, dx: -span / 13.333, dy: 0, repeat: 60 }]);
  });
  txt('예) 3번 테이블 냉수 주문 한 건 — 주방에서 물을 따른 뒤 테이블로 출발합니다', ML, 4.78, CW, 0.3, { size: 13, bold: true, color: C.ink }, { e: 'fade', d: 1300, t: 0.4 });
  const steps = [['주문 접수', '노트북 (웹 화면)'], ['주방으로 이동', '파이 (길 안내)\nESP32 (바퀴 회전)'], ['주방에서 물 따르기', '노트북 (정책 추론 → 양팔)\n→ 노트북 (물 양 확인)'], ['테이블로 출발', '파이 · ESP32'], ['컵 놓고 복귀', '노트북 (컵 배치 정책)\n→ 파이·ESP32 (복귀)']];
  const sw = (CW - 4 * 0.12) / 5;
  steps.forEach((s, i) => {
    const x = ML + i * (sw + 0.12), a = { e: 'fade', d: 1400 + i * 130, t: 0.4 }, hi = i === 2;
    box(x, 5.15, sw, 0.97, { r: 0.1, fill: hi ? C.mainT : C.panel, line: hi ? C.main : undefined }, a);
    txt([{ text: `${i + 1}  `, options: { color: C.main } }, { text: s[0], options: { color: C.ink } }], x + 0.14, 5.2, sw - 0.2, 0.28, { size: 12.5, bold: true, check: false }, a);
    txt(s[1], x + 0.14, 5.47, sw - 0.2, 0.6, { size: 11, color: C.text, ls: 1.0 }, a);
  });
  callout('왜 이렇게 나눴나', '각자 잘하는 일을 맡깁니다.   무거운 AI 계산 → GPU 노트북  ·  순서와 길 찾기 → 파이  ·  매 순간 정확한 바퀴 회전 → ESP32', 6.3, { h: 0.56, size: 12.5 }, { e: 'up', d: 2100, t: 0.45 });
  notes('전체 시스템 구조입니다.\n무거운 AI 계산은 로봇 밖 GPU 노트북이, 순서와 길 찾기는 라즈베리파이가, 바퀴 회전처럼 매 순간 정확해야 하는 일은 ESP32가 맡습니다.\n양팔 정책 추론은 노트북에서 로컬로 실행해 무선 지연이 조작에 영향을 주지 않도록 했습니다.');
}

async function s8() {
  newSlide();
  header({ num: '06', label: '팀 · 팀 구성', title: '4개의 파트가 하나의 Action 계약에서 만난다', orb: 'e' });
  const cw = 4.95, ch = 2.2, xs = [ML, CR - cw], ys = [1.95, 4.5];
  const parts = [
    ['LuRoute', '이동 · 연결', '임주영', 'SLAM/Nav2 · 1 cm 정밀주차 · 도크 복귀 · phase 연결 · Isaac Sim 모델·검증', '도착·정렬 결과, 조작 Action 호출 계약'],
    ['LuHand', '조작 정책', '장준서', '양팔 텔레옵 수집 · SmolVLA/π0/ACT/GR00T 학습 · 실물 롤아웃', 'task별 정책 checkpoint · 롤아웃 래퍼'],
    ['LuWrench', '기구 · 하드웨어', '공통', '4WD 차체 · 상판 · 그리퍼 · 배선 · 전원', '실측 치수 · 장착 좌표'],
    ['LuScanEye', '인지 · 물 양', '지민석', '컵·액면 인식 · 물 양 판정 · 목표량 도달 신호', '컵·물통 위치 · 물 양 판정'],
  ];
  const hx = 6.667, hy = 4.35;
  circle(hx, hy, 2.55, { line: C.mainT3, lw: 1.5, dash: 'dash' }, [{ e: 'zoom', d: 0, t: 0.5 }, { e: 'pulse', d: 1600, t: 0.6, size: 108, autoRev: true, repeat: 2 }]);
  txt('ROS2\nAction 계약', hx - 1.0, hy - 1.0, 2.0, 2.0, { shape: pres.shapes.OVAL, fill: C.ink, color: C.white, bold: true, size: 17, align: 'center', valign: 'middle', ls: 1.1, check: false }, { e: 'grow', d: 100, t: 0.45 });
  for (let i = 0; i < 4; i++) {
    const [ic, part, who, desc, hand] = parts[i];
    const x = xs[i % 2], y = ys[Math.floor(i / 2)], a = { e: 'zoom', d: 450 + i * 170, t: 0.45 };
    box(x, y, cw, ch, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft', name: `!!q${i + 1}` }, a);
    await iconDot(ic, x + 0.5, y + 0.48, 0.56, C.mainT, C.main, a);
    txt(part, x + 0.9, y + 0.3, 2.6, 0.36, { size: 15, bold: true, color: C.main, valign: 'middle' }, a);
    txt(who, x + cw - 1.6, y + 0.27, 1.35, 0.42, { size: 21, bold: true, color: C.ink, align: 'right', valign: 'middle' }, a);
    txt(desc, x + 0.3, y + 0.86, cw - 0.6, 0.6, { size: 13, color: C.muted, ls: 1.1 }, a);
    txt([{ text: '넘겨주는 것  ', options: { bold: true, color: C.main } }, { text: hand, options: { color: C.ink } }], x + 0.2, y + ch - 0.6, cw - 0.4, 0.42, { fill: C.mainT, size: 12.5, font: FSB, valign: 'middle', margin: 5 }, a);
  }
  notes('팀 구성입니다. 네 개의 파트가 하나의 ROS 2 Action 계약에서 만납니다.\n이동과 연결은 임주영 님, 조작 정책은 제가, 인지와 물 양 판정은 지민석 님이 맡았고 하드웨어는 공동으로 작업했습니다.\n지금부터 파트별 결과를 말씀드리겠습니다.');
}

module.exports = [s1, s2, s3, s4, s5, s6, s7, s8];
