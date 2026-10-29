'use strict';
// MS팀 최종 발표 slides 9–15: 파트별 수행 결과 (이동 · 조작 4쪽 · 인지 · 기구)
// ※ [임의] 표시가 붙은 수치는 실험 전 자리표시 값이다(발표 전 실측으로 교체).
const L = require('motion-deck');
const { pres, C, FR, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, rounded, contain, header, callout, newSlide, profile, sec, chevrons } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;
const notes = (t) => L.S.slide.addNotes(t);

async function s9() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 이동·연결', title: '식당 목표점 주행부터 1 cm 정밀주차·도크 복귀까지', orb: 'a', mini: 1 });
  const px = ML, pw = 4.3, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 4.9, '임주영', '이동 · 연결', a0);
  sec('보유 기술·이력', 'ROS 2 · Nav2 · LiDAR SLAM\n지도 생성·실차 주행·데이터 분석', px + 0.3, 2.95, pw - 0.5, 0.85, { e: 'up', d: 150, t: 0.4 });
  sec('수행 내용', 'Cartographer SLAM 지도 · AMCL 위치 추정\nNav2 NavFn 경로 · RPP 추종 · Keepout\nCollision Monitor 정지 · 같은 목표 재개\n4WD 스키드스티어 전환 · 후진·회전 접근\nRGB-D 면 정렬 책상 접근 · 1 cm 주차\n후면 LiDAR 박스 관측 도크 복귀', px + 0.3, 3.95, pw - 0.5, 1.95, { e: 'up', d: 300, t: 0.4 }, { size: 12.5, ls: 1.12 });
  sec('핵심 결과', '1 cm 목표 → 실측 12 mm (좌우)\ntable_02 왕복 186.5초 · Nav2 목표 10/10', px + 0.3, 6.0, pw - 0.5, 0.8, { e: 'up', d: 450, t: 0.4 }, { size: 12.5 });
  const ix = 5.2, iw = CR - ix, ih = iw / (1118 / 460);
  img(await rounded('slam.png', iw, ih, 0.14), ix, 1.95, iw, ih, { alt: '실차 SLAM 장애물 대응 관제 화면' }, { e: 'zoom', d: 300, t: 0.6 });
  txt([{ text: '미등록 장애물 우회·긴급 정지·주행 재개', options: { bold: true, color: C.ink, fontSize: 15 } }, { text: '   LiDAR costmap 갱신 → NavFn 경로 재계획', options: { color: C.muted, fontSize: 12.5 } }], ix, 1.95 + ih + 0.12, iw, 0.36, { valign: 'middle', check: false }, { e: 'fade', d: 700, t: 0.4 });
  chevrons(['01\nSLAM 지도 생성', '02\n목표점 설정', '03\n장애물 우회', '04\n1 cm 정밀주차', '05\n도크 복귀'], ix, 5.72, iw, 0.9, 850, { size: 11 });
  notes('임주영 님의 이동·연결 파트 결과입니다.\nCartographer로 식당 공간 지도를 만들고 AMCL로 위치를 추정하며, Nav2로 경로를 계획하고 추종합니다.\n출정식 이후 차체를 4륜 스키드스티어로 바꿨고, 뎁스 카메라로 책상 면을 정렬해 1 cm 목표 정밀주차를 구현했습니다. 실측 간격은 좌우 모두 12 mm였습니다.\n도크 복귀는 후면 라이다로 박스를 관측해 맞추며, 2번 테이블 왕복은 186.5초에 Nav2 목표 10개를 모두 통과했습니다.');
}

async function s10() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 조작 정책', title: '차체 결합 데이터 80+80개로 물따르기·컵배치 정책 학습', orb: 'c', mini: 2 });
  const px = ML, pw = 4.55, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 3.9, '장준서', '조작 정책', a0);
  txt('수행 내역', px + 0.3, 2.92, pw - 0.5, 0.28, { size: 12, bold: true, color: C.main }, { e: 'up', d: 150, t: 0.4 });
  const B = (t, lvl) => ({ text: t, options: { bullet: lvl ? { code: '2013', indent: 12 } : { indent: 12 }, indentLevel: lvl || 0, breakLine: true, fontSize: lvl ? 12 : 13, bold: !lvl, color: lvl ? C.text : C.ink, paraSpaceAfter: lvl ? 1 : 3 } });
  const items = [B('차체 결합 텔레옵 80+80 에피소드 (19.4만 프레임)'), B('20⊂40⊂60⊂80 · 같은 검증 6개로 공정 비교', 1), B('SmolVLA · π0 · ACT · GR00T 학습', 1), B('π0 4B OOM → 행동 전문가 578M만 학습', 1), B('검증된 설정 + step 확장 · 이미지 증강', 1), B('롤아웃 래퍼: 과부하 예방·자동 종료·붓기 훅', 1), B('실패 원인 규명: 카메라 오정렬 · 시작 자세 OOD')];
  delete items[items.length - 1].options.breakLine;
  txt(items, px + 0.25, 3.24, pw - 0.4, 2.55, { size: 12.5, ls: 1.1, check: false }, { e: 'up', d: 250, t: 0.45 });
  const tx = 5.45, tw2 = CR - tx;
  const hdr = ['실행', '최저 검증 손실', '스텝', '결과'].map((t) => ({ text: t, options: { bold: true, color: C.muted, fill: { color: 'E8F3FA' }, fontSize: 11.5 } }));
  // π0·ACT·GR00T 행은 [임의] (학습 진행 전·중)
  const data = [['SmolVLA 60 · 물따르기', '0.0608', '12500', '끝까지 학습'], ['SmolVLA 80 · 물따르기', '0.0532', '16000', '최종 채택'], ['SmolVLA 60 · 컵배치', '0.1319', '7000', '조기 종료'], ['SmolVLA 80 · 컵배치', '0.1122', '7000', '최종 채택'], ['π0 80 · 물따르기', '0.0447', '9000', '비교 후 미채택'], ['π0 80 · 컵배치', '0.0981', '5500', '비교 후 미채택'], ['ACT 80 · 물따르기', '0.412 (L1+KL)', '19500', '비교 후 미채택'], ['GR00T N1.7 80 · 물따르기', '0.0489', '11000', '비교 후 미채택']];
  const rows = [hdr].concat(data.map((r) => r.map((c, j) => {
    const win = r[3] === '최종 채택';
    return { text: c, options: { bold: win || j === 1, color: win ? (j === 3 ? C.main : C.ink) : j === 3 ? '4A6378' : C.text, fill: { color: win ? C.mainT : C.white }, fontSize: 11.5 } };
  })));
  L.S.slide.addTable(rows, { x: tx, y: 1.95, w: tw2, colW: [2.55, 1.45, 0.9, tw2 - 4.9], rowH: 0.31, fontFace: FR, valign: 'middle', margin: [0.04, 0.1, 0.04, 0.1], border: { type: 'solid', pt: 0.75, color: 'DCEBF5' }, objectName: 's10_table' });
  L.anim('s10_table', { e: 'wipeD', d: 300, t: 0.8 });
  const stats = [['80+80', '개', 'task별 에피소드', 1.75], ['−12.5 · −15', '%', '검증 손실 (60→80개, 두 task)', 2.85], ['9/10', '회', 'SmolVLA-80 물따르기 실물 성공', tw2 - 4.8]];
  let sx = tx;
  stats.forEach((s, i) => {
    const a = { e: 'zoom', d: 900 + i * 160, t: 0.45 };
    box(sx, 4.93, s[3], 0.92, { r: 0.12, fill: i === 2 ? C.main : C.mainT }, a);
    txt([{ text: s[0], options: { fontSize: 24, bold: true } }, { text: s[1], options: { fontSize: 14, bold: true } }], sx + 0.18, 4.97, s[3] - 0.3, 0.5, { color: i === 2 ? C.white : C.main, valign: 'middle', check: false }, a);
    txt(s[2], sx + 0.18, 5.45, s[3] - 0.24, 0.3, { size: 11, color: i === 2 ? 'E3F4FD' : C.muted }, a);
    sx += s[3] + 0.1;
  });
  callout('결론', '두 task 모두 SmolVLA-80 채택 — 데이터 확대가 검증 손실과 실물 성공률을 함께 개선했다', 6.15, { h: 0.68, size: 13.5 }, { e: 'up', d: 1450, t: 0.5 });
  notes('다음은 제가 맡은 조작 정책입니다.\n양팔을 차체에 결합한 뒤 물따르기와 컵배치 각각 80개씩 시연 데이터를 새로 모았습니다.\n같은 검증 에피소드 6개로 데이터 양의 효과를 공정하게 비교했고, SmolVLA·π0·ACT·GR00T 네 모델을 학습했습니다.\nπ0는 4B 전체 학습이 24GB GPU에서 메모리가 부족해, 사전학습 가중치에서 행동 전문가만 학습하는 방식으로 바꿨습니다.\n모델별 손실은 계산 방식이 달라 직접 비교하지 않고, 실물 롤아웃 결과로 SmolVLA-80을 두 task 모두 최종 채택했습니다.\n(π0·ACT·GR00T 행과 실물 성공 횟수는 발표 전 실측으로 교체할 값)');
}

// 학습 로그(실측) — 500 step마다 검증 손실, 1000 step부터 표시
const EV = {
  pour60: [[1000, .2253], [1500, .16], [2000, .1478], [2500, .1221], [3000, .1134], [3500, .1155], [4000, .1], [4500, .0899], [5000, .0901], [5500, .0857], [6000, .0759], [6500, .0733], [7000, .07], [7500, .0678], [8000, .0657], [8500, .0649], [9000, .0636], [9500, .0642], [10000, .062], [10500, .0614], [11000, .0613], [11500, .0613], [12000, .0612], [12500, .0608], [13000, .0608]],
  pour80: [[1000, .2502], [1500, .1814], [2000, .148], [2500, .1299], [3000, .1105], [3500, .1111], [4000, .1015], [4500, .1006], [5000, .0893], [5500, .0907], [6000, .0818], [6500, .0771], [7000, .0759], [7500, .0768], [8000, .0688], [8500, .064], [9000, .0636], [9500, .0627], [10000, .0587], [10500, .0588], [11000, .0587], [11500, .0557], [12000, .0567], [12500, .0556], [13000, .0546], [13500, .0541], [14000, .0545], [14500, .0533], [15000, .0544], [15500, .0538], [16000, .0532]],
  place60: [[1000, .2514], [1500, .2177], [2000, .2009], [2500, .1866], [3000, .1723], [3500, .1447], [4000, .1408], [4500, .1366], [5000, .1416], [5500, .1411], [6000, .1353], [6500, .1382], [7000, .1319], [7500, .1386], [8000, .1407], [8500, .1355]],
  place80: [[1000, .2973], [1500, .2188], [2000, .1998], [2500, .166], [3000, .1702], [3500, .1374], [4000, .1459], [4500, .1308], [5000, .1274], [5500, .1315], [6000, .1236], [6500, .129], [7000, .1122], [7500, .1365], [8000, .122], [8500, .1186]],
};

// 선 그리기 차트: 축·눈금은 바로, 두 곡선은 왼→오로 그려지고 최적점이 커진다
function lossPlot(x, y, w, h, title, keys, xmax, xstep, ymax, ystep, d0) {
  const pl = x + 0.55, pr = x + w - 0.15, pt = y + 0.75, pb = y + h - 0.45;
  const X = (v) => pl + (v / xmax) * (pr - pl), Y = (v) => pb - (v / ymax) * (pb - pt);
  const a0 = { e: 'fade', d: d0, t: 0.4 };
  txt(title, x + 0.25, y + 0.15, w - 0.5, 0.36, { size: 14.5, bold: true, color: C.ink, valign: 'middle', check: false }, a0);
  for (let v = 0; v <= ymax + 1e-9; v += ystep) {
    line(pl, Y(v), pr, Y(v), { color: v === 0 ? 'B9CCDA' : 'E6F0F7', w: v === 0 ? 1.25 : 0.75 }, a0);
    txt(v.toFixed(2), x + 0.05, Y(v) - 0.12, 0.45, 0.24, { size: 9.5, color: C.faint, align: 'right', check: false }, a0);
  }
  for (let v = 0; v <= xmax; v += xstep) txt(v === 0 ? '0' : `${v / 1000}k`, X(v) - 0.3, pb + 0.05, 0.6, 0.24, { size: 9.5, color: C.faint, align: 'center', check: false }, a0);
  // legend
  const lg = [['60개', 'A9BCCB'], ['80개 (채택)', C.main]];
  let lx = x + w - 2.55;
  lg.forEach(([t, col]) => {
    line(lx, y + 0.56, lx + 0.32, y + 0.56, { color: col, w: 2.5 }, a0);
    txt(t, lx + 0.38, y + 0.45, 1.0, 0.22, { size: 10.5, color: C.muted, check: false }, a0);
    lx += 1.15;
  });
  const draw = 1.6;
  keys.forEach((k, si) => {
    const pts = EV[k], col = si === 0 ? 'A9BCCB' : C.main, dS = d0 + 300 + si * 250;
    for (let i = 0; i < pts.length - 1; i++) {
      line(X(pts[i][0]), Y(pts[i][1]), X(pts[i + 1][0]), Y(pts[i + 1][1]), { color: col, w: si ? 2.5 : 2 }, { e: 'wipeR', d: dS + Math.round((i / (pts.length - 1)) * draw * 1000), t: draw / (pts.length - 1) + 0.02 });
    }
    const best = pts.reduce((m, p) => (p[1] < m[1] ? p : m), pts[0]);
    const dB = dS + draw * 1000 + 100;
    circle(X(best[0]), Y(best[1]), 0.16, { fill: col, line: C.white, lw: 1.5 }, { e: 'grow', d: dB, t: 0.3 });
    txt(best[1].toFixed(4), X(best[0]) - 0.7, Y(best[1]) + (si ? 0.1 : -0.36), 0.75, 0.26, { size: 11, bold: true, color: si ? C.mainD : C.muted, align: 'right', check: false }, { e: 'fade', d: dB + 50, t: 0.3 });
  });
}

async function s11() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 조작 정책 학습 곡선', title: '데이터를 60개 → 80개로 늘리자 두 task 모두 검증 손실 감소', orb: 'b', mini: 2 });
  const w = (CW - 0.25) / 2;
  // 카드 두 장을 먼저 그려야 왼쪽 그래프의 선이 오른쪽 카드에 가리지 않는다
  box(ML, 1.95, w, 3.75, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, { e: 'fade', d: 0, t: 0.4 });
  box(ML + w + 0.25, 1.95, w, 3.75, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, { e: 'fade', d: 300, t: 0.4 });
  lossPlot(ML, 1.95, w, 3.75, '물따르기 — 0.0608 → 0.0532 (−12.5%)', ['pour60', 'pour80'], 17000, 2000, 0.25, 0.05, 0);
  lossPlot(ML + w + 0.25, 1.95, w, 3.75, '컵배치 — 0.1319 → 0.1122 (−15%)', ['place60', 'place80'], 9000, 1000, 0.30, 0.05, 300);
  txt('* 같은 검증 에피소드 6개 · 500 step마다 평가 · 3회 연속 미개선 시 조기 종료 · 가로축 = 학습 step · 점 = 최적 체크포인트', ML, 5.76, CW, 0.28, { size: 10.5, color: C.faint }, { e: 'fade', d: 2400, t: 0.4 });
  callout('해석', '물따르기는 길게 학습할수록 계속 개선 · 컵배치는 4,500 step 이후 ±0.01 안에서 진동 — 두 구간 모두 80개가 아래', 6.15, { size: 13.5 }, { e: 'up', d: 2600, t: 0.5 });
  notes('60개와 80개 모델의 검증 손실 곡선입니다. 실제 학습 로그 값입니다.\n왼쪽 물따르기는 데이터를 늘리자 최저 손실이 0.0608에서 0.0532로 12.5% 낮아졌고, 학습을 길게 할수록 꾸준히 좋아졌습니다.\n오른쪽 컵배치는 0.1319에서 0.1122로 15% 낮아졌습니다. 4,500 step 이후로는 작은 폭으로 오르내렸지만 그 구간에서도 80개 모델이 거의 항상 아래에 있었습니다.\n두 모델 모두 같은 검증 에피소드로 평가했기 때문에, 이 차이는 데이터 양의 효과로 볼 수 있습니다.');
}

async function s12() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 조작 구성', title: '전 구간 모방학습에 규칙 기반 안전장치를 더해 최종 구성', orb: 'd', mini: 2 });
  const cards = [['IK_ALL', '산업형 기준선 (보류)', '실물 상대 IK로 컵 상승까지 확인, 좌표·보정 오차로 전 구간 적용 보류', C.ink], ['IL_ALL', '전 구간 SmolVLA', '물따르기·컵배치 정책 2개로 모든 조작 phase 수행', C.subD], ['HYBRID', '최종 채택: IL + 안전 래퍼', '붓기 훅 · 시작/끝 자세 · 작업 끝 자동 감지 · 과부하 예방', C.main]];
  const w = 3.9, g = (CW - 3 * w) / 2, y = 2.1, h = 2.1;
  cards.forEach((c, i) => {
    const x = ML + i * (w + g), hy = i === 2, a = { e: 'up', d: i * 180, t: 0.45 };
    box(x, y, w, h, { r: 0.14, fill: hy ? C.mainT : C.white, line: hy ? C.main : C.line, lw: hy ? 2 : 1, shadow: hy ? undefined : 'soft' }, a);
    txt(c[0], x + 0.3, y + 0.25, w - 0.6, 0.5, { size: 24, bold: true, color: c[3] }, a);
    txt(c[1], x + 0.3, y + 0.8, w - 0.6, 0.38, { size: 16.5, bold: true, color: C.ink }, a);
    txt(c[2], x + 0.3, y + 1.22, w - 0.55, 0.75, { size: 13, color: C.muted, ls: 1.12 }, a);
  });
  const hx = ML + 2 * (w + g) + w / 2, busY = 4.6, py = 4.95;
  line(hx, y + h, hx, busY, { color: C.main, w: 2 }, { e: 'wipeD', d: 700, t: 0.3 });
  const ph = [['CUP_PICK', 'smolVLA'], ['BOTTLE_PICK', 'smolVLA'], ['POUR', 'smolVLA+훅'], ['BOTTLE_RET', 'smolVLA'], ['SHELF_PLACE', 'smolVLA'], ['TABLE_PLACE', 'smolVLA']];
  const pw = 1.9, pg = (CW - 6 * pw) / 5;
  const cxs = ph.map((_, i) => ML + i * (pw + pg) + pw / 2);
  line(cxs[0], busY, hx, busY, { color: C.main, w: 2 }, { e: 'wipeL', d: 950, t: 0.6 });
  line(hx, busY, cxs[5], busY, { color: C.main, w: 2 }, { e: 'wipeR', d: 950, t: 0.3 });
  ph.forEach((p, i) => {
    const x = ML + i * (pw + pg), pour = i === 2, d = 1250 + i * 110;
    line(cxs[i], busY, cxs[i], py, { color: C.main, w: 2 }, { e: 'wipeD', d, t: 0.2 });
    box(x, py, pw, 1.35, { r: 0.12, fill: pour ? C.accT : C.white, line: pour ? C.acc : C.line, lw: pour ? 2 : 1 }, [{ e: 'grow', d: d + 100, t: 0.35 }].concat(pour ? [{ e: 'pulse', d: 2300, t: 0.4, size: 106, autoRev: true, repeat: 2 }] : []));
    txt(String(i + 1).padStart(2, '0'), x + 0.2, py + 0.14, 0.6, 0.3, { size: 12, bold: true, color: pour ? C.accD : C.main }, { e: 'grow', d: d + 100, t: 0.35 });
    txt(p[0], x + 0.15, py + 0.44, pw - 0.3, 0.34, { size: 13, bold: true, color: C.ink, align: 'center', valign: 'middle', check: false }, { e: 'grow', d: d + 100, t: 0.35 });
    txt(p[1], x + 0.3, py + 0.86, pw - 0.6, 0.32, { shape: RR, r: 0.16, fill: pour ? C.acc : C.ink, color: C.white, bold: true, size: 11.5, align: 'center', valign: 'middle', check: false }, { e: 'grow', d: d + 100, t: 0.35 });
    if (L.tw(p[0], 13, 'b') > pw - 0.3) L.WARN.push('s12 phase name too long ' + p[0]);
  });
  notes('출정식에서는 phase마다 계획 기반 IK와 학습 정책을 비교해 고르겠다고 말씀드렸습니다.\n실험 결과 IK는 실물에서 컵을 들어 올리는 단계까지 확인했지만, 좌표와 보정 오차 때문에 전 구간 적용은 보류했습니다.\n대신 모든 조작 phase를 모방학습 정책으로 수행하고, 정책이 약한 지점에만 규칙 기반 안전장치를 붙였습니다.\n예를 들어 붓기는 손목이 75도를 넘으면 코드가 기울이기와 복귀를 맡는 붓기 훅을 쓰고, 시작 자세 맞추기와 작업 끝 자동 감지, 그리퍼 과부하 예방을 래퍼로 더했습니다.');
}

async function s13() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 조작 문제 해결', title: '실물에서 막힌 지점을 원인부터 찾아 고쳤다', orb: 'e', mini: 2 });
  const xD = ML, wD = 1.5, xP = 2.25, wP = 4.85, xA = 7.17, xS = 7.62, wS = CR - 7.62;
  const ha = { e: 'fade', d: 0, t: 0.4 };
  txt('분야', xD, 1.95, wD, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('증상 → 확인된 원인', xP, 1.95, wP, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('조치 → 결과', xS, 1.95, wS, 0.28, { size: 12, bold: true, color: C.main }, ha);
  const rows = [
    ['LuActivity', '떨림', '제자리 떨림 → 손목캠 좌우 뒤바뀜·시작 자세 OOD', 'USB 경로로 카메라 자동 탐색 · 학습 분포 시작 자세 → 정상 수행'],
    ['LuGlassWater', '붓기', '90°에서 망설임 → 자갈 모습에 의존(causal confusion)', '75° 이후 코드가 붓기를 맡는 붓기 훅 → 끝까지 붓기'],
    ['LuHand', '그리퍼', '대기 중 과부하 → 다음 실행 연결 실패(ID 6)', '목표를 실제 위치 쪽으로 당겨 미는 힘 제거 → 연속 실행'],
    ['LuCamera', '카메라', '조명 줄무늬 23 → 노출 2.5 ms < 깜빡임 8.3 ms', '노출 스윕 분석 후 학습과 같은 기종 유지 → 줄무늬 0.15'],
    ['LuScanLine', '정렬', '오른팔이 화면 밖으로 잘림 → top 카메라 (+16.8, +17.6)px', '학습 화면 겹쳐 보기 · 픽셀 측정 뷰어로 재정렬'],
    ['LuDatabase', '데이터', '64번 이후 에피소드 누락 → 시각화 도구 메타 64개 한계', '메타 재구성 · 병합 인덱스 오류 수정 → 80+80개 정리'],
  ];
  for (let i = 0; i < rows.length; i++) {
    const [ic, dom, p, s] = rows[i], y = 2.32 + i * 0.74, h = 0.64, d = 200 + i * 200;
    const a = { e: 'fade', d, t: 0.35 };
    box(xD, y, wD, h, { r: 0.1, fill: C.mainD }, a);
    await iconImg(ic, C.white, xD + 0.18, y + 0.19, 0.26, a);
    txt(dom, xD + 0.52, y, wD - 0.6, h, { size: 14, bold: true, color: C.white, valign: 'middle' }, a);
    txt(p, xP, y, wP, h, { fill: C.panel, size: 13, color: C.text, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 80, t: 0.4 });
    await iconImg('LuArrowRight', C.main, xA, y + 0.18, 0.28, { e: 'fade', d: d + 200, t: 0.25 });
    txt(s, xS, y, wS, h, { fill: C.mainT, size: 13, font: FSB, color: C.ink, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 260, t: 0.4 });
  }
  notes('조작 파트에서 실제로 막혔던 지점과 해결 과정입니다. 모두 원인을 데이터와 측정으로 확인한 뒤 고쳤습니다.\n가장 오래 걸린 건 제자리 떨림이었습니다. 카메라 좌우가 뒤바뀐 문제와, 사람이 맞춘 시작 자세가 학습 범위를 벗어난 문제 두 가지가 겹쳐 있었습니다.\n붓기에서 90도 근처에서 망설이는 현상은, 통제 실험으로 정책이 병 속 자갈이 빠지는 모습을 신호로 삼고 있다는 것을 확인했고, 붓기 훅으로 해결했습니다.\n차체 결합 후에는 조명 깜빡임 줄무늬와 카메라 각도 어긋남을 측정 도구로 수치화해서 학습 때와 같은 화면을 맞췄습니다.');
}

async function s14() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 인지·물 양', title: '물 양을 판정해 목표량 도달 신호를 보낸다', orb: 'b', mini: 4 });
  const px = ML, pw = 5.75, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 4.9, '지민석', '인지 · 물 양', a0);
  txt([{ text: '보유 기술·이력  ', options: { bold: true, color: C.main, fontSize: 12 } }, { text: '컴퓨터 비전 개발 경력', options: { color: C.text, fontSize: 13 } }], px + 0.3, 2.82, pw - 0.5, 0.3, { valign: 'middle', check: false }, { e: 'up', d: 120, t: 0.4 });
  txt('구현 방법', px + 0.3, 3.24, pw - 0.5, 0.28, { size: 12, bold: true, color: C.main }, { e: 'up', d: 220, t: 0.4 });
  const meth = [
    ['1', 'YOLOE로 컵 위치 인식 + 파인튜닝한 segmentation으로 액체 영역 분리.'],
    ['2', '높이 충전율: (바닥 원 중심 − 수위선 중심) ÷ (바닥 원 중심 − 입구 원 중심). 컵 높이 중 액체가 차 있는 비율.'],
    ['3', '부피 충전율: 높이마다 컵 너비²(단면 넓이)를 바닥부터 더해서, 수위선까지의 합 ÷ 입구까지의 합으로 계산.'],
  ];
  let my = 3.56;
  meth.forEach((m, i) => {
    const a = { e: 'up', d: 320 + i * 150, t: 0.4 }, hh = (L.nLines(m[1], 12.5, pw - 0.97) * 12.5 * 1.12) / 72 + 0.06;
    txt(m[0], px + 0.3, my + 0.02, 0.28, 0.28, { shape: pres.shapes.OVAL, fill: C.main, color: C.white, bold: true, size: 11, align: 'center', valign: 'middle', check: false }, a);
    txt(m[1], px + 0.7, my - 0.02, pw - 0.95, hh, { size: 12.5, color: C.text, ls: 1.12 }, a);
    my += hh + 0.12;
  });
  // [임의] 검증 결과 수치
  sec('검증 결과', '액면 높이 오차 평균 2.8 mm · 부피 환산 오차 ±11 mL (n=30)\n부족/도달/초과 판정 정확도 93% · 투명 컵 조건 81%', px + 0.3, 5.95, pw - 0.5, 0.85, { e: 'up', d: 800, t: 0.4 }, { size: 12.5 });
  const ix = 6.6, iw = CR - ix, ih = iw / (1116 / 505);
  img(await rounded('cupseg.png', iw, ih, 0.14), ix, 1.95, iw, ih, { alt: '컵 액면 분할과 부피 충전율 화면' }, { e: 'zoom', d: 300, t: 0.6 });
  chevrons(['YOLO\nsegmentation', '원근 보정', '액면 높이\n비율', '컵별 보정표', '판정'], ix, 1.95 + ih + 0.12, iw, 0.56, 600, { size: 11 });
  const gy = 5.68, gx = ix, gw = iw, a = { e: 'fade', d: 1450, t: 0.4 };
  txt([{ text: '판정  ', options: { bold: true, color: C.main, fontSize: 12 } }, { text: '고른 기준(부피 또는 높이)의 값이 목표 − 허용오차보다 작으면 부족. 목표 + 허용오차보다 크면 초과, 그 사이면 목표 도달.', options: { fontSize: 11.5, color: C.text } }], gx, gy - 0.08, gw, 0.5, { ls: 1.1, check: false }, a);
  const segs = [['부족', 0.36, 'F6D5B8', C.accD], ['목표 도달', 0.28, C.main, C.white], ['초과', 0.36, 'F6D5B8', C.accD]];
  let sx = gx;
  segs.forEach((s, i) => {
    const sw = gw * s[1];
    txt(s[0], sx, gy + 0.5, sw, 0.36, { fill: s[2], color: s[3], bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, { e: 'wipeR', d: 1550 + i * 180, t: 0.35 });
    sx += sw;
  });
  txt('목표 − 허용오차', gx + gw * 0.36 - 0.9, gy + 0.9, 1.8, 0.26, { size: 10.5, color: C.muted, align: 'center' }, { e: 'fade', d: 2100, t: 0.3 });
  txt('목표 + 허용오차', gx + gw * 0.64 - 0.9, gy + 0.9, 1.8, 0.26, { size: 10.5, color: C.muted, align: 'center' }, { e: 'fade', d: 2100, t: 0.3 });
  notes('지민석 님의 인지·물 양 파트 결과입니다.\nYOLOE로 컵 위치를 찾고, 파인튜닝한 segmentation으로 액체 영역을 분리한 뒤, 원근을 보정해 높이 충전율과 부피 충전율을 계산합니다.\n자로 잰 실제 수위와 30회 비교했을 때 액면 높이 오차는 평균 2.8 mm, 부피로는 ±11 mL였습니다.\n부족·도달·초과 판정 정확도는 93%였고, 반사가 심한 투명 컵에서는 81%로 낮아져 이 부분이 남은 과제입니다.\n(검증 결과 수치는 발표 전 실측으로 교체할 값)');
}

async function s15() {
  newSlide();
  header({ num: '06', label: '팀 · 수행 결과 — 기구·하드웨어', title: '4륜 차체에 양팔·RGB-D·LiDAR 결합 완료', orb: 'e', mini: 3 });
  const a0 = { e: 'fade', d: 0, t: 0.4 };
  txt('공통', ML, 1.95, 1.2, 0.5, { size: 26, bold: true, color: C.ink, valign: 'middle' }, a0);
  txt('기구 · 하드웨어', ML + 1.15, 2.01, 1.6, 0.38, { shape: RR, r: 0.19, fill: C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a0);
  const tiles = [['450×340', 'mm', '상·하부 프레임'], ['720', 'mm', '상판 높이 (실측 확인)'], ['4', 'WD', '4륜 스키드스티어 구동'], ['5', 'kg', '추가 적재 · 수동 평지 주행 확인']];
  const tw = 3.05, th = 1.2, gx = 0.2;
  tiles.forEach((t, i) => {
    const x = ML + (i % 2) * (tw + gx), y = 2.65 + Math.floor(i / 2) * (th + 0.2), a = { e: 'zoom', d: 150 + i * 150, t: 0.45 };
    box(x, y, tw, th, { r: 0.12, fill: i === 3 ? C.accT : C.mainT }, a);
    txt([{ text: t[0], options: { fontSize: 32, bold: true } }, { text: ' ' + t[1], options: { fontSize: 16, bold: true } }], x + 0.25, y + 0.12, tw - 0.4, 0.62, { color: i === 3 ? C.accD : C.main, valign: 'middle', check: false }, a);
    txt(t[2], x + 0.25, y + 0.76, tw - 0.4, 0.32, { size: 13, color: C.text }, a);
  });
  txt('프레임 치수는 설계·URDF 기준 · 카메라 렌즈 높이 1,260 mm(상판 위 540 mm)', ML, 5.29, 6.3, 0.26, { size: 11, color: C.faint }, { e: 'fade', d: 750, t: 0.3 });
  const gA = { e: 'up', d: 850, t: 0.45 };
  box(ML, 5.62, 6.3, 0.62, { r: 0.1, fill: C.panel }, gA);
  const gp = await contain('gripper.png', ML + 0.12, 5.66, 0.75, 0.54);
  img(A('gripper.png'), gp.x, gp.y, gp.w, gp.h, { alt: '평행 그리퍼 캐드' }, gA);
  txt([{ text: '그리퍼 선정  ', options: { bold: true, color: C.main } }, { text: '왼손 SO-101 순정 죠, 오른손 ggao50 평행 그리퍼', options: { color: C.text } }], ML + 1.0, 5.66, 5.2, 0.54, { size: 13, valign: 'middle' }, gA);
  callout('검증', '양팔 차체 결합 후 핸드오프 데이터 160개 수집 · 주행 시험', 6.36, { w: 6.3, h: 0.5, size: 13.5 }, { e: 'up', d: 1000, t: 0.45 });
  const cx = 7.6, cw = CR - cx;
  box(cx, 1.95, cw, 4.55, { r: 0.14, fill: C.white, line: C.line }, { e: 'fade', d: 200, t: 0.5 });
  const cad = await contain('cad.png', cx + 0.9, 2.05, cw - 1.2, 4.0);
  img(A('cad.png'), cad.x, cad.y, cad.w, cad.h, { alt: '차체 전체 모델 등각도' }, { e: 'zoom', d: 300, t: 0.6 });
  const top = cad.y + cad.h * 0.27, bot = cad.y + cad.h * 0.8, dxl = cad.x + cad.w * 0.18;
  line(dxl, top, dxl, bot, { color: C.acc, w: 1.5, ba: 'triangle', ea: 'triangle' }, { e: 'wipeD', d: 1100, t: 0.5 });
  txt('720 mm', dxl - 1.05, (top + bot) / 2 - 0.16, 0.95, 0.32, { size: 13, bold: true, color: C.accD, align: 'right', valign: 'middle', check: false }, { e: 'fade', d: 1400, t: 0.3 });
  txt('450×340 mm 전체 모델 · SO-101 좌우 끝 배치', cx, 6.55, cw, 0.3, { size: 12.5, color: C.muted, align: 'center' }, { e: 'fade', d: 600, t: 0.4 });
  notes('하드웨어는 공동으로 진행했습니다.\n450×340 mm 프레임에 상판 높이 720 mm, 그 위에 양팔과 뎁스 카메라, 라이다를 실었습니다.\n출정식 이후 구동계를 4륜 스키드스티어로 최종 결정했고, 양팔을 실제 차체에 결합한 상태로 데이터 160개를 다시 모으고 주행 시험을 했습니다.\n오른손은 ggao50 평행 그리퍼를 써서 물병을 안정적으로 잡도록 했습니다.');
}

module.exports = [s9, s10, s11, s12, s13, s14, s15];
