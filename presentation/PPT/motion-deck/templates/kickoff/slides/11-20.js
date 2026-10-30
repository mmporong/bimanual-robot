'use strict';
// MS팀 출정식 slides 11–20
const L = require('motion-deck');
const { pres, C, FR, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, cover, rounded, contain, header, callout, newSlide, profile, sec, chevrons } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;

async function s11() {
  newSlide();
  header({ num: '07', label: '팀 · 팀 구성', title: '4개의 파트가 하나의 Action 계약에서 만난다', orb: 'e' });
  const cw = 4.95, ch = 2.2, xs = [ML, CR - cw], ys = [1.95, 4.5];
  const parts = [
    ['LuRoute', '이동 · 연결', '임주영', 'SLAM/Nav2 · 장애물 회피 · station 반복 접근 · phase 연결 · Isaac Sim 모델·검증', '도착·정렬 결과, 조작 Action 호출 계약'],
    ['LuHand', '조작 정책', '장준서', '양팔 물 따르기 모방학습', 'phase별 정책 checkpoint'],
    ['LuWrench', '기구 · 하드웨어', '공통', '차체 · 상판 · 그리퍼 · 배선 · 전원', '실측 치수 · 장착 좌표'],
    ['LuScanEye', '인지 · 물 양', '지민석', '물체 인식 · 붓는 동안 무게 변화 대응', '컵·물통 위치 · 물 양 판정'],
  ];
  // hub first so the cards appear to spring out of it
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
}

async function s12() {
  newSlide();
  header({ num: '07', label: '팀 · 수행현황 — 이동·연결', title: '식당 내 목표점 주행과 장애물 대응, 테이블 정밀주차·충전소 복귀 적용', orb: 'a', mini: 1 });
  const px = ML, pw = 4.3, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 4.9, '임주영', '이동 · 연결', a0);
  sec('보유 기술·이력', 'ROS 2 · Nav2 · LiDAR SLAM\n지도 생성·실차 주행·데이터 분석', px + 0.3, 2.95, pw - 0.5, 0.85, { e: 'up', d: 150, t: 0.4 });
  sec('수행 내용', 'Cartographer SLAM·정합·루프 폐합\nAMCL 위치 추정·Keepout 금지구역\nNav2 BT(행동 트리) 조건부 재계획\nNavFn 경로 계획·RPP 경로 추종\nCollision Monitor 정지·같은 목표 재개\n테이블 정밀주차·충전소 도킹 연계', px + 0.3, 3.95, pw - 0.5, 1.95, { e: 'up', d: 300, t: 0.4 }, { size: 12.5, ls: 1.12 });
  sec('다음 단계', '정밀주차·도킹 반복 검증과\n식당 통로의 장애물 대응 연계', px + 0.3, 6.0, pw - 0.5, 0.8, { e: 'up', d: 450, t: 0.4 }, { size: 12.5 });
  const ix = 5.2, iw = CR - ix, ih = iw / (1118 / 460);
  img(await rounded('slam.png', iw, ih, 0.14), ix, 1.95, iw, ih, { alt: '실차 SLAM 장애물 대응 관제 화면' }, { e: 'zoom', d: 300, t: 0.6 });
  txt([{ text: '미등록 장애물 우회·긴급 정지·주행 재개', options: { bold: true, color: C.ink, fontSize: 15 } }, { text: '   LiDAR costmap 갱신 → NavFn 경로 재계획', options: { color: C.muted, fontSize: 12.5 } }], ix, 1.95 + ih + 0.12, iw, 0.36, { valign: 'middle', check: false }, { e: 'fade', d: 700, t: 0.4 });
  chevrons(['01\nSLAM 지도 생성', '02\n목표점·Keepout', '03\n장애물 우회', '04\n긴급 정지', '05\n같은 목표 재개'], ix, 5.72, iw, 0.9, 850, { size: 11 });
}

async function s13() {
  newSlide();
  header({ num: '07', label: '팀 · 수행 현황 — 조작 정책', title: '텔레옵 시연 기반 모방학습으로 물 따르기 Task 구현', orb: 'c', mini: 2 });
  const px = ML, pw = 4.55, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 3.9, '장준서', '조작 정책', a0);
  txt('수행 내역', px + 0.3, 2.92, pw - 0.5, 0.28, { size: 12, bold: true, color: C.main }, { e: 'up', d: 150, t: 0.4 });
  const B = (t, lvl) => ({ text: t, options: { bullet: lvl ? { code: '2013', indent: 12 } : { indent: 12 }, indentLevel: lvl || 0, breakLine: true, fontSize: lvl ? 12 : 13, bold: !lvl, color: lvl ? C.text : C.ink, paraSpaceAfter: lvl ? 1 : 3 } });
  const items = [B('텔레옵 에피소드 60개 수집 및 모델 학습'), B('20⊂40⊂60 부분집합으로 데이터 양에 따른 효과 비교', 1), B('모델 4종 시도 (ACT·SmolVLA·pi0·GR00T)', 1), B('하드웨어 제약에 따른 모델별 한계 확인', 1), B('pi0 원격 추론 지연 7.4초 → 130ms대 단축', 1), B('SmolVLA-40 실물 롤아웃 1회에서 태스크 완주 관찰', 1), B('SmolVLA eval_loss: 20개 → 60개 (38.8%↓)', 1), B('그리퍼 서보 레지스터 문제 2건 원인 규명·복구')];
  delete items[items.length - 1].options.breakLine;
  txt(items, px + 0.25, 3.24, pw - 0.4, 2.55, { size: 12.5, ls: 1.1, check: false }, { e: 'up', d: 250, t: 0.45 });
  // results table (rebuilt from the original screenshot)
  const tx = 5.45, tw2 = CR - tx;
  const hdr = ['실행', '최저 평가 손실', '스텝', '결과'].map((t) => ({ text: t, options: { bold: true, color: C.muted, fill: { color: 'E8F3FA' }, fontSize: 11.5 } }));
  const data = [['ACT — 20 ep', '0.400', '2000', '조기 종료'], ['ACT — 40 ep', '0.367', '3500', '조기 종료'], ['SmolVLA — 20 ep', '0.255', '3000', '조기 종료'], ['SmolVLA — 40 ep', '0.213', '5500', '조기 종료'], ['SmolVLA — 60 ep', '0.156', '6500', '최종 채택'], ['π0 — 20 ep', '0.306', '8500', '학습 완료, 배포 보류'], ['π0 — 40 ep', '0.278', '9500', '학습 완료, 배포 보류'], ['GR00T — 20 / 40 ep', '–', '–', '하드웨어 제약상 불가']];
  const rows = [hdr].concat(data.map((r) => r.map((c, j) => {
    const win = r[3] === '최종 채택';
    return { text: c, options: { bold: win || j === 1, color: win ? (j === 3 ? C.main : C.ink) : j === 3 ? '4A6378' : C.text, fill: { color: win ? C.mainT : C.white }, fontSize: 12 } };
  })));
  L.S.slide.addTable(rows, { x: tx, y: 1.95, w: tw2, colW: [2.35, 1.6, 1.0, tw2 - 4.95], rowH: 0.31, fontFace: FR, valign: 'middle', margin: [0.04, 0.12, 0.04, 0.12], border: { type: 'solid', pt: 0.75, color: 'DCEBF5' }, objectName: 's13_table' });
  L.anim('s13_table', { e: 'wipeD', d: 300, t: 0.8 });
  const stats = [['60', '개', '텔레옵 에피소드 수집', 1.75], ['7.4초 → 130', 'ms대', 'pi0 원격 추론 지연 단축', 2.85], ['38.8', '%↓', 'SmolVLA eval_loss (20→60개)', tw2 - 4.8]];
  let sx = tx;
  stats.forEach((s, i) => {
    const a = { e: 'zoom', d: 900 + i * 160, t: 0.45 };
    box(sx, 4.93, s[3], 0.92, { r: 0.12, fill: i === 2 ? C.main : C.mainT }, a);
    txt([{ text: s[0], options: { fontSize: 24, bold: true } }, { text: s[1], options: { fontSize: 14, bold: true } }], sx + 0.18, 4.97, s[3] - 0.3, 0.5, { color: i === 2 ? C.white : C.main, valign: 'middle', check: false }, a);
    txt(s[2], sx + 0.18, 5.45, s[3] - 0.24, 0.3, { size: 11, color: i === 2 ? 'E3F4FD' : C.muted }, a);
    sx += s[3] + 0.1;
  });
  callout('다음 목표', '(모델 선정) 고정 프로토콜 10회 롤아웃 정량 평가 → (일반화) 시작 위치 다양화 데이터 → 실패 지점 재수집.', 6.15, { h: 0.68, size: 13.5 }, { e: 'up', d: 1450, t: 0.5 });
}

async function s14() {
  newSlide();
  header({ num: '07', label: '팀 · 수행 현황 — 조작 정책', title: 'phase마다 계획 기반과 학습 정책을 같은 조건에서 비교', orb: 'd', mini: 2 });
  const cards = [['IK_ALL', '산업형 기준선', 'RGB-D / 그립 계산 / IK / 검증 궤적 / 그리퍼 피드백, 액면 폐루프', C.ink], ['IL_ALL', '전체 IL 가능성 측정', '모든 조작 phase를 학습 정책으로 실행', C.subD], ['HYBRID', '실이점 구간에만 학습', 'phase별로 더 나은 방식을 선택', C.main]];
  const w = 3.9, g = (CW - 3 * w) / 2, y = 2.1, h = 2.1;
  cards.forEach((c, i) => {
    const x = ML + i * (w + g), hy = i === 2, a = { e: 'up', d: i * 180, t: 0.45 };
    box(x, y, w, h, { r: 0.14, fill: hy ? C.mainT : C.white, line: hy ? C.main : C.line, lw: hy ? 2 : 1, shadow: hy ? undefined : 'soft' }, a);
    txt(c[0], x + 0.3, y + 0.25, w - 0.6, 0.5, { size: 24, bold: true, color: c[3] }, a);
    txt(c[1], x + 0.3, y + 0.8, w - 0.6, 0.38, { size: 16.5, bold: true, color: C.ink }, a);
    txt(c[2], x + 0.3, y + 1.22, w - 0.55, 0.75, { size: 13, color: C.muted, ls: 1.12 }, a);
  });
  // HYBRID fans out into the six manipulation phases
  const hx = ML + 2 * (w + g) + w / 2, busY = 4.6, py = 4.95;
  line(hx, y + h, hx, busY, { color: C.main, w: 2 }, { e: 'wipeD', d: 700, t: 0.3 });
  const ph = [['CUP_PICK', 'PLANNED'], ['BOTTLE_PICK', 'PLANNED'], ['PRE-POUR', 'PLANNED'], ['POUR', 'smolVLA'], ['BOTTLE_RETURN', 'PLANNED'], ['SHELF_PLACE', 'PLANNED']];
  const pw = 1.9, pg = (CW - 6 * pw) / 5;
  const cxs = ph.map((_, i) => ML + i * (pw + pg) + pw / 2);
  line(cxs[0], busY, hx, busY, { color: C.main, w: 2 }, { e: 'wipeL', d: 950, t: 0.6 });
  line(hx, busY, cxs[5], busY, { color: C.main, w: 2 }, { e: 'wipeR', d: 950, t: 0.3 });
  ph.forEach((p, i) => {
    const x = ML + i * (pw + pg), pour = p[1] === 'smolVLA', d = 1250 + i * 110;
    line(cxs[i], busY, cxs[i], py, { color: C.main, w: 2 }, { e: 'wipeD', d, t: 0.2 });
    const card = box(x, py, pw, 1.35, { r: 0.12, fill: pour ? C.accT : C.white, line: pour ? C.acc : C.line, lw: pour ? 2 : 1 }, [{ e: 'grow', d: d + 100, t: 0.35 }].concat(pour ? [{ e: 'pulse', d: 2300, t: 0.4, size: 106, autoRev: true, repeat: 2 }] : []));
    txt(String(i + 1).padStart(2, '0'), x + 0.2, py + 0.14, 0.6, 0.3, { size: 12, bold: true, color: pour ? C.accD : C.main }, { e: 'grow', d: d + 100, t: 0.35 });
    txt(p[0], x + 0.15, py + 0.44, pw - 0.3, 0.34, { size: 13.5, bold: true, color: C.ink, align: 'center', valign: 'middle', check: false }, { e: 'grow', d: d + 100, t: 0.35 });
    txt(p[1], x + 0.35, py + 0.86, pw - 0.7, 0.32, { shape: RR, r: 0.16, fill: pour ? C.acc : C.ink, color: C.white, bold: true, size: 11.5, align: 'center', valign: 'middle', check: false }, { e: 'grow', d: d + 100, t: 0.35 });
    if (L.tw(p[0], 13.5, 'b') > pw - 0.3) L.WARN.push('s14 phase name too long ' + p[0]);
  });
}

async function s15() {
  newSlide();
  header({ num: '07', label: '팀 · 수행 현황 — 인지·물 양', title: '컵·물통 위치 인식과 물 양·흘림 판정, 붓는 동안 무게 변화 대응', orb: 'b', mini: 4 });
  const px = ML, pw = 5.75, a0 = { e: 'fade', d: 0, t: 0.45 };
  profile(px, 1.95, pw, 4.9, '지민석', '인지 · 물 양', a0);
  txt([{ text: '보유 기술·이력  ', options: { bold: true, color: C.main, fontSize: 12 } }, { text: '컴퓨터 비전 개발 경력', options: { color: C.text, fontSize: 13 } }], px + 0.3, 2.82, pw - 0.5, 0.3, { valign: 'middle', check: false }, { e: 'up', d: 120, t: 0.4 });
  txt('계획된 방법', px + 0.3, 3.24, pw - 0.5, 0.28, { size: 12, bold: true, color: C.main }, { e: 'up', d: 220, t: 0.4 });
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
  sec('다음 단계', '자로 잰 실제 수위와 비교해 정확도 검증 → 컵별 mL 보정표\n맑은 물·여러 액체 추가 학습, 흘림 판정, 붓기 정지 신호 연동', px + 0.3, 5.95, pw - 0.5, 0.85, { e: 'up', d: 800, t: 0.4 }, { size: 12.5 });
  const ix = 6.6, iw = CR - ix, ih = iw / (1116 / 505);
  img(await rounded('cupseg.png', iw, ih, 0.14), ix, 1.95, iw, ih, { alt: '컵 액면 분할과 부피 충전율 화면' }, { e: 'zoom', d: 300, t: 0.6 });
  chevrons(['YOLO\nsegmentation', '원근 보정', '액면 높이\n비율', '컵별 보정표', '판정'], ix, 1.95 + ih + 0.12, iw, 0.56, 600, { size: 11 });
  // decision rule drawn as a gauge
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
}

async function s16() {
  newSlide();
  header({ num: '07', label: '팀 · 수행현황 — 기구·하드웨어', title: '양팔·RGB-D·LiDAR를 싣고 컵을 운반할 차체와 상판', orb: 'e', mini: 3 });
  const a0 = { e: 'fade', d: 0, t: 0.4 };
  txt('공통', ML, 1.95, 1.2, 0.5, { size: 26, bold: true, color: C.ink, valign: 'middle' }, a0);
  txt('기구 · 하드웨어', ML + 1.15, 2.01, 1.6, 0.38, { shape: RR, r: 0.19, fill: C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a0);
  const tiles = [['450×340', 'mm', '상·하부 프레임'], ['720', 'mm', '상판 높이'], ['540', 'mm', '바퀴 포함 폭'], ['5', 'kg', '추가 적재 · 수동 평지 주행 확인']];
  const tw = 3.05, th = 1.2, gx = 0.2;
  tiles.forEach((t, i) => {
    const x = ML + (i % 2) * (tw + gx), y = 2.65 + Math.floor(i / 2) * (th + 0.2), a = { e: 'zoom', d: 150 + i * 150, t: 0.45 };
    box(x, y, tw, th, { r: 0.12, fill: i === 3 ? C.accT : C.mainT }, a);
    txt([{ text: t[0], options: { fontSize: 32, bold: true } }, { text: ' ' + t[1], options: { fontSize: 16, bold: true } }], x + 0.25, y + 0.12, tw - 0.4, 0.62, { color: i === 3 ? C.accD : C.main, valign: 'middle', check: false }, a);
    txt(t[2], x + 0.25, y + 0.76, tw - 0.4, 0.32, { size: 13, color: C.text }, a);
  });
  txt('치수는 설계·URDF 기준', ML, 5.29, 6.3, 0.26, { size: 11, color: C.faint }, { e: 'fade', d: 750, t: 0.3 });
  const gA = { e: 'up', d: 850, t: 0.45 };
  box(ML, 5.62, 6.3, 0.62, { r: 0.1, fill: C.panel }, gA);
  const gp = await contain('gripper.png', ML + 0.12, 5.66, 0.75, 0.54);
  img(A('gripper.png'), gp.x, gp.y, gp.w, gp.h, { alt: '평행 그리퍼 캐드' }, gA);
  txt([{ text: '그리퍼 선정  ', options: { bold: true, color: C.main } }, { text: '왼손 SO-101 순정 죠, 오른손 ggao50 평행 그리퍼', options: { color: C.text } }], ML + 1.0, 5.66, 5.2, 0.54, { size: 13, valign: 'middle' }, gA);
  callout('검증', '실측 치수·장착 좌표 → URDF·Isaac 모델', 6.36, { w: 6.3, h: 0.5, size: 13.5 }, { e: 'up', d: 1000, t: 0.45 });
  // CAD with a height dimension drawn on
  const cx = 7.6, cw = CR - cx;
  box(cx, 1.95, cw, 4.55, { r: 0.14, fill: C.white, line: C.line }, { e: 'fade', d: 200, t: 0.5 });
  const cad = await contain('cad.png', cx + 0.9, 2.05, cw - 1.2, 4.0);
  img(A('cad.png'), cad.x, cad.y, cad.w, cad.h, { alt: '차체 전체 모델 등각도' }, { e: 'zoom', d: 300, t: 0.6 });
  const top = cad.y + cad.h * 0.27, bot = cad.y + cad.h * 0.8, dxl = cad.x + cad.w * 0.18;
  line(dxl, top, dxl, bot, { color: C.acc, w: 1.5, ba: 'triangle', ea: 'triangle' }, { e: 'wipeD', d: 1100, t: 0.5 });
  txt('720 mm', dxl - 1.05, (top + bot) / 2 - 0.16, 0.95, 0.32, { size: 13, bold: true, color: C.accD, align: 'right', valign: 'middle', check: false }, { e: 'fade', d: 1400, t: 0.3 });
  txt('450×340 mm 전체 모델 · SO-101 좌우 끝 배치', cx, 6.55, cw, 0.3, { size: 12.5, color: C.muted, align: 'center' }, { e: 'fade', d: 600, t: 0.4 });
}

async function s17() {
  newSlide();
  header({ num: '08', label: '시스템 구조', title: '컴퓨터 3대가 역할을 나눠 로봇 한 대를 움직입니다', orb: 'c', sub: '무거운 AI 계산은 로봇 밖 노트북이, 가볍지만 빠른 판단·실행은 로봇 안의 컴퓨터가 나눠 맡습니다.' });
  L.S.slide.addNotes('[16페이지] 약 1분\n이 로봇은 컴퓨터 세 대가 역할을 나눠 움직입니다.\n첫째, 로봇 밖에 있는 노트북은 생각하는 역할입니다. 웹 주문 화면을 띄우고, 학습한 AI로 컵 잡기와 물 붓기 동작을 계산하고, 카메라 영상으로 컵 속 물 양을 봅니다. 이런 계산은 무거워서 GPU가 있는 노트북에서 합니다.\n둘째, 로봇 안의 라즈베리파이는 두뇌입니다. 주문을 정해진 순서대로 진행하고, 지도를 보며 주방과 테이블, 충전소를 오가고, 양팔 모터에 동작을 전달합니다.\n셋째, ESP32 보드는 손발입니다. 파이가 정한 속도대로 바퀴를 돌리고, 바퀴가 얼마나 굴렀는지와 배터리 상태를 초당 50번 파이에게 알려 줍니다.\n예를 들어 3번 테이블에서 냉수를 주문하면, 노트북 화면으로 주문이 들어오고, 파이가 주방까지 길을 안내하면 ESP32가 바퀴를 돌립니다. 주방에서는 노트북 AI가 동작을 계산하고 파이가 양팔을 움직여 물을 따르며, 노트북이 물 양을 확인합니다. 물을 다 따르면 컵을 상판에 싣고 그때 3번 테이블로 출발합니다. 컵을 내려놓은 뒤에는 충전소로 돌아갑니다.\n이렇게 나눈 이유는 각자 잘하는 일을 맡기기 위해서입니다. 무거운 AI 계산은 GPU 노트북이, 순서와 길 찾기는 파이가, 매 순간 정확해야 하는 바퀴 회전은 ESP32가 맡습니다.\n(참고: 노트북↔파이는 Wi-Fi, 파이↔ESP32는 40핀 UART 50 Hz. 팔 모터 포트는 파이만 연다. 구동계는 선행 JD-AMR에서 검증한 것을 재사용)');
  const y = 2.32, h = 2.25, nw = 3.45, px = 5.2, ex = CR - nw;
  // zones
  box(ML - 0.08, y - 0.3, nw + 0.16, h + 0.42, { r: 0.12, line: C.mainT3, lw: 1.25, dash: 'dash' }, { e: 'fade', d: 0, t: 0.4 });
  txt('로봇 밖', ML + 0.1, y - 0.42, 0.85, 0.26, { size: 11, bold: true, color: C.main, fill: C.white, align: 'center', valign: 'middle', check: false }, { e: 'fade', d: 0, t: 0.4 });
  box(px - 0.12, y - 0.3, CR - px + 0.2, h + 0.42, { r: 0.12, line: C.mainT3, lw: 1.25, dash: 'dash' }, { e: 'fade', d: 100, t: 0.4 });
  txt('로봇 안', px + 0.06, y - 0.42, 0.85, 0.26, { size: 11, bold: true, color: C.main, fill: C.white, align: 'center', valign: 'middle', check: false }, { e: 'fade', d: 100, t: 0.4 });
  const nodes = [
    [ML, 'LuLaptop', '노트북', '로봇의 대뇌', '웹 주문 화면을 띄웁니다\n학습한 AI로 잡기 · 붓기 동작을 계산합니다\n카메라 영상으로 컵 속 물 양을 봅니다', 'GPU 노트북'],
    [px, 'LuCpu', '라즈베리파이', '로봇의 소뇌', '주문을 정해진 순서대로 진행합니다\n지도를 보고 길을 찾아 이동합니다\n양팔 모터에 동작을 전달합니다', 'ROS 2 · SLAM · Nav2'],
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
  // links with data packets that keep moving
  const ly = y + h / 2;
  const links = [[ML + nw, px, '무선'], [px + nw, ex, '유선']];
  links.forEach((lk, i) => {
    const [a1, a2, lab] = lk, d = 800 + i * 200;
    line(a1 + 0.05, ly, a2 - 0.05, ly, { color: C.main, w: 2, ba: 'triangle', ea: 'triangle' }, { e: 'wipeR', d, t: 0.4 });
    txt(lab, (a1 + a2) / 2 - 0.4, ly - 0.42, 0.8, 0.26, { size: 11.5, bold: true, color: C.main, align: 'center', check: false }, { e: 'fade', d, t: 0.4 });
    const span = a2 - a1 - 0.5, per = i === 0 ? 1.4 : 0.7;
    circle(a1 + 0.25, ly, 0.13, { fill: C.sub, name: `s17_pk${i}a` }, [{ e: 'fade', d: d + 300, t: 0.2 }, { e: 'path', d: d + 300, t: per, dx: span / 13.333, dy: 0, repeat: 60 }]);
    circle(a2 - 0.25, ly, 0.13, { fill: C.acc, name: `s17_pk${i}b` }, [{ e: 'fade', d: d + 300 + per * 500, t: 0.2 }, { e: 'path', d: d + 300 + per * 500, t: per, dx: -span / 13.333, dy: 0, repeat: 60 }]);
  });
  txt('예) 3번 테이블 냉수 주문 한 건 — 주방에서 물을 따른 뒤 테이블로 출발합니다', ML, 4.78, CW, 0.3, { size: 13, bold: true, color: C.ink }, { e: 'fade', d: 1300, t: 0.4 });
  const steps = [['주문 접수', '노트북 (웹 화면)'], ['주방으로 이동', '파이 (길 안내)\nESP32 (바퀴 회전)'], ['주방에서 물 따르기', '노트북 (AI가 동작 계산)\n→ 파이 (양팔 실행)\n→ 노트북 (물 양 확인)'], ['테이블로 출발', '파이 · ESP32'], ['컵 놓고 복귀', '노트북·파이 (배치)\n→ 파이·ESP32 (복귀)']];
  const sw = (CW - 4 * 0.12) / 5;
  steps.forEach((s, i) => {
    const x = ML + i * (sw + 0.12), a = { e: 'fade', d: 1400 + i * 130, t: 0.4 }, hi = i === 2;
    box(x, 5.15, sw, 0.97, { r: 0.1, fill: hi ? C.mainT : C.panel, line: hi ? C.main : undefined }, a);
    txt([{ text: `${i + 1}  `, options: { color: C.main } }, { text: s[0], options: { color: C.ink } }], x + 0.14, 5.2, sw - 0.2, 0.28, { size: 12.5, bold: true, check: false }, a);
    txt(s[1], x + 0.14, 5.47, sw - 0.2, 0.6, { size: 11, color: C.text, ls: 1.0 }, a);
  });
  callout('왜 이렇게 나눴나', '각자 잘하는 일을 맡깁니다.   무거운 AI 계산 → GPU 노트북  ·  순서와 길 찾기 → 파이  ·  매 순간 정확한 바퀴 회전 → ESP32', 6.3, { h: 0.56, size: 12.5 }, { e: 'up', d: 2100, t: 0.45 });
}

async function s18() {
  newSlide();
  header({ num: '09', label: '추진 계획 · 개발 추진 계획 — 수행 일정', title: '5주 시나리오', orb: 'a' });
  const wk = [
    ['1주차 · 10/6', '새 차체가 굴러간다', ['구동계 이식·기본 주행', '식당 공간 새 지도 생성']],
    ['2주차 · 10/13', '정해진 자리에 정확히 선다', ['station 교시', '반복 접근 성공률 기록']],
    ['3주차 · 10/20', '웹 주문이 실제 주행을 움직인다', ['Nav2 실차 이동 + 조작 Action', '무선·도킹·충전 신호 시험']],
    ['4주차 · 10/27', '순정 그리퍼로 다시 붓는다', ['시뮬 전체 재실행', 'IK 컵 파지 실물 건식 시험']],
    ['5주차 · 11/2', '4가지 파트가 하나로 만난다', ['정책·인지 결과물 연결', '통합 리허설']],
  ];
  const bw = 2.3, g = (CW - 5 * bw) / 4, base = 6.78, hs = [2.75, 3.1, 3.45, 3.8, 4.15];
  const fills = [C.mainT, C.mainT2, C.mainT3, C.main, C.mainD];
  wk.forEach((w, i) => {
    const x = ML + i * (bw + g), h = hs[i], y = base - h, dark = i >= 3, d = i * 260;
    box(x, y, bw, h, { r: 0.12, fill: fills[i] }, { e: 'wipeU', d, t: 0.55 });
    const ta = { e: 'fade', d: d + 300, t: 0.4 };
    txt(w[0], x + 0.2, y + 0.2, bw - 0.4, 0.28, { size: 12.5, bold: true, color: dark ? 'D9F0FD' : C.mainD }, ta);
    txt(w[1], x + 0.2, y + 0.52, bw - 0.35, 0.72, { size: 15.5, bold: true, color: dark ? C.white : C.ink, ls: 1.1 }, ta);
    txt(w[2].map((t, j) => ({ text: t, options: { bullet: { indent: 10 }, breakLine: j < w[2].length - 1 } })), x + 0.15, y + 0.62 + L.nLines(w[1], 15.5, bw - 0.4, 'b') * 0.285 + 0.12, bw - 0.25, 1.0, { size: 12, color: dark ? 'EAF6FE' : C.text, ls: 1.08, psa: 3, check: false }, ta);
    // week index fills the taller lower part of each step
    txt(String(i + 1).padStart(2, '0'), x + 0.15, base - 0.82, bw - 0.3, 0.7, { size: 40, bold: true, color: [C.mainT2, 'B4DEF7', '8CCBF2', '4FB0E8', '3F98D4'][i], align: 'right', valign: 'bottom', check: false }, { e: 'fade', d: d + 450, t: 0.5 });
  });
  // rising trend arrow + finish flag
  line(ML + 0.3, base - hs[0] - 0.35, ML + 4 * (bw + g) + bw - 0.9, base - hs[4] - 0.35, { color: C.acc, w: 2, dash: 'dash', ea: 'triangle' }, { e: 'wipeR', d: 1500, t: 0.9 });
  await iconDot('LuFlag', ML + 4 * (bw + g) + bw - 0.45, base - hs[4] - 0.42, 0.56, C.acc, C.white, { e: 'grow', d: 2350, t: 0.4 });
}

async function s19() {
  newSlide();
  header({ num: '10', label: '난제와 대응 · 예상 난제와 해결 방안', title: '막힐 것으로 예상되는 지점, 지금부터 대응', orb: 'd' });
  const xD = ML, wD = 1.5, xP = 2.25, wP = 4.85, xA = 7.17, xS = 7.62, wS = CR - 7.62;
  const ha = { e: 'fade', d: 0, t: 0.4 };
  txt('분야', xD, 1.95, wD, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('예상 난제', xP, 1.95, wP, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('해결 방안', xS, 1.95, wS, 0.28, { size: 12, bold: true, color: C.main }, ha);
  const rows = [['LuNavigation', '주행', '팔·상판이 올라간 새 차체의 정지거리·footprint 변화', '실제 외곽으로 footprint 재작성, 실측값 반영'], ['LuCpu', '온보드', 'Pi 4B USB 포트 수·단일 컨트롤러 대역폭', '센서 재배치, 팔 버스 통합 검토, Pi 5 전환 조건 측정'], ['LuWifi', '무선', '추론 지연·두절 중 오래된 명령 실행', '관측 ID·유효기간 계약, 두절 시 안전 hold'], ['LuHand', '조작', 'SO-101 페이로드와 물통 무게', '무게 상한 실측, 컵 몸통 중간 수평 파지'], ['LuDroplet', '물 양', '투명 컵·반사로 액면 인식 어려움', '컵 한 종류·고정 배경부터, 컵별 보정표, 저울 대조'], ['LuMerge', '통합', '파트별 결과물의 입출력 불일치', 'ROS 2 Action 계약 선확정, mock으로 먼저 연결']];
  for (let i = 0; i < rows.length; i++) {
    const [ic, dom, p, s] = rows[i], y = 2.32 + i * 0.74, h = 0.64, d = 200 + i * 200;
    const a = { e: 'fade', d, t: 0.35 };
    box(xD, y, wD, h, { r: 0.1, fill: C.mainD }, a);
    await iconImg(ic, C.white, xD + 0.18, y + 0.19, 0.26, a);
    txt(dom, xD + 0.52, y, wD - 0.6, h, { size: 14, bold: true, color: C.white, valign: 'middle' }, a);
    txt(p, xP, y, wP, h, { fill: C.panel, size: 13.5, color: C.text, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 80, t: 0.4 });
    await iconImg('LuArrowRight', C.main, xA, y + 0.18, 0.28, { e: 'fade', d: d + 200, t: 0.25 });
    txt(s, xS, y, wS, h, { fill: C.mainT, size: 13.5, font: FSB, color: C.ink, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 260, t: 0.4 });
  }
}

async function s20() {
  newSlide();
  header({ num: '11', label: '기대 효과 · 기대 효과[1] — 적용 범위·후속 연계', title: '식당에서 만든 미션 뼈대, 그 다음은 실험실', orb: 'b' });
  const lw = 5.2, rx = CR - 5.2, rw = 5.2;
  const ha = { e: 'fade', d: 0, t: 0.4 };
  box(ML, 1.95, lw, 0.52, { r: 0.1, fill: C.main }, ha);
  await iconImg('LuGlassWater', C.white, ML + 0.2, 2.07, 0.28, ha);
  txt('물 서빙', ML + 0.58, 1.95, 3, 0.52, { size: 16, bold: true, color: C.white, valign: 'middle' }, ha);
  const hb = { e: 'fade', d: 150, t: 0.4 };
  box(rx, 1.95, rw, 0.52, { r: 0.1, fill: C.ink }, hb);
  await iconImg('LuFlaskConical', C.sub, rx + 0.2, 2.07, 0.28, hb);
  txt('실험실 적용', rx + 0.58, 1.95, 3, 0.52, { size: 16, bold: true, color: C.white, valign: 'middle' }, hb);
  const rows = [['웹 주문', '실험 요청(시료 ID·목표량)'], ['주방·테이블 station', '시료 보관대·분석 장비 앞'], ['왼팔 컵·오른팔 물통 따르기', '한 팔 용기 고정·한 팔 시료 따르기'], ['물 양 판정(YOLO·보정표·저울)', '눈금·중량 기반 정량 확인'], ['주문 단위 기록·실패 규약', '실험 추적성(누가·언제·얼마나)']];
  rows.forEach((r, i) => {
    const y = 2.62 + i * 0.66, h = 0.56, d = 300 + i * 230;
    txt(r[0], ML, y, lw, h, { fill: C.mainT, size: 14, font: FSB, color: C.ink, valign: 'middle', margin: 12 }, { e: 'fade', d, t: 0.35 });
    line(ML + lw + 0.15, y + h / 2, rx - 0.15, y + h / 2, { color: C.mainM, w: 1.75, ea: 'triangle' }, { e: 'wipeR', d: d + 120, t: 0.35 });
    txt(r[1], rx, y, rw, h, { fill: C.panel, line: C.line, size: 14, font: FSB, color: C.ink, valign: 'middle', margin: 12 }, { e: 'wipeR', d: d + 300, t: 0.4 });
  });
  callout('선행 사례', '리버풀대 이동형 화학 로봇 8일간 688회 실험(Nature, 2020) · 이동형 매니퓰레이터+양팔 로봇 3대의 12단계 워크플로(Chemical\u00A0Science,\u00A02024)', 6.08, { h: 0.74, size: 13 }, { e: 'up', d: 1600, t: 0.5 });
}

module.exports = [s11, s12, s13, s14, s15, s16, s17, s18, s19, s20];
