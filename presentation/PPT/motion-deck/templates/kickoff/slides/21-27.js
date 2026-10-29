'use strict';
// MS팀 출정식 slides 21–27
const L = require('motion-deck');
const { pres, C, FR, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, header, callout, newSlide } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;

async function s21() {
  newSlide();
  header({ num: '11', label: '기대 효과 · 기대 효과[2] — 실용성 관점', title: '저비용 오픈소스 위의 서비스 계층', orb: 'c' });
  const x = ML, w = 6.7;
  // base layer rises first, then the service tiles stack onto it
  const baseY = 4.25;
  box(x, baseY, w, 1.55, { r: 0.14, fill: C.mainD }, { e: 'wipeU', d: 0, t: 0.55 });
  txt('오픈소스 양팔', x + 0.35, baseY + 0.22, 3, 0.36, { size: 15, bold: true, color: 'CDEBFB' }, { e: 'fade', d: 350, t: 0.4 });
  txt([{ text: '약 ', options: { fontSize: 22, bold: true } }, { text: '$660', options: { fontSize: 46, bold: true } }, { text: '대', options: { fontSize: 22, bold: true } }], x + 0.35, baseY + 0.55, 4.5, 0.85, { color: C.white, valign: 'middle', check: false }, { e: 'zoom', d: 450, t: 0.55 });
  await iconImg('LuCoins', 'CDEBFB', x + w - 1.05, baseY + 0.5, 0.62, { e: 'fade', d: 450, t: 0.4 });
  const topY = 1.98;
  box(x, topY, w, 2.12, { r: 0.14, fill: C.main }, { e: 'down', d: 800, t: 0.55 });
  txt('서비스 미션 계층', x + 0.35, topY + 0.2, 4, 0.38, { size: 16, bold: true, color: C.white }, { e: 'fade', d: 1000, t: 0.4 });
  const tiles = [['LuUsers', '역할 분리'], ['LuClipboardList', '주문 기록'], ['LuSplit', 'phase 비교'], ['LuFileCheck', '실패 규약']];
  const tw = (w - 0.7 - 3 * 0.15) / 4;
  for (let i = 0; i < 4; i++) {
    const tx = x + 0.35 + i * (tw + 0.15), ty = topY + 0.72, a = { e: 'down', d: 1100 + i * 150, t: 0.45 };
    box(tx, ty, tw, 1.15, { r: 0.1, fill: C.white }, a);
    await iconImg(tiles[i][0], C.main, tx + tw / 2 - 0.2, ty + 0.18, 0.4, a);
    txt(tiles[i][1], tx, ty + 0.66, tw, 0.36, { size: 14, bold: true, color: C.ink, align: 'center', valign: 'middle' }, a);
  }
  const rx = 7.75, rw = CR - rx;
  txt('약 $660대 오픈소스 양팔\n+ 서비스 미션 계층', rx, 2.75, rw, 1.15, { size: 24, bold: true, color: C.ink, ls: 1.15 }, { e: 'fade', d: 1300, t: 0.5 });
  txt('저가 오픈소스 양팔 위에 쌓은 서비스 계층이라, 초기 도입 비용 장벽이 상대적으로 낮다.', rx, 4.1, rw, 1.0, { size: 16, color: C.muted, ls: 1.3 }, { e: 'fade', d: 1450, t: 0.5 });
  callout('열린 질문', '팔이 늘어난 만큼 비용·복잡도도 늘어난다 — 사업화 판단은 비용 대비 활용도 검증 이후로 열어둔다.', 6.25, { color: C.acc, size: 13.5 }, { e: 'up', d: 1900, t: 0.5 });
}

async function s22() {
  newSlide();
  header({ num: '12', label: '안전·보안 · 보안·안전성 검증 및 조치', title: '명령은 한 통로로, 실기체는 확인 후에만', orb: 'a' });
  // single path for every command (detail in appendix A)
  const flow = ['lease 하나', '중재기', '안전 게이트', '하드웨어'];
  const n = flow.length, ov = 0.18, fw = (CW + ov * (n - 1)) / n, fy = 1.98, fh = 0.62;
  flow.forEach((f, i) => {
    txt(f, ML + i * (fw - ov), fy, fw, fh, { shape: i === 0 ? pres.shapes.PENTAGON : pres.shapes.CHEVRON, fill: [C.mainT, C.mainT2, C.main, C.mainD][i], color: i >= 2 ? C.white : C.ink, bold: true, size: 15, align: 'center', valign: 'middle', check: false }, { e: 'wipeR', d: i * 160, t: 0.35 });
  });
  circle(ML + 0.3, fy + fh / 2, 0.16, { fill: C.acc, line: C.white, lw: 1.5, name: 's22_cmd' }, [{ e: 'fade', d: 700, t: 0.2 }, { e: 'path', d: 750, t: 1.6, dx: (CW - 0.75) / 13.333, dy: 0, repeat: 2 }]);
  const cards = [['LuShieldCheck', '명령 중재', '단일 lease + 안전 게이트', '스톨·관절 한계·충돌 위험·통신 timeout·컵 이탈·흘림 위험은 실제 hold/stop으로 연결한다.'], ['LuWifi', '무선 계약', '유효기간 지난 명령은 폐기', '지연·중복·재연결 후 남은 행동은 Pi에서 폐기하고 안전 hold로 전환한다.'], ['LuDatabase', '데이터·운영', '실물은 확인 후, 데이터는 비공개', '팔을 움직이는 모든 동작은 사용자 확인 후 실행하며, 데이터셋·모델은 HuggingFace Hub(private)에 둔다.']];
  const w = 3.9, g = (CW - 3 * w) / 2, y = 2.95, h = 3.8;
  for (let i = 0; i < 3; i++) {
    const x = ML + i * (w + g), a = { e: 'up', d: 900 + i * 200, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    await iconDot(cards[i][0], x + 0.68, y + 0.72, 0.8, C.mainT, C.main, a);
    txt(cards[i][1], x + 0.32, y + 1.36, w - 0.6, 0.3, { size: 13, bold: true, color: C.main }, a);
    txt(cards[i][2], x + 0.32, y + 1.68, w - 0.5, 0.8, { size: 18, bold: true, color: C.ink, ls: 1.12 }, a);
    txt(cards[i][3], x + 0.32, y + 2.5, w - 0.6, 1.15, { size: 13.5, color: C.muted, ls: 1.2 }, a);
  }
}

async function s23() {
  newSlide();
  header({ num: '13', label: '핵심 포인트', title: '오늘 꼭 기억해 주셨으면 하는 네 가지', orb: 'e' });
  const cards = [
    ['시뮬', C.sub, C.ink, '12/12', 'phase', '웹 주문 한 건, 12 phase 완주', '473.9초 만에 도크 복귀까지 · 운반 중 물 손실 0'],
    ['실차·실물', C.main, C.white, '20/20', 'waypoint', '복도 77 m 왕복', '20/20 waypoint · recovery 0회 (선행 JD-AMR 구동계)'],
    ['실차·실물', C.main, C.white, '38.8%↓', 'eval_loss', 'SmolVLA 정책 학습', 'eval_loss 38.8%↓(20→60 에피소드) · 실물 롤아웃 태스크 완주 관찰'],
    ['계획', C.acc, C.white, '+4', '서비스 계층', '오픈소스 팔 + 서비스 계층', '역할분리·주문기록·phase 비교·실패 규약 네 가지를 더한다'],
  ];
  const w = 5.97, h = 1.92, gx = CW - 2 * w;
  cards.forEach((c, i) => {
    const x = ML + (i % 2) * (w + gx), y = 1.95 + Math.floor(i / 2) * (h + 0.2), a = { e: 'zoom', d: i * 200, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    txt(c[3], x + 0.3, y + 0.3, 2.15, 0.85, { size: c[3].length > 5 ? 34 : 40, bold: true, color: c[1] === C.acc ? C.acc : C.main, valign: 'middle', check: false }, a);
    txt(c[4], x + 0.3, y + 1.15, 2.15, 0.3, { size: 12, color: C.muted }, a);
    txt(c[0], x + w - 1.4, y + 0.25, 1.15, 0.32, { shape: RR, r: 0.16, fill: c[1], color: c[2], bold: true, size: 11.5, align: 'center', valign: 'middle', check: false }, a);
    txt(c[5], x + 2.55, y + 0.62, w - 2.85, 0.42, { size: 17, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(c[6], x + 2.55, y + 1.05, w - 2.85, 0.72, { size: 12.5, color: C.muted, ls: 1.15 }, a);
  });
  callout('핵심 포인트', '시뮬 1회·실차 1회·실물 1회의 근거 위에, 서비스 계층이라는 포지셔닝을 더한다.', 6.27, { color: C.main }, { e: 'up', d: 1000, t: 0.5 });
}

async function s24() {
  newSlide();
  L.orbs('t', false, 'E3F4FD', 'CDEBFB');
  // header without the default orbs (the closing slide uses the title slide's backdrop)
  txt('14', ML, 0.47, 0.46, 0.3, { name: '!!chipnum', shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.15, fill: C.main, color: C.white, bold: true, size: 11, align: 'center', valign: 'middle', check: false });
  txt('마무리', ML + 0.6, 0.47, 8.6, 0.3, { name: '!!chiplabel', font: FSB, size: 12.5, color: C.mainD, valign: 'middle', check: false });
  txt('오늘 멘토링에서 검증받고 싶은 두 가지', ML, 0.88, 11.45, 0.72, { name: '!!title', size: 30, bold: true, color: C.ink, valign: 'middle', check: false });
  img(A('logo_mark_light_sky.png'), 12.29, 0.42, 0.44, 0.418, { name: '!!logo', alt: 'MS Team 로고' });
  L.pageNum();
  const qs = [['Q1', '궁금한 점 1', '에이전트 계층의 경계', '자연어 주문 해석·예외 상황 설명을 에이전트에 맡길 때, 허용된 스킬 목록 안에서만 고르게 하는 설계가 현장에서 충분할지 검증받고 싶습니다.'], ['Q2', '궁금한 점 2', '모바일 매니퓰레이터의 현장 운용', '팔을 더한 서빙 로봇의 기능 대비 비용 판단과 매장 안전 운용 기준에 대한 조언을 구하고 싶습니다.']];
  const w = 5.97, gx = CW - 2 * w, y = 1.95, h = 2.75;
  qs.forEach((q, i) => {
    const x = ML + i * (w + gx), a = { e: 'up', d: i * 220, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    txt(q[0], x + 0.32, y + 0.25, 1.2, 0.72, { size: 40, bold: true, color: C.main, valign: 'middle', check: false }, a);
    txt(q[1], x + 1.45, y + 0.44, 2.5, 0.32, { size: 12.5, color: C.muted, valign: 'middle' }, a);
    txt(q[2], x + 0.32, y + 1.05, w - 0.6, 0.45, { size: 20, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(q[3], x + 0.32, y + 1.58, w - 0.6, 1.0, { size: 13.5, color: C.muted, ls: 1.2 }, a);
  });
  txt('물 한 잔을, 준비부터 테이블 위까지. 한 주문의 기록이 끝까지 이어지는 로봇을 만듭니다.', ML, 5.08, CW, 0.45, { size: 19, font: FSB, color: C.mainD }, { e: 'wipeR', d: 700, t: 0.9 });
  txt('감사합니다', ML, 5.68, 5, 0.75, { size: 36, bold: true, color: C.ink, valign: 'middle' }, { e: 'zoom', d: 1500, t: 0.5 });
  txt('github.com/mmporong/bimanual-robot', ML, 6.42, 6, 0.34, { size: 14.5, color: C.mainD, hyperlink: { url: 'https://github.com/mmporong/bimanual-robot' } }, { e: 'fade', d: 1700, t: 0.4 });
  img(A('logo_h_light_sky.png'), CR - 2.72, 5.9, 2.72, 0.558, { alt: 'MS Team 로고' }, { e: 'fade', d: 1700, t: 0.5 });
}

async function s25() {
  newSlide();
  header({ num: 'A', label: '부록 A — 안전·실패데이터 규약·무선 계약', title: '본문 요약을 프로토콜 수준까지 풀어쓴 Q&A 백업', orb: 'b' });
  const w = 3.92, g = (CW - 3 * w) / 2, y = 1.95, h = 4.0;
  const heads = [['LuShieldCheck', '안전·명령 중재'], ['LuListChecks', '실패 데이터 규약'], ['LuWifi', '무선 계약']];
  for (let i = 0; i < 3; i++) {
    const x = ML + i * (w + g), a = { e: 'up', d: i * 180, t: 0.45 };
    box(x, y, w, h, { r: 0.14, fill: C.panel }, a);
    await iconImg(heads[i][0], C.main, x + 0.25, y + 0.22, 0.32, a);
    txt(heads[i][1], x + 0.68, y + 0.18, w - 0.9, 0.4, { size: 15, bold: true, color: C.ink, valign: 'middle' }, a);
  }
  // col 1: arbitration chain
  let x = ML + 0.25;
  const chain = ['lease 하나', '중재기', '안전 게이트', '하드웨어'];
  chain.forEach((c, i) => {
    const cy = y + 0.75 + i * 0.42, a = { e: 'fade', d: 400 + i * 120, t: 0.3 };
    txt(c, x, cy, 1.9, 0.32, { shape: RR, r: 0.16, fill: i === 3 ? C.ink : C.white, line: C.line, color: i === 3 ? C.white : C.ink, bold: true, size: 12, align: 'center', valign: 'middle', check: false }, a);
    if (i < 3) txt('↓', x + 1.95, cy + 0.02, 0.3, 0.3, { size: 12, color: C.main, bold: true, check: false }, a);
  });
  const bl = (arr) => arr.map((t, j) => ({ text: t, options: { bullet: { indent: 10 }, breakLine: j < arr.length - 1 } }));
  txt(bl(['스톨·관절 한계·충돌 위험·통신 timeout·컵 이탈·흘림 위험은 hold/stop으로 연결', '정지가 항상 토크 OFF는 아니다 — 물체를 든 자세를 고려한다']), x, y + 2.5, w - 0.45, 1.4, { size: 12, color: C.text, ls: 1.12, psa: 4 }, { e: 'fade', d: 900, t: 0.4 });
  // col 2: L1→L5 ladder
  x = ML + (w + g) + 0.25;
  txt('L1→L5 순서로 원인을 배제한다', x, y + 0.68, w - 0.45, 0.3, { size: 12.5, bold: true, color: C.main }, { e: 'fade', d: 400, t: 0.3 });
  const lv = ['L1 하드웨어·연결', 'L2 좌표', 'L3 로직·파라미터', 'L4 데이터·분포', 'L5 측정'];
  lv.forEach((t, i) => {
    txt(t, x + i * 0.12, y + 1.05 + i * 0.3, 2.6, 0.26, { fill: [C.mainT, C.mainT2, 'B9E2F9', C.mainT3, C.mainM][i], color: C.ink, bold: true, size: 11, valign: 'middle', margin: 6, check: false }, { e: 'wipeR', d: 500 + i * 110, t: 0.3 });
  });
  txt(bl(['실패 1건으로 고치지 않는다 — 같은 조건 3회 + 다른 조건 3회', '원인이 확인되지 않은 실패는 학습 데이터에 넣지 않는다']), x, y + 2.65, w - 0.45, 1.25, { size: 12, color: C.text, ls: 1.12, psa: 4 }, { e: 'fade', d: 1100, t: 0.4 });
  // col 3: wireless contract
  x = ML + 2 * (w + g) + 0.25;
  txt('관측 ID·유효기간이 있는 명령만 실행', x, y + 0.68, w - 0.45, 0.3, { size: 12.5, bold: true, color: C.main }, { e: 'fade', d: 600, t: 0.3 });
  txt(bl(['Pi → 노트북: 요청·관측 ID, 관절 상태·시각', '노트북 → Pi: 대응 관측 ID, checkpoint 버전, 관절 순서·단위·유효기간', '지연·중복·재연결 후 남은 행동은 폐기하고 안전 hold로 전환']), x, y + 1.12, w - 0.45, 2.8, { size: 13.5, color: C.text, ls: 1.2, psa: 12 }, { e: 'fade', d: 700, t: 0.4 });
  callout('이 자료는', '본문 22쪽(안전·보안)의 요약을 프로토콜 수준까지 풀어쓴 Q&A 백업이다 — 본문을 대체하지 않는다.', 6.15, { size: 13.5 }, { e: 'up', d: 1300, t: 0.45 });
}

async function s26() {
  newSlide();
  header({ num: 'B', label: '부록 B — 시뮬 수치 원본·예상 난제', title: '발표 중 세부 수치를 물으면 여기서 답한다', orb: 'c' });
  txt('시뮬 · Isaac Sim 5.1 · 1회', ML, 1.95, L.tw('시뮬 · Isaac Sim 5.1 · 1회', 12.5, 'sb') + 0.42, 0.36, { shape: RR, r: 0.18, fill: C.mainT, color: C.mainD, font: FSB, size: 12.5, align: 'center', valign: 'middle', check: false }, { e: 'fade', d: 0, t: 0.4 });
  const hdr = ['실행', '조건', '결과'].map((t) => ({ text: t, options: { bold: true, color: C.white, fill: { color: C.ink }, fontSize: 13 } }));
  const data = [['plate_service02', '물따르기→상판적재→주행→재파지→테이블배치 1회', '282.9초 · 컵684·병230·바깥4 · 팔·환경접촉 0'], ['dock_roundtrip02', '12 phase, 충전소 왕복 포함', '473.9초 · 컵687·병227·바깥4'], ['flared03', '입구 90mm 컵 양팔 물붓기', '유출 3/918 입자'], ['순정 SO-101 왼손 컵 접촉', '10g·210g 조건 파지·상승·내려놓기·손 분리', '각 1회 통과']];
  const rows = [hdr].concat(data.map((r, i) => r.map((c, j) => ({ text: c, options: { bold: j === 0, color: j === 0 ? C.ink : C.text, fill: { color: i % 2 ? C.panel : C.white }, fontSize: 13 } }))));
  L.S.slide.addTable(rows, { x: ML, y: 2.5, w: CW, colW: [2.75, 4.68, 4.7], rowH: [0.46, 0.62, 0.62, 0.62, 0.62], fontFace: FR, valign: 'middle', margin: [0.04, 0.14, 0.04, 0.14], border: { type: 'solid', pt: 0.75, color: 'DCEBF5' }, objectName: 's26_table' });
  L.anim('s26_table', { e: 'wipeD', d: 200, t: 0.8 });
  txt('공통: 시뮬 정답 좌표 사용 · dock_roundtrip02/plate_service02는 왼손 80mm 강체 패드 형상 가정', ML, 5.55, CW, 0.3, { size: 12, color: C.muted }, { e: 'fade', d: 900, t: 0.4 });
  callout('참고', '예상 난제·해결 방안 표는 본문 19쪽과 동일하다 — 여기서는 세부 시뮬 수치만 보탠다.', 6.15, { size: 13.5 }, { e: 'up', d: 1100, t: 0.45 });
}

async function s27() {
  newSlide();
  header({ num: 'C', label: '부록 C — 주제 변천·저장소 운영', title: '왜 지금 이 미션인가, 그리고 어떻게 운영하는가', orb: 'd' });
  const ev = [['8/21', '양팔 커넥터 체결 셀'], ['8/28', '이동형 용기 붓기·봉지 열기 병행'], ['9/7', '물 서빙 단일 미션 확정 — 양팔 필요성·하드웨어 한계·데이터 수집 가능성·이동 통합 기준']];
  const w = 3.92, g = (CW - 3 * w) / 2, ly = 2.2;
  line(ML + 0.25, ly, ML + 2 * (w + g) + 0.25, ly, { color: C.mainT2, w: 2.5 }, { e: 'wipeR', d: 0, t: 0.9 });
  ev.forEach((e, i) => {
    const x = ML + i * (w + g), last = i === 2, d = 150 + i * 280, a = { e: 'fade', d: d + 100, t: 0.4 };
    txt(String(i + 1), x, ly - 0.25, 0.5, 0.5, { shape: pres.shapes.OVAL, fill: last ? C.main : C.white, line: C.main, lw: 2, color: last ? C.white : C.main, bold: true, size: 14, align: 'center', valign: 'middle', check: false }, { e: 'grow', d, t: 0.3 });
    txt(e[0], x, ly + 0.4, w, 0.45, { size: 22, bold: true, color: last ? C.main : C.ink }, a);
    txt(e[1], x, ly + 0.9, w - 0.2, 0.95, { size: 13.5, color: C.text, ls: 1.15 }, a);
  });
  txt('저장소 운영', ML, 4.15, 3, 0.32, { size: 15, bold: true, color: C.ink }, { e: 'fade', d: 1000, t: 0.4 });
  const fl = ['Issue', '브랜치', 'PR', 'CI 검사', '병합'];
  const n = fl.length, ov = 0.14, fw = 1.45, fy = 4.58;
  fl.forEach((f, i) => {
    txt(f, ML + i * (fw - ov), fy, fw, 0.5, { shape: i === 0 ? pres.shapes.PENTAGON : pres.shapes.CHEVRON, fill: i === n - 1 ? C.main : [C.mainT, C.mainT2, 'B9E2F9', C.mainT3][i], color: i === n - 1 ? C.white : C.ink, bold: true, size: 13, align: 'center', valign: 'middle', check: false }, { e: 'wipeR', d: 1100 + i * 130, t: 0.3 });
  });
  const notes = [['LuDatabase', '데이터·모델은 HuggingFace Hub(private), 저장소에는 인덱스만'], ['LuFileCheck', '실패 원인 귀속 순서와 재시험 규칙을 저장소 규칙으로 공유']];
  const nx = 7.9, nw = CR - nx;
  for (let i = 0; i < 2; i++) {
    const y = 4.15 + i * 0.75, a = { e: 'up', d: 1700 + i * 150, t: 0.4 };
    box(nx, y, nw, 0.62, { r: 0.1, fill: C.panel }, a);
    await iconImg(notes[i][0], C.main, nx + 0.2, y + 0.17, 0.28, a);
    txt(notes[i][1], nx + 0.62, y, nw - 0.75, 0.62, { size: 12.5, color: C.text, valign: 'middle', ls: 1.08 }, a);
  }
  callout('운영 원칙', '결정 기준과 저장소 규칙을 투명하게 공개해 재현성을 높인다.', 6.15, { size: 14 }, { e: 'up', d: 2100, t: 0.45 });
}

module.exports = [s21, s22, s23, s24, s25, s26, s27];
