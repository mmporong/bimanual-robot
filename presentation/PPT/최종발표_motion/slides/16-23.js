'use strict';
// MS팀 최종 발표 slides 16–23: 데모 · 계획 대비 실적 · 난제 대응 · 기대 효과 · 한계 · 핵심 포인트 · 마무리
// ※ [임의] 표시가 붙은 수치는 실험 전 자리표시 값이다(발표 전 실측으로 교체).
const L = require('motion-deck');
const { pres, C, FSB, ML, CR, CW, A, txt, box, circle, line, img, iconImg, iconDot, header, callout, newSlide, tagPill, videoCard, seqLine } = L;
const RR = pres.shapes.ROUNDED_RECTANGLE;
const notes = (t) => L.S.slide.addNotes(t);

async function s16() {
  newSlide();
  header({ num: '07', label: '데모 · 시뮬레이션 데모', title: '시뮬레이션에서 전 과정을 먼저 검증했다', orb: 'a' });
  const vw = 7.75, vh = vw * 9 / 16;
  await videoCard('s16_video', 'v9n.mp4', 'poster9n.jpg', ML, 1.9, vw, vh, 19883, 600);
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
  txt('구형 80 mm 강체 패드·정답 좌표 조건의 시뮬 1회 — 실물 결과와 합산하지 않는다', x, 6.05, w, 0.66, { size: 11.5, color: C.muted, ls: 1.15 }, { e: 'fade', d: 1200, t: 0.4 });
  notes('데모입니다. 먼저 Isaac Sim 시뮬레이션에서 전 과정을 검증했습니다.\n컵과 병을 잡고 물을 따른 뒤 상판에 싣고 이동해, 테이블에 컵을 놓고 도크로 돌아오는 12개 phase를 473.9초에 모두 통과했고, 운반 중 추가로 흘린 물은 없었습니다.');
}

async function s17() {
  newSlide();
  header({ num: '07', label: '데모 · 실물 데모 (물따르기 → 컵배치)', title: '실물에서도 핸드오프 전 과정을 수행한다', orb: 'b' });
  const vw = 7.75, vh = vw * 9 / 16, vx = CR - vw;
  // ※ 영상은 출정식 실물 영상(v10) 자리표시 — 차체 결합 물따르기→컵배치 영상으로 교체
  await videoCard('s17_video', 'v10.mp4', 'poster10.jpg', vx, 1.9, vw, vh, 38633, 600);
  seqLine(['컵·병 파지', '물따르기', '상판 적재', '테이블 배치'], vx, 1.9 + vh + 0.16, vw, 400);
  const x = ML, w = vx - ML - 0.35;
  tagPill('SmolVLA · 차체 결합 80개 학습', x, 1.9, 300);
  const rows = [['실행 방식', 'IK 없이 학습 정책 단독 실행(전 구간 IL)'], ['수행 모델', 'SmolVLA-80 (물따르기·컵배치 정책 각 1개)'], ['롤아웃 결과', '파지→붓기→상판 적재→테이블 배치 완주', 1], ['정량 평가', '고정 프로토콜 10회: 물따르기 9 · 컵배치 8회']];
  rows.forEach((r, i) => {
    const y = 2.45 + i * 0.78, a = { e: 'up', d: 450 + i * 150, t: 0.4 };
    line(x, y, x + w, y, { color: C.line, w: 1 }, a);
    txt(r[0], x, y + 0.08, w, 0.26, { size: 11.5, color: C.muted }, a);
    txt(r[1], x, y + 0.33, w, 0.42, { size: 14, bold: true, color: r[2] ? C.main : C.ink, ls: 1.05 }, a);
  });
  const cy = 5.6, ch = 1.12, a = { e: 'up', d: 1150, t: 0.5 };
  box(x, cy, w, ch, { r: 0.1, fill: C.panel }, a);
  txt('검증', x + 0.18, cy + 0.15, 0.78, 0.34, { shape: RR, r: 0.17, fill: C.main, color: C.white, bold: true, size: 12.5, align: 'center', valign: 'middle', check: false }, a);
  txt('IK 없이 모방학습과 안전 래퍼만으로 핸드오프 task를 반복 수행함을 확인.', x + 0.18, cy + 0.56, w - 0.36, 0.5, { size: 13, font: FSB, color: C.ink, ls: 1.1 }, a);
  notes('실물 데모입니다.\n차체에 결합된 양팔이 학습한 정책만으로 컵과 병을 잡고 물을 따른 뒤, 컵을 상판에 싣고, 이어서 컵을 테이블에 내려놓습니다.\n같은 조건 10회 시험에서 물따르기는 9회, 컵배치는 8회 성공했습니다.\nIK 없이 모방학습과 안전 래퍼만으로 핸드오프 전 과정을 반복 수행할 수 있음을 확인했습니다.\n(영상은 차체 결합 핸드오프 영상으로 교체, 10회 결과는 실측으로 교체)');
}

async function s18() {
  newSlide();
  header({ num: '08', label: '추진 실적 · 5주 계획 대비 결과', title: '5주 계획 대비 실적', orb: 'a' });
  const wk = [
    ['1주차 · 10/6', '새 차체가 굴러간다', ['✓ 4WD 구동계 이식', '✓ 식당 공간 지도 생성']],
    ['2주차 · 10/13', '정해진 자리에 정확히 선다', ['✓ 1 cm 목표 정밀주차', '   실측 12 mm']],
    ['3주차 · 10/20', '웹 주문이 실제 주행을 움직인다', ['✓ 도크→급수대→테이블', '   →도크 연속 왕복']],
    ['4주차 · 10/27', '실제로 물을 붓고 목표 테이블로 이동한다', ['✓ 핸드오프 IL 80+80개', '✓ 실물 연속 완주', '✓ 물 양 판정 93%']],
    ['5주차 · 11/2', '4가지 파트가 하나로 만난다', ['△ 통합 시연', '   10회 중 7회 완주']],
  ];
  const bw = 2.3, g = (CW - 5 * bw) / 4, base = 6.78, hs = [2.75, 3.1, 3.45, 3.8, 4.15];
  const fills = [C.mainT, C.mainT2, C.mainT3, C.main, C.mainD];
  wk.forEach((w, i) => {
    const x = ML + i * (bw + g), h = hs[i], y = base - h, dark = i >= 3, d = i * 260;
    box(x, y, bw, h, { r: 0.12, fill: fills[i] }, { e: 'wipeU', d, t: 0.55 });
    const ta = { e: 'fade', d: d + 300, t: 0.4 };
    txt(w[0], x + 0.2, y + 0.2, bw - 0.4, 0.28, { size: 12.5, bold: true, color: dark ? 'D9F0FD' : C.mainD }, ta);
    txt(w[1], x + 0.2, y + 0.52, bw - 0.35, 0.72, { size: 15.5, bold: true, color: dark ? C.white : C.ink, ls: 1.1 }, ta);
    const ty = y + 0.62 + L.nLines(w[1], 15.5, bw - 0.4, 'b') * 0.285 + 0.12;
    txt(w[2].map((t, j) => ({ text: t, options: { breakLine: j < w[2].length - 1 } })), x + 0.2, ty, bw - 0.3, 1.0, { size: 12, font: FSB, color: dark ? 'EAF6FE' : C.text, ls: 1.08, psa: 3, check: false }, { e: 'fade', d: d + 450, t: 0.4 });
    txt(String(i + 1).padStart(2, '0'), x + 0.15, base - 0.82, bw - 0.3, 0.7, { size: 40, bold: true, color: [C.mainT2, 'B4DEF7', '8CCBF2', '4FB0E8', '3F98D4'][i], align: 'right', valign: 'bottom', check: false }, { e: 'fade', d: d + 450, t: 0.5 });
  });
  line(ML + 0.3, base - hs[0] - 0.35, ML + 4 * (bw + g) + bw - 0.9, base - hs[4] - 0.35, { color: C.acc, w: 2, dash: 'dash', ea: 'triangle' }, { e: 'wipeR', d: 1500, t: 0.9 });
  await iconDot('LuFlag', ML + 4 * (bw + g) + bw - 0.45, base - hs[4] - 0.42, 0.56, C.acc, C.white, { e: 'grow', d: 2350, t: 0.4 });
  notes('출정식에서 발표한 5주 계획 대비 실적입니다.\n1주차 4륜 구동계 이식과 지도 생성, 2주차 1 cm 목표 정밀주차, 3주차 도크부터 테이블까지 연속 왕복을 달성했습니다.\n4주차에는 차체 결합 데이터로 물따르기와 컵배치 정책을 학습해 실물 연속 완주를 확인했고, 물 양 판정 모델도 완성했습니다.\n5주차 통합 시연은 10회 중 7회 완주로, 반복 신뢰성은 아직 개선할 부분이 있어 세모로 표시했습니다.\n(93%, 10회 중 7회는 실측으로 교체할 값)');
}

async function s19() {
  newSlide();
  header({ num: '09', label: '난제와 대응 · 예상 난제 대응 결과', title: '출정식에서 예상한 난제, 이렇게 대응했다', orb: 'd' });
  const xD = ML, wD = 1.5, xP = 2.25, wP = 4.85, xA = 7.17, xS = 7.62, wS = CR - 7.62;
  const ha = { e: 'fade', d: 0, t: 0.4 };
  txt('분야', xD, 1.95, wD, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('예상했던 난제', xP, 1.95, wP, 0.28, { size: 12, bold: true, color: C.muted }, ha);
  txt('실제 대응 결과', xS, 1.95, wS, 0.28, { size: 12, bold: true, color: C.main }, ha);
  const rows = [['LuNavigation', '주행', '팔·상판이 올라간 새 차체의 정지거리·footprint 변화', '4WD 전환 · 통로 0.12 m/s · 주차 제동 구간 적용'], ['LuCpu', '온보드', 'Pi 4B USB 포트 수·단일 컨트롤러 대역폭', 'LiDAR·구동부 UART 분리, 양팔·카메라는 노트북 직결'], ['LuWifi', '무선', '추론 지연·두절 중 오래된 명령 실행', '정책 추론은 노트북 로컬 실행, 카메라 지연 시 틱 건너뛰고 토크 유지'], ['LuHand', '조작', 'SO-101 페이로드와 물통 무게', '300g대 물병 사용, 그리퍼 과부하 예방 로직 추가'], ['LuDroplet', '물 양', '투명 컵·반사로 액면 인식 어려움', '배경·컵 고정 조건에서 판정 정확도 93%'], ['LuMerge', '통합', '파트별 결과물의 입출력 불일치', 'ROS 2 Action 계약 · mock 검증 후 통합 시연 진행']];
  for (let i = 0; i < rows.length; i++) {
    const [ic, dom, p, s] = rows[i], y = 2.32 + i * 0.74, h = 0.64, d = 200 + i * 200;
    const a = { e: 'fade', d, t: 0.35 };
    box(xD, y, wD, h, { r: 0.1, fill: C.mainD }, a);
    await iconImg(ic, C.white, xD + 0.18, y + 0.19, 0.26, a);
    txt(dom, xD + 0.52, y, wD - 0.6, h, { size: 14, bold: true, color: C.white, valign: 'middle' }, a);
    txt(p, xP, y, wP, h, { fill: C.panel, size: 13.5, color: C.text, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 80, t: 0.4 });
    await iconImg('LuArrowRight', C.main, xA, y + 0.18, 0.28, { e: 'fade', d: d + 200, t: 0.25 });
    txt(s, xS, y, wS, h, { fill: C.mainT, size: 13, font: FSB, color: C.ink, valign: 'middle', margin: 10, ls: 1.05 }, { e: 'wipeR', d: d + 260, t: 0.4 });
  }
  notes('출정식에서 예상했던 난제 여섯 가지를 실제로 어떻게 대응했는지 정리했습니다.\n주행은 4륜 전환과 통로 저속 주행, 주차 제동 구간으로 안정성을 확보했고, 온보드 포트 부족은 라이다와 구동부를 UART로 분리하고 양팔과 카메라를 노트북에 직접 연결해 해결했습니다.\n무선 지연 문제는 정책 추론을 노트북에서 로컬로 돌려 피했고, 카메라 프레임이 늦게 오면 그 틱만 건너뛰고 토크를 유지해 물체를 떨어뜨리지 않게 했습니다.');
}

async function s20() {
  newSlide();
  header({ num: '10', label: '기대 효과 · 적용 범위·후속 연계', title: '식당에서 만든 미션 뼈대, 그 다음은 실험실', orb: 'b' });
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
  callout('선행 사례', '리버풀대 이동형 화학 로봇 8일간 688회 실험(Nature, 2020) · 이동형 매니퓰레이터+양팔 로봇 3대의 12단계 워크플로(Chemical Science, 2024)', 6.08, { h: 0.74, size: 13 }, { e: 'up', d: 1600, t: 0.5 });
  notes('기대 효과입니다. 식당에서 만든 미션 구조는 실험실로 확장할 수 있습니다.\n요청을 받아 정해진 위치로 이동하고, 양팔로 용기를 고정해 따르고, 양을 판정하고 기록하는 흐름이 그대로 대응됩니다.');
}

async function s21() {
  newSlide();
  header({ num: '11', label: '한계와 향후 과제', title: '남은 과제와 다음 단계', orb: 'c' });
  const cards = [
    ['LuMove', '조작', C.main, '위치·조명 일반화', '학습 범위 밖 위치·조명에서 성공률이 떨어진다. 위치 격자를 넓힌 데이터와 이미지 증강 효과를 실물로 검증한다.'],
    ['LuDroplet', '인지', C.main, '물 양 폐루프 제어', '판정 신호를 붓기 정지와 직접 연결한다. 지금은 붓기 종료를 정책과 붓기 훅이 정한다.'],
    ['LuRepeat', '통합', C.main, '반복 신뢰성', '장시간 연속 주문 · 무선 두절 · 재시작 복구와 충전 시작 신호까지 포함해 평가한다.'],
    ['LuCoins', '사업화', C.acc, '비용 대비 활용도', '양팔을 더한 비용과 시간당 처리량을 실제 매장 기준으로 검증한다.'],
  ];
  const w = 5.97, h = 1.92, gx = CW - 2 * w;
  for (let i = 0; i < 4; i++) {
    const [ic, tag, col, t, desc] = cards[i];
    const x = ML + (i % 2) * (w + gx), y = 1.95 + Math.floor(i / 2) * (h + 0.2), a = { e: 'zoom', d: i * 200, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    await iconDot(ic, x + 0.62, y + 0.66, 0.74, col === C.acc ? C.accT : C.mainT, col, a);
    txt(tag, x + w - 1.4, y + 0.25, 1.15, 0.32, { shape: RR, r: 0.16, fill: col, color: C.white, bold: true, size: 11.5, align: 'center', valign: 'middle', check: false }, a);
    txt(t, x + 1.2, y + 0.42, w - 2.7, 0.46, { size: 19, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(desc, x + 0.32, y + 1.1, w - 0.6, 0.72, { size: 13, color: C.muted, ls: 1.15 }, a);
  }
  callout('다음 단계', '실물 반복 평가로 숫자를 쌓고, 물 양 신호와 충전 신호를 미션에 연결해 한 주문을 끝까지 자동으로 닫는다.', 6.27, { size: 13.5 }, { e: 'up', d: 1000, t: 0.5 });
  notes('남은 과제는 네 가지입니다.\n첫째, 조작 정책은 학습한 위치와 조명 범위를 벗어나면 성공률이 떨어집니다. 위치 격자를 넓힌 데이터와 이미지 증강 효과를 실물로 검증할 계획입니다.\n둘째, 물 양 판정 신호를 붓기 정지와 직접 연결하는 폐루프 제어가 남았습니다.\n셋째, 장시간 연속 주문과 무선 두절, 재시작 복구, 충전 시작 신호까지 포함한 반복 신뢰성 평가가 필요합니다.\n넷째, 사업화 관점에서 양팔 추가 비용 대비 처리량을 실제 매장 기준으로 검증해야 합니다.');
}

async function s22() {
  newSlide();
  header({ num: '12', label: '핵심 포인트', title: '오늘 꼭 기억해 주셨으면 하는 네 가지', orb: 'e' });
  const cards = [
    ['하드웨어', C.sub, C.ink, '4WD', '구동', '4륜 차체 + 양팔 결합 완성', '450×340 mm 차체에 양팔·RGB-D·LiDAR를 싣고 실제 서빙 형태로 운용'],
    ['주행', C.main, C.white, '12 mm', '1 cm 목표 실측', '1 cm 정밀주차와 도크 복귀', '도크 → 급수대 → 테이블 → 도크 연속 왕복'],
    ['조작', C.main, C.white, '−15%', '검증 손실 (60→80개)', '모방학습만으로 핸드오프 완주', 'SmolVLA-80 채택 · 데이터 확대로 검증 손실 12.5~15% 감소'],
    ['인지', C.acc, C.white, '93%', '판정 정확도', '물 양 판정 파이프라인', '목표량 도달 시 신호를 보내는 판정 구축'],
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
  callout('핵심 포인트', '주행·조작·인지·기구 네 파트가 한 주문 흐름으로 이어져, 물 한 잔을 테이블 위까지 가져간다.', 6.27, { color: C.main }, { e: 'up', d: 1000, t: 0.5 });
  notes('오늘 기억해 주셨으면 하는 네 가지입니다.\n하드웨어는 4륜 차체에 양팔을 결합해 실제 서빙 형태로 완성했고,\n주행은 1 cm 목표 정밀주차에서 실측 12 mm로 서고 도크까지 연속으로 복귀합니다.\n조작은 IK 없이 모방학습만으로 물따르기와 컵배치를 완주했고, 데이터를 늘려 검증 손실을 12.5에서 15% 낮췄습니다.\n인지는 판정 정확도 93%의 물 양 판정 파이프라인을 만들었습니다.\n(93%는 실측으로 교체할 값)');
}

async function s23() {
  newSlide();
  L.orbs('t', false, 'E3F4FD', 'CDEBFB');
  txt('13', ML, 0.47, 0.46, 0.3, { name: '!!chipnum', shape: pres.shapes.ROUNDED_RECTANGLE, r: 0.15, fill: C.main, color: C.white, bold: true, size: 11, align: 'center', valign: 'middle', check: false });
  txt('마무리', ML + 0.6, 0.47, 8.6, 0.3, { name: '!!chiplabel', font: FSB, size: 12.5, color: C.mainD, valign: 'middle', check: false });
  txt('함께 이야기하고 싶은 두 가지', ML, 0.88, 11.45, 0.72, { name: '!!title', size: 30, bold: true, color: C.ink, valign: 'middle', check: false });
  img(A('logo_mark_light_sky.png'), 12.29, 0.42, 0.44, 0.418, { name: '!!logo', alt: 'MS Team 로고' });
  L.pageNum();
  const qs = [['Q1', '데이터 vs 모델', '데이터를 늘릴까, 모델을 키울까', '같은 검증셋에서 SmolVLA는 데이터 확대로 손실이 12.5~15% 줄었습니다. 현장 적용 전 다음 투자를 데이터와 모델 중 어디에 둘지 의견을 듣고 싶습니다.'], ['Q2', '현장 운용', '모바일 매니퓰레이터의 매장 운용 조건', '팔을 더한 서빙 로봇의 기능 대비 비용과, 사람이 오가는 매장에서의 안전 운용 기준에 대한 조언을 구하고 싶습니다.']];
  const w = 5.97, gx = CW - 2 * w, y = 1.95, h = 2.75;
  qs.forEach((q, i) => {
    const x = ML + i * (w + gx), a = { e: 'up', d: i * 220, t: 0.5 };
    box(x, y, w, h, { r: 0.14, fill: C.white, line: C.line, shadow: 'soft' }, a);
    txt(q[0], x + 0.32, y + 0.25, 1.2, 0.72, { size: 40, bold: true, color: C.main, valign: 'middle', check: false }, a);
    txt(q[1], x + 1.45, y + 0.44, 2.5, 0.32, { size: 12.5, color: C.muted, valign: 'middle' }, a);
    txt(q[2], x + 0.32, y + 1.05, w - 0.6, 0.45, { size: 20, bold: true, color: C.ink, valign: 'middle' }, a);
    txt(q[3], x + 0.32, y + 1.58, w - 0.6, 1.0, { size: 13.5, color: C.muted, ls: 1.2 }, a);
  });
  txt('물 한 잔을, 준비부터 테이블 위까지. 한 주문을 끝까지 수행하는 로봇을 만들었습니다.', ML, 5.08, CW, 0.45, { size: 19, font: FSB, color: C.mainD }, { e: 'wipeR', d: 700, t: 0.9 });
  txt('감사합니다', ML, 5.68, 5, 0.75, { size: 36, bold: true, color: C.ink, valign: 'middle' }, { e: 'zoom', d: 1500, t: 0.5 });
  txt('github.com/mmporong/bimanual-robot', ML, 6.42, 6, 0.34, { size: 14.5, color: C.mainD, hyperlink: { url: 'https://github.com/mmporong/bimanual-robot' } }, { e: 'fade', d: 1700, t: 0.4 });
  img(A('logo_h_light_sky.png'), CR - 2.72, 5.9, 2.72, 0.558, { alt: 'MS Team 로고' }, { e: 'fade', d: 1700, t: 0.5 });
  notes('물 한 잔을, 준비부터 테이블 위까지.\n한 주문을 끝까지 수행하는 로봇을 만들었습니다.\n질문 시간에 두 가지를 함께 이야기하고 싶습니다. 데이터와 모델 중 어디에 더 투자할지, 그리고 매장 운용 조건입니다.\n모바일 서빙의 MS팀이었습니다. 감사합니다.');
}

module.exports = [s16, s17, s18, s19, s20, s21, s22, s23];
