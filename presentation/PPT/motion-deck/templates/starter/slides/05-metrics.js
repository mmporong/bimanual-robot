'use strict';
// Results: native (editable) bar chart that wipes up + three stat tiles + a filled progress bar
const L = require('motion-deck');
const { pres, C, FR, ML, CR, txt, box, header, callout, newSlide, bigNum } = L;

module.exports = async function metrics() {
  newSlide();
  header({ num: '04', label: '결과 — 핵심 지표', title: '목표 대비 무엇을 달성했는가', orb: 'c' });
  const cats = ['1차', '2차', '3차', '최종'];
  L.S.slide.addChart(pres.charts.BAR, [{ name: '성공률', labels: cats, values: [40, 55, 70, 90] }], {
    x: ML, y: 1.92, w: 5.75, h: 3.9, objectName: 'm_chart', barDir: 'col', barGapWidthPct: 55,
    chartColors: [C.main], showLegend: false,
    showTitle: true, title: '반복별 성공률(%) — 예시 데이터', titleFontFace: FR, titleFontSize: 13, titleColor: C.ink, titleBold: true,
    showValue: true, dataLabelPosition: 'inEnd', dataLabelColor: C.white, dataLabelFontFace: FR, dataLabelFontSize: 12, dataLabelFontBold: true,
    catAxisLabelFontFace: FR, catAxisLabelFontSize: 11.5, catAxisLabelColor: C.muted, catAxisLineShow: true, catAxisLineColor: 'B9CCDA',
    valAxisLabelFontFace: FR, valAxisLabelFontSize: 10, valAxisLabelColor: C.faint, valAxisLineShow: false, valAxisMaxVal: 100, valAxisMajorUnit: 20,
    valGridLine: { color: 'E6F0F7', size: 0.75 }, catGridLine: { style: 'none' },
  });
  L.anim('m_chart', { e: 'wipeU', d: 0, t: 0.9 });
  const x = 6.75, w = CR - x;
  const stats = [['00', '%', '목표 지표 1 — 무엇을 셌는지'], ['0.0', '초', '목표 지표 2 — 측정 조건'], ['0', '회', '목표 지표 3 — 기준과 비교']];
  stats.forEach((s, i) => {
    const y = 1.98 + i * 1.08, a = { e: 'zoom', d: 300 + i * 170, t: 0.45 };
    box(x, y, w, 0.94, { r: 0.12, fill: i === 0 ? C.main : C.mainT }, a);
    txt(bigNum(s[0], s[1], 30, i === 0 ? C.white : C.main), x + 0.3, y + 0.1, 2.4, 0.74, { valign: 'middle', check: false }, a);
    txt(s[2], x + 2.6, y + 0.1, w - 2.85, 0.74, { size: 14, color: i === 0 ? C.white : C.text, valign: 'middle' }, a);
  });
  const by = 5.3, a2 = { e: 'fade', d: 900, t: 0.3 };
  txt('목표 달성도', x, by - 0.05, 1.25, 0.3, { size: 12.5, color: C.muted }, a2);
  box(x + 1.4, by, w - 1.4, 0.22, { r: 0.11, fill: C.line }, a2);
  box(x + 1.4, by, (w - 1.4) * 0.9, 0.22, { r: 0.11, fill: C.main }, { e: 'wipeR', d: 1000, t: 0.9 });
  callout('결론', '수치가 말해 주는 한 문장 — 무엇이 되고 무엇이 남았는지', 6.15, {}, { e: 'up', d: 1350, t: 0.5 });
};
