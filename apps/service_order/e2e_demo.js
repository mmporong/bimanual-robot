// 배포된 데모 화면 E2E 확인: 손님 메뉴 -> 주문 확인 창 -> 관제 표시 -> 일시정지/한 단계 진행 -> 대리 주문·취소 -> 완료 -> 손님 취소.
// 준비: 빈 폴더에서 `npm i puppeteer-core@23.11.1` 후 이 파일을 복사해 실행한다(Edge 사용, Windows 경로).
// 실행: node e2e_demo.js [사이트 주소] [스크린샷 폴더]
const os = require("os");
const puppeteer = require("puppeteer-core");
const BASE = process.argv[2] || "https://ms-water-serving.netlify.app";
const OUT = process.argv[3] || os.tmpdir();
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const text = (page, sel) => page.$eval(sel, (e) => e.textContent.trim());
const ok = (cond, msg) => { console.log(`${cond ? "PASS" : "FAIL"} ${msg}`); if (!cond) process.exitCode = 1; };

(async () => {
  const browser = await puppeteer.launch({
    executablePath: "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
    headless: true, args: ["--no-first-run"],
  });
  const guest = await browser.newPage();
  await guest.setViewport({ width: 375, height: 760, isMobile: true, hasTouch: true });
  guest.on("dialog", (d) => d.accept());
  await guest.goto(`${BASE}/control`, { waitUntil: "networkidle0" });
  await guest.evaluate(() => window.resetDemo());

  await guest.goto(`${BASE}/order?table=3`, { waitUntil: "networkidle0" });
  ok(await guest.$eval("#submit", (b) => b.disabled), "주문 버튼은 음료 선택 전 비활성");
  ok((await text(guest, "#table-label")) === "3번 테이블", "QR 테이블 번호 표시");
  ok(await guest.evaluate(() => document.documentElement.scrollWidth <= innerWidth), "375px 가로 넘침 없음");
  ok((await guest.$$(".item.disabled")).length === 0 && (await guest.$$("[data-drink]")).length === 2, "첫 화면은 로봇 서빙 메뉴 2개");
  await guest.screenshot({ path: `${OUT}/0_guest_menu.png` });
  await guest.click('[data-drink="COLD_WATER"]');
  await guest.click("#submit");
  await guest.waitForSelector("#confirm:not([hidden])");
  await guest.click("#confirm-ok");
  await guest.waitForSelector("#track-view:not([hidden])");
  ok(guest.url().includes("order=WEB-"), `주소에 주문 번호 기록 (${guest.url().split("?")[1]})`);
  await guest.screenshot({ path: `${OUT}/1_guest_tracking.png` });

  const control = await browser.newPage();
  await control.setViewport({ width: 1280, height: 900 });
  control.on("dialog", (d) => d.accept());
  await control.goto(`${BASE}/control`, { waitUntil: "networkidle0" });
  await sleep(1200);
  ok((await text(control, "#current-head")).includes("3번 테이블"), "관제 화면에 현재 주문 표시");
  await control.click("#pause");
  await sleep(1200);
  ok(!(await control.$eval("#paused-banner", (e) => e.hidden)), "일시정지 배너 표시");
  const phaseA = await text(control, "#current-head");
  await sleep(6000);
  ok(phaseA === (await text(control, "#current-head")), "일시정지 중 단계 멈춤");
  await control.click("#t-step");
  await sleep(1200);
  ok(phaseA !== (await text(control, "#current-head")), "시험 도구: 한 단계 진행");
  await control.screenshot({ path: `${OUT}/2_control_paused.png` });

  // 직원 대리 주문 2건 -> 대기열, 하나는 관제에서 취소
  await control.select("#f-table", "table_1");
  await control.click("#f-submit");
  await sleep(300);
  await control.select("#f-table", "table_2");
  await control.click("#f-submit");
  await sleep(1200);
  ok((await control.$$("#queue tr button")).length === 2, "대리 주문 2건 대기열");
  await (await control.$$("#queue tr button"))[1].click();
  await sleep(1200);
  ok((await control.$$("#queue tr button")).length === 1, "관제에서 대기 주문 취소");

  await control.click("#pause"); // 재개
  await guest.bringToFront();
  const deadline = Date.now() + 60000;
  while (Date.now() < deadline && (await text(guest, "#status-title")) !== "물이 도착했어요") await sleep(1000);
  const done = (await text(guest, "#status-title")) === "물이 도착했어요";
  ok(done, "손님 화면 완료 표시");
  if (!done) {
    console.log("guest:", await text(guest, "#status-title"), "|", await text(guest, "#status-sub"));
    console.log("demo:", await guest.evaluate(async () => { const s = await window.orderApi("/api/state"); return JSON.stringify({ phase: s.phase, paused: s.auto_paused, active: s.active_order_id, orders: s.orders.map((o) => o.state) }); }));
    await browser.close(); return;
  }
  await guest.screenshot({ path: `${OUT}/3_guest_done.png` });

  // 새 주문 -> 대기 중 손님 취소 (앞 주문이 진행 중이어야 대기 상태가 된다)
  await guest.click("#again");
  await guest.click('[data-drink="HOT_WATER"]');
  await guest.click("#submit");
  await guest.waitForSelector("#confirm:not([hidden])");
  await guest.click("#confirm-ok");
  await guest.waitForSelector("#track-view:not([hidden])");
  await sleep(1200);
  const queuedSub = await text(guest, "#status-sub");
  ok(queuedSub.includes("앞에"), `대기 순서 안내 (${queuedSub})`);
  await guest.click("#cancel");
  await sleep(1500);
  ok((await text(guest, "#status-title")) === "주문이 취소됐어요", "손님 대기 주문 취소");

  await control.bringToFront();
  await sleep(1500);
  const history = await text(control, "#history");
  ok(history.includes("완료") && history.includes("취소"), "관제 최근 주문에 완료·취소 기록");
  await control.screenshot({ path: `${OUT}/4_control_history.png`, fullPage: true });
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
