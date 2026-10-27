"use strict";

// 손님용 주문 화면(식당 키오스크). 주문 API(/api/orders)와 상태 API(/api/state)만 사용한다.
// 로봇이 실제로 서빙할 수 있는 메뉴는 냉수·온수뿐이다. 나머지는 매장 분위기용으로 "준비 중"으로 보여 주고 누를 수 없다.
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수" };

// 64x64 그림은 order.html 의 <template id="art-이름"> 에 있다.
const artOf = (name) => document.getElementById(`art-${name}`).content.cloneNode(true);

// 매장 메뉴. drink 가 있는 것만 실제 주문(로봇 서빙)이 된다.
const MENU = [
  { id: "robot", tab: "로봇 서빙", live: true, title: "로봇 서빙", sub: "양팔 로봇이 직접 컵에 물을 따라 테이블까지 가져다드려요.",
    items: [
      { drink: "COLD_WATER", name: "얼음 냉수", desc: "시원한 물 한 잔", price: "무료", art: "cold" },
      { drink: "HOT_WATER", name: "따뜻한 온수", desc: "따뜻한 물 한 잔", price: "무료", art: "hot" },
    ] },
  { id: "main", tab: "식사", title: "식사", sub: "서빙 로봇이 곧 배울 메뉴예요.",
    items: [
      { name: "소고기 덮밥", desc: "간장 불고기, 반숙 계란", price: "11,000원", art: "bowl" },
      { name: "크림 파스타", desc: "베이컨, 버섯, 파르메산", price: "13,000원", art: "pasta" },
      { name: "안심 스테이크", desc: "구운 채소, 매시드 포테이토", price: "24,000원", art: "steak" },
      { name: "해물 리조또", desc: "새우, 홍합, 토마토 소스", price: "14,000원", art: "bowl" },
    ] },
  { id: "side", tab: "사이드", title: "사이드", sub: "서빙 로봇이 곧 배울 메뉴예요.",
    items: [
      { name: "시저 샐러드", desc: "로메인, 크루통, 시저 드레싱", price: "8,000원", art: "salad" },
      { name: "감자튀김", desc: "트러플 소금", price: "5,000원", art: "fries" },
      { name: "리코타 샐러드", desc: "제철 과일, 발사믹", price: "9,000원", art: "salad" },
    ] },
  { id: "drink", tab: "음료", title: "음료", sub: "서빙 로봇이 곧 배울 메뉴예요.",
    items: [
      { name: "수제 레몬에이드", desc: "레몬청, 탄산수", price: "6,000원", art: "lemon" },
      { name: "콜라", desc: "얼음 가득", price: "3,000원", art: "soda" },
      { name: "탄산수", desc: "라임 한 조각", price: "3,000원", art: "sparkling" },
    ] },
];
const ART_BY_DRINK = { COLD_WATER: "cold", HOT_WATER: "hot" };
const NAME_BY_DRINK = { COLD_WATER: "얼음 냉수", HOT_WATER: "따뜻한 온수" };

const STEPS = [
  { label: "주문 접수", title: "주문이 접수됐어요", sub: "로봇이 곧 출발합니다.", art: "receipt" },
  { label: "주방 이동", title: "로봇이 주방으로 가고 있어요", sub: "잠시만 기다려 주세요.", art: "robot" },
  { label: "따르기", title: "물을 따르고 있어요", sub: "양팔 로봇이 컵에 물을 담는 중입니다.", art: "pour" },
  { label: "테이블 이동", title: "테이블로 가고 있어요", sub: "물을 싣고 이동 중입니다.", art: "robot" },
  { label: "서빙", title: "곧 도착해요", sub: "로봇이 컵을 테이블 위에 놓습니다. 컵에 손대지 말아 주세요.", art: "pour" },
];
const STEP_BY_PHASE = {
  NAVIGATE_KITCHEN: 1, ALIGN_KITCHEN: 1,
  GRASP_CUP: 2, GRASP_BOTTLE: 2, POUR: 2, RETURN_BOTTLE: 2, PLACE_DECK: 2,
  NAVIGATE_TABLE: 3, ALIGN_TABLE: 3,
  REGRASP_CUP: 4, SERVE: 4,
};
const STORAGE_KEY = "bimanual-robot.order";

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const fixedTable = normalizeTable(params.get("table"));
let selectedTable = fixedTable;
let selectedDrink = null;
let activeTab = MENU[0].id;
const linkedOrder = params.get("order");
let currentOrder = linkedOrder ? { order_id: linkedOrder } : loadSaved();
let pollTimer = null;

function normalizeTable(value) {
  if (!value) return null;
  const id = /^\d+$/.test(value) ? `table_${value}` : value;
  return /^table_\d+$/.test(id) ? id : null;
}
function tableName(id) { return `${id.replace("table_", "")}번 테이블`; }
// 주문 번호 끝의 임의 4자리를 영수증 번호처럼 크게 보여 준다.
function ticketNo(orderId) { return (orderId.split("-").pop() || orderId).slice(-4).toUpperCase(); }

function loadSaved() {
  try { return JSON.parse(localStorage.getItem(STORAGE_KEY) || "null"); } catch { return null; }
}
function save(order) {
  try {
    if (order) localStorage.setItem(STORAGE_KEY, JSON.stringify(order));
    else localStorage.removeItem(STORAGE_KEY);
  } catch { /* 저장 불가 환경에서는 새로고침 복구만 빠진다 */ }
}

// 주소에 주문 번호를 남겨 새로고침·링크 공유 때도 같은 주문을 계속 추적한다.
function setOrderParam(orderId) {
  const url = new URL(location.href);
  if (orderId) url.searchParams.set("order", orderId);
  else url.searchParams.delete("order");
  history.replaceState(null, "", url);
}

function newOrderId() {
  const now = new Date();
  const p = (n) => String(n).padStart(2, "0");
  const stamp = `${now.getFullYear()}${p(now.getMonth() + 1)}${p(now.getDate())}-${p(now.getHours())}${p(now.getMinutes())}${p(now.getSeconds())}`;
  return `WEB-${stamp}-${Math.random().toString(36).slice(2, 6)}`;
}

const api = (path, options) => window.orderApi(path, options);

// ---------- 메뉴 화면 ----------
function renderTabs() {
  $("tabs").replaceChildren(...MENU.map((section) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "tab";
    b.setAttribute("role", "tab");
    b.setAttribute("aria-selected", String(section.id === activeTab));
    if (section.live) {
      const dot = document.createElement("span");
      dot.className = "dot";
      b.append(dot);
    }
    b.append(section.tab);
    b.addEventListener("click", () => { activeTab = section.id; renderTabs(); renderItems(); });
    return b;
  }));
}

function renderItems() {
  const section = MENU.find((s) => s.id === activeTab);
  $("section-title").textContent = section.title;
  $("section-sub").textContent = section.sub;
  $("items").classList.toggle("feature", Boolean(section.live));
  $("items").replaceChildren(...section.items.map((item) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "item";
    const art = document.createElement("div");
    art.className = "art";
    art.append(artOf(item.art));
    const name = document.createElement("div");
    name.className = "name";
    name.textContent = item.name;
    const desc = document.createElement("div");
    desc.className = "desc";
    desc.textContent = item.desc;
    const foot = document.createElement("div");
    foot.className = "foot";
    const price = document.createElement("span");
    price.className = "price";
    price.textContent = item.price;
    const badge = document.createElement("span");
    badge.className = `badge ${item.drink ? "robot" : "soon"}`;
    badge.textContent = item.drink ? "로봇 서빙" : "준비 중";
    foot.append(price, badge);
    b.append(art, name, desc, foot);
    if (item.drink) {
      b.dataset.drink = item.drink;
      b.addEventListener("click", () => { selectedDrink = item.drink; refreshForm(); });
    } else {
      b.classList.add("disabled");
      b.disabled = true;
      b.setAttribute("aria-label", `${item.name} (준비 중)`);
    }
    return b;
  }));
  refreshForm();
}

function renderTables(tables) {
  $("tables").replaceChildren(...tables.map((t) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "table-btn";
    b.dataset.table = t.id;
    b.textContent = t.id.replace("table_", "");
    b.setAttribute("aria-label", tableName(t.id));
    b.addEventListener("click", () => { selectedTable = t.id; refreshForm(); });
    return b;
  }));
  refreshForm();
}

function refreshForm() {
  document.querySelectorAll("[data-table]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.table === selectedTable)));
  document.querySelectorAll("[data-drink]").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.drink === selectedDrink)));
  $("table-label").textContent = selectedTable ? tableName(selectedTable) : "";
  $("cart-what").textContent = selectedDrink ? `${NAME_BY_DRINK[selectedDrink]} 1잔 · 무료` : "메뉴를 골라 주세요";
  $("cart-hint").textContent = !selectedTable ? "테이블을 먼저 골라 주세요" : "로봇은 한 번에 한 잔씩 서빙해요";
  $("submit").disabled = !(selectedTable && selectedDrink);
}

function openConfirm() {
  $("confirm-art").replaceChildren(artOf(ART_BY_DRINK[selectedDrink]));
  $("confirm-title").textContent = `${NAME_BY_DRINK[selectedDrink]} 1잔`;
  $("confirm-sub").textContent = `${tableName(selectedTable)}로 로봇이 가져다드려요.`;
  $("confirm").hidden = false;
  $("confirm-ok").focus();
}

async function submitOrder() {
  const button = $("confirm-ok");
  button.disabled = true;
  button.textContent = "주문 보내는 중…";
  $("order-error").textContent = "";
  const order = { order_id: newOrderId(), table_id: selectedTable, drink: selectedDrink };
  try {
    await api("/api/orders", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(order) });
    currentOrder = order;
    save(order);
    setOrderParam(order.order_id);
    $("confirm").hidden = true;
    showTracking();
  } catch (error) {
    $("confirm").hidden = true;
    $("order-error").textContent = error.status === 400 ? "주문할 수 없는 테이블 또는 메뉴예요." : "주문을 보내지 못했어요. 다시 눌러 주세요.";
  } finally {
    button.disabled = false;
    button.textContent = "주문 확정";
    refreshForm();
  }
}

// ---------- 진행 화면 ----------
function showOrderForm() {
  currentOrder = null;
  save(null);
  setOrderParam(null);
  clearInterval(pollTimer);
  selectedDrink = null;
  activeTab = MENU[0].id;
  $("track-view").hidden = true;
  $("order-view").hidden = false;
  renderTabs();
  renderItems();
}

function showTracking() {
  $("order-view").hidden = true;
  $("track-view").hidden = false;
  renderSummary();
  poll();
  clearInterval(pollTimer);
  pollTimer = setInterval(poll, 1000);
}

async function poll() {
  try {
    const state = await api("/api/state");
    $("offline").hidden = true;
    renderStatus(state);
  } catch {
    $("offline").hidden = false;
  }
}

function renderSummary() {
  $("table-label").textContent = currentOrder.table_id ? tableName(currentOrder.table_id) : "";
  $("track-no").textContent = ticketNo(currentOrder.order_id);
  $("track-drink").textContent = currentOrder.drink ? `${NAME_BY_DRINK[currentOrder.drink] || DRINKS[currentOrder.drink] || currentOrder.drink} 1잔` : "";
  $("track-id").textContent = `주문 번호 ${currentOrder.order_id}`;
}

function renderStatus(state) {
  const order = state.orders.find((o) => o.order_id === currentOrder.order_id);
  if (order && !currentOrder.table_id) {
    currentOrder = { order_id: order.order_id, table_id: order.table_id, drink: order.drink };
    save(currentOrder);
    renderSummary();
  }
  const card = $("status-card");
  card.className = "card";
  $("cancel").hidden = true;
  $("again").hidden = true;

  if (!order) {
    setStatus("주문을 찾을 수 없어요", "서버가 초기화되었을 수 있어요. 다시 주문해 주세요.", -1, "fail");
    card.classList.add("fail");
    $("again").hidden = false;
    return;
  }
  if (order.state === "QUEUED") {
    const ahead = state.queue.indexOf(order.order_id) + (state.active_order_id ? 1 : 0);
    setStatus(STEPS[0].title, ahead > 0 ? `앞에 ${ahead}건의 주문이 있어요. 순서대로 준비합니다.` : STEPS[0].sub, 0, STEPS[0].art);
    $("cancel").hidden = false;
  } else if (order.state === "RUNNING") {
    const step = STEP_BY_PHASE[state.phase] ?? 1;
    setStatus(STEPS[step].title, STEPS[step].sub, step, STEPS[step].art);
  } else if (order.state === "SUCCEEDED") {
    setStatus("물이 도착했어요", "맛있게 드세요!", STEPS.length, "done");
    card.classList.add("ok");
    $("again").hidden = false;
    clearInterval(pollTimer);
  } else if (order.state === "CANCELED") {
    setStatus("주문이 취소됐어요", "", -1, "fail");
    $("again").hidden = false;
    clearInterval(pollTimer);
  } else {
    setStatus("준비 중 문제가 생겼어요", "직원이 확인 후 직접 가져다드릴게요.", -1, "fail");
    card.classList.add("fail");
    $("again").hidden = false;
    clearInterval(pollTimer);
  }
}

let heroArt = null;
function setStatus(title, sub, step, art) {
  $("status-title").textContent = title;
  $("status-sub").textContent = sub;
  if (art !== heroArt) { $("hero").replaceChildren(artOf(art)); heroArt = art; }
  $("hero").classList.toggle("live", step >= 0 && step < STEPS.length);
  $("steps").replaceChildren(...STEPS.map((s, i) => {
    const li = document.createElement("li");
    li.textContent = s.label;
    if (step >= 0 && i < step) li.className = "done";
    else if (i === step) li.className = "now";
    return li;
  }));
}

async function cancelOrder() {
  if (!confirm("주문을 취소할까요?")) return;
  try {
    await api(`/api/orders/${encodeURIComponent(currentOrder.order_id)}/cancel`, { method: "POST" });
    poll();
  } catch {
    alert("취소하지 못했어요. 이미 준비가 시작됐을 수 있어요.");
  }
}

// ---------- 시작 ----------
$("submit").addEventListener("click", openConfirm);
$("confirm-ok").addEventListener("click", submitOrder);
$("confirm-cancel").addEventListener("click", () => { $("confirm").hidden = true; });
$("confirm").addEventListener("click", (e) => { if (e.target === $("confirm")) $("confirm").hidden = true; });
document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("confirm").hidden = true; });
$("cancel").addEventListener("click", cancelOrder);
$("again").addEventListener("click", showOrderForm);
if (fixedTable) $("table-section").hidden = true;
$("demo-note").hidden = window.orderApiMode !== "demo";
renderTabs();
renderItems();

api("/api/state")
  .then((state) => {
    const tables = state.restaurant.tables;
    if (fixedTable && !tables.some((t) => t.id === fixedTable)) {
      selectedTable = null;
      $("table-section").hidden = false;
    }
    renderTables(tables);
  })
  .catch(() => { $("order-error").textContent = "서버에 연결할 수 없어요."; });

if (currentOrder) showTracking();
