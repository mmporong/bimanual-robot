"use strict";

// 손님용 주문 화면. 주문 API(/api/orders)와 상태 API(/api/state)만 사용한다.
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수" };
const STEPS = [
  { label: "주문 접수", title: "주문이 접수됐어요", sub: "로봇이 곧 출발합니다." },
  { label: "주방으로 이동", title: "로봇이 주방으로 가고 있어요", sub: "잠시만 기다려 주세요." },
  { label: "물 따르기", title: "물을 따르고 있어요", sub: "양팔 로봇이 컵에 물을 담는 중입니다." },
  { label: "테이블로 이동", title: "테이블로 가고 있어요", sub: "물을 싣고 이동 중입니다." },
  { label: "테이블에 놓기", title: "곧 도착해요", sub: "로봇이 컵을 테이블 위에 놓습니다. 컵에 손대지 말아 주세요." },
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
const linkedOrder = params.get("order");
let currentOrder = linkedOrder ? { order_id: linkedOrder } : loadSaved();
let pollTimer = null;

function normalizeTable(value) {
  if (!value) return null;
  const id = /^\d+$/.test(value) ? `table_${value}` : value;
  return /^table_\d+$/.test(id) ? id : null;
}
function tableName(id) { return `${id.replace("table_", "")}번 테이블`; }

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

// ---------- 주문 화면 ----------
function renderTables(tables) {
  const box = $("tables");
  box.replaceChildren(...tables.map((t) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "choice";
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
  $("submit").disabled = !(selectedTable && selectedDrink);
}

async function submitOrder() {
  const button = $("submit");
  button.disabled = true;
  button.textContent = "주문 보내는 중…";
  $("order-error").textContent = "";
  const order = { order_id: newOrderId(), table_id: selectedTable, drink: selectedDrink };
  try {
    await api("/api/orders", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(order) });
    currentOrder = order;
    save(order);
    setOrderParam(order.order_id);
    showTracking();
  } catch (error) {
    $("order-error").textContent = error.status === 400 ? "주문할 수 없는 테이블 또는 음료예요." : "주문을 보내지 못했어요. 다시 눌러 주세요.";
  } finally {
    button.textContent = "주문하기";
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
  $("track-view").hidden = true;
  $("order-view").hidden = false;
  refreshForm();
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
  const table = currentOrder.table_id ? tableName(currentOrder.table_id) : "";
  $("table-label").textContent = table;
  $("track-drink").textContent = currentOrder.drink ? `${DRINKS[currentOrder.drink] || currentOrder.drink} 한 잔` : "";
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
    setStatus("주문을 찾을 수 없어요", "서버가 초기화되었을 수 있어요. 다시 주문해 주세요.", -1);
    card.classList.add("fail");
    $("again").hidden = false;
    return;
  }
  if (order.state === "QUEUED") {
    const ahead = state.queue.indexOf(order.order_id) + (state.active_order_id ? 1 : 0);
    setStatus(STEPS[0].title, ahead > 0 ? `앞에 ${ahead}건의 주문이 있어요. 순서대로 준비합니다.` : STEPS[0].sub, 0);
    $("cancel").hidden = false;
  } else if (order.state === "RUNNING") {
    const step = STEP_BY_PHASE[state.phase] ?? 1;
    setStatus(STEPS[step].title, STEPS[step].sub, step);
  } else if (order.state === "SUCCEEDED") {
    setStatus("물이 도착했어요", "맛있게 드세요!", STEPS.length);
    card.classList.add("ok");
    $("again").hidden = false;
    clearInterval(pollTimer);
  } else if (order.state === "CANCELED") {
    setStatus("주문이 취소됐어요", "", -1);
    $("again").hidden = false;
    clearInterval(pollTimer);
  } else {
    setStatus("준비 중 문제가 생겼어요", "직원이 확인 후 직접 가져다드릴게요.", -1);
    card.classList.add("fail");
    $("again").hidden = false;
    clearInterval(pollTimer);
  }
}

function setStatus(title, sub, step) {
  $("status-title").textContent = title;
  $("status-sub").textContent = sub;
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
document.querySelectorAll("[data-drink]").forEach((b) => b.addEventListener("click", () => { selectedDrink = b.dataset.drink; refreshForm(); }));
$("submit").addEventListener("click", submitOrder);
$("cancel").addEventListener("click", cancelOrder);
$("again").addEventListener("click", showOrderForm);
if (fixedTable) $("table-section").hidden = true;
$("demo-note").hidden = window.orderApiMode !== "demo";

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
