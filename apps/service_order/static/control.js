"use strict";

// 운영자용 관제 화면. /api/state를 1초마다 읽고, 주문·취소·일시정지·시험 단계 API를 호출한다.
const SERVICE_PHASES = [
  ["NAVIGATE_KITCHEN", "주방 이동"], ["ALIGN_KITCHEN", "주방 정렬"], ["GRASP_CUP", "컵 파지"],
  ["GRASP_BOTTLE", "물통 파지"], ["POUR", "물 따르기"], ["RETURN_BOTTLE", "물통 반환"],
  ["PLACE_DECK", "상판 적재"], ["NAVIGATE_TABLE", "테이블 이동"], ["ALIGN_TABLE", "테이블 정렬"],
  ["REGRASP_CUP", "컵 재파지"], ["SERVE", "테이블 배치"],
];
const PHASE_LABEL = Object.fromEntries([
  ...SERVICE_PHASES,
  ["IDLE_AT_DOCK", "충전소 대기"], ["NAVIGATE_DOCK", "충전소 복귀"],
  ["CHARGING", "충전 중"], ["TERMINAL_HOLD", "정지 대기"],
]);
const MISSION_LABEL = { RUNNING: "작업 중", IDLE: "대기", CHARGING: "충전 중", TERMINAL: "정지 대기" };
const STATE_LABEL = { QUEUED: "대기", RUNNING: "진행 중", SUCCEEDED: "완료", FAILED: "실패", CANCELED: "취소" };
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수" };
const EVENT_LABEL = {
  ORDER_QUEUED: "주문 접수", ORDER_CANCELED: "주문 취소", ORDER_DUPLICATE_IGNORED: "중복 주문 무시",
  MISSION_STARTED: "작업 시작", MISSION_SUCCEEDED: "작업 완료", MISSION_FAILED: "작업 실패",
  PHASE_SUCCEEDED: "단계 완료", PHASE_RETRY: "단계 재시도", SYSTEM_PHASE_FAILED: "시스템 단계 실패",
  CHARGE_REQUIRED: "충전 필요", CHARGE_COMPLETED: "충전 완료", RETURN_REQUIRED_FOR_CHARGE: "충전 복귀 필요",
  RETURN_TO_DOCK_STARTED: "충전소 복귀 시작", DOCK_ARRIVED: "충전소 도착",
  CONTROLLER_STARTED: "관제 시작", CONTROLLER_RESTORED: "관제 상태 복구", ROUNDTRIP_TERMINAL_HOLD: "정지 대기 전환",
};

const $ = (id) => document.getElementById(id);
let state = null;

const place = (id) => !id ? "-" : id === "dock" ? "충전소" : id === "kitchen" ? "주방" : /^table_\d+$/.test(id) ? `${id.slice(6)}번 테이블` : id;
const time = (iso) => iso ? new Date(iso).toLocaleTimeString("ko-KR", { hour12: false }) : "-";

function el(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}
function row(cells) {
  const tr = document.createElement("tr");
  cells.forEach((c) => { const td = document.createElement("td"); c instanceof Node ? td.append(c) : (td.textContent = c); tr.append(td); });
  return tr;
}
function emptyRow(text, span) {
  const tr = document.createElement("tr");
  const td = el("td", text, "empty");
  td.colSpan = span;
  tr.append(td);
  return tr;
}
function failureText(failure) {
  if (!failure) return "-";
  const [phase, code] = failure.split(":");
  return code ? `${PHASE_LABEL[phase] || phase} · ${code}` : failure;
}
function badge(orderState) { return el("span", STATE_LABEL[orderState] || orderState, `badge ${orderState}`); }

async function api(path, payload) {
  const options = {};
  if (payload !== undefined) {
    options.method = "POST";
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(payload);
  }
  return window.orderApi(path, options);
}

async function refresh() {
  try {
    state = await api("/api/state");
    $("conn").textContent = `마지막 갱신 ${time(state.timestamp)}`;
    $("conn").className = "conn";
    render();
  } catch {
    $("conn").textContent = "서버 연결 끊김 — 재연결 중";
    $("conn").className = "conn bad";
  }
}

function render() {
  const orders = new Map(state.orders.map((o) => [o.order_id, o]));
  const active = state.active_order_id ? orders.get(state.active_order_id) : null;
  const mode = state.execution_backend?.mode || "-";

  $("s-mission").textContent = MISSION_LABEL[state.mission_state] || state.mission_state;
  $("s-location").textContent = place(state.current_location);
  $("s-battery").textContent = `${state.battery_percent.toFixed(1)}%`;
  $("s-mode").textContent = mode;
  $("s-mode").title = mode;

  $("pause").textContent = state.auto_paused ? "자동 진행 재개" : "자동 진행 일시정지";
  $("pause").className = state.auto_paused ? "btn primary" : "btn";
  $("paused-banner").hidden = !state.auto_paused;
  $("abort").disabled = !active;
  $("test-tools").hidden = !["simulation_dry_run", "browser_demo"].includes(mode);
  $("t-reset").hidden = mode !== "browser_demo";

  // 현재 작업
  const head = $("current-head");
  if (active) {
    head.replaceChildren(
      el("b", `${place(active.table_id)} · ${DRINKS[active.drink] || active.drink}`),
      el("span", `주문 ${active.order_id}`, "muted"),
      el("span", `단계: ${PHASE_LABEL[state.phase] || state.phase}${state.phase_attempt > 1 ? ` (${state.phase_attempt}번째 시도)` : ""}`),
    );
  } else {
    head.replaceChildren(el("span", `진행 중인 주문 없음 — ${PHASE_LABEL[state.phase] || state.phase}`, "muted"));
  }
  const nowIndex = active ? SERVICE_PHASES.findIndex(([key]) => key === state.phase) : -1;
  $("phases").replaceChildren(...SERVICE_PHASES.map(([, label], i) => {
    const cls = nowIndex < 0 ? "phase" : i < nowIndex ? "phase done" : i === nowIndex ? "phase now" : "phase";
    return el("div", label, cls);
  }));

  // 대기 주문
  $("queue-count").textContent = state.queue.length ? `${state.queue.length}건` : "";
  const queueRows = state.queue.map((id, i) => {
    const o = orders.get(id);
    const cancel = el("button", "취소", "btn small");
    cancel.type = "button";
    cancel.addEventListener("click", () => cancelOrder(id));
    return row([String(i + 1), id, place(o.table_id), DRINKS[o.drink] || o.drink, time(o.created_at), cancel]);
  });
  $("queue").replaceChildren(...(queueRows.length ? queueRows : [emptyRow("대기 중인 주문이 없습니다.", 6)]));

  // 최근 주문(완료·실패·취소)
  const done = state.orders.filter((o) => ["SUCCEEDED", "FAILED", "CANCELED"].includes(o.state)).reverse().slice(0, 10);
  $("history").replaceChildren(...(done.length
    ? done.map((o) => row([o.order_id, place(o.table_id), DRINKS[o.drink] || o.drink, badge(o.state), time(o.completed_at), failureText(o.failure)]))
    : [emptyRow("아직 끝난 주문이 없습니다.", 6)]));

  // 이벤트
  $("events").replaceChildren(...state.events.slice(-30).reverse().map((e) => {
    const phase = e.completed_phase || e.failed_phase;
    const extra = [e.order_id, phase && PHASE_LABEL[phase], e.failure].filter(Boolean).join(" · ");
    return el("li", `${time(e.timestamp)}  ${EVENT_LABEL[e.kind] || e.kind}${extra ? ` — ${extra}` : ""}`);
  }));

  if (!$("f-table").options.length) {
    const tables = state.restaurant.tables.map((t) => t.id);
    $("f-table").replaceChildren(...tables.map((id) => { const o = el("option", place(id)); o.value = id; return o; }));
    $("links").replaceChildren(...tables.map((id) => {
      const li = document.createElement("li");
      const a = el("a", `${place(id)} 주문 화면`);
      a.href = `/order?table=${id.slice(6)}`;
      a.target = "_blank";
      li.append(a);
      return li;
    }));
  }
}

async function act(fn) {
  try { await fn(); } catch (error) { alert(`요청 실패: ${error.message}`); }
  refresh();
}
function cancelOrder(id) {
  if (confirm(`주문 ${id}을(를) 취소할까요?`)) act(() => api(`/api/orders/${encodeURIComponent(id)}/cancel`, {}));
}

$("pause").addEventListener("click", () => act(() => api("/api/control", { paused: !state?.auto_paused })));
$("abort").addEventListener("click", () => {
  const id = state?.active_order_id;
  if (id && confirm(`진행 중인 주문 ${id}을(를) 중단할까요?\n로봇은 다음 주문 또는 충전소 복귀로 넘어갑니다.`)) {
    act(() => api(`/api/orders/${encodeURIComponent(id)}/cancel`, {}));
  }
});
$("f-submit").addEventListener("click", () => act(async () => {
  const d = new Date();
  const two = (n) => String(n).padStart(2, "0");
  const stamp = `${d.getFullYear()}${two(d.getMonth() + 1)}${two(d.getDate())}-${two(d.getHours())}${two(d.getMinutes())}${two(d.getSeconds())}`;
  const order = { order_id: `STAFF-${stamp}-${Math.random().toString(36).slice(2, 6)}`, table_id: $("f-table").value, drink: $("f-drink").value };
  await api("/api/orders", order);
  $("f-msg").textContent = `${place(order.table_id)} ${DRINKS[order.drink]} 주문을 등록했습니다.`;
}));
$("demo-note").hidden = window.orderApiMode !== "demo";
$("t-reset").addEventListener("click", () => {
  if (confirm("데모 주문과 기록을 모두 지울까요?")) window.resetDemo().then(() => location.reload());
});
$("t-step").addEventListener("click", () => act(() => api("/api/step", { success: true })));
$("t-fail").addEventListener("click", () => act(() => api("/api/step", { success: false, failure: "OPERATOR_INJECTED" })));

refresh();
setInterval(refresh, 1000);
