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
const DRINKS = { COLD_WATER: "냉수", HOT_WATER: "온수", LEMONADE: "레몬에이드", COLA: "콜라" };
const MANIP_LABEL = { loading: "준비 중", ready: "대기", running: "실행 중", closing: "종료 중", unreachable: "연결 안 됨" };
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
const itemsText = (ticket) => ticket ? ticket.items.filter((i) => !i.robot).map((i) => `${i.name}×${i.qty}`).join(", ") : "";
function drinkText(o, tickets) {
  const food = itemsText(tickets.get(o.order_id));
  return `${DRINKS[o.drink] || o.drink}${food ? ` + ${food}` : ""}`;
}
// 주문 번호 표시: 9자리(YYMMDDNNN)가 아닌 예전 번호(WEB-…, STAFF-…, 시험용)도 같은 형식으로 보여 준다.
// 접수 날짜별로 시간순으로, 그날 이미 쓰인 9자리 번호를 건너뛰며 남는 번호를 붙인다(화면 표시만, 서버 기록은 그대로).
let displayNo = new Map();
const ORDER_NO = /^\d{9}$/;
function buildDisplayNumbers(orders, tickets) {
  const all = new Map();
  for (const o of [...orders, ...tickets]) if (!all.has(o.order_id)) all.set(o.order_id, o.created_at);
  const used = new Map(); // YYMMDD -> 쓰인 일련번호
  const legacy = [];
  for (const [id, created] of all) {
    if (ORDER_NO.test(id)) {
      if (!used.has(id.slice(0, 6))) used.set(id.slice(0, 6), new Set());
      used.get(id.slice(0, 6)).add(Number(id.slice(6)));
    } else legacy.push([id, created ? new Date(created) : new Date()]);
  }
  legacy.sort((a, b) => a[1] - b[1]);
  const map = new Map();
  const two = (n) => String(n).padStart(2, "0");
  for (const [id, when] of legacy) {
    const day = `${String(when.getFullYear()).slice(2)}${two(when.getMonth() + 1)}${two(when.getDate())}`;
    if (!used.has(day)) used.set(day, new Set());
    const taken = used.get(day);
    let n = 1;
    while (taken.has(n)) n += 1;
    taken.add(n);
    map.set(id, `${day}${String(n).padStart(3, "0")}`);
  }
  displayNo = map;
}
const no = (id) => (id ? displayNo.get(id) || id : id);

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
  const tickets = new Map((state.tickets || []).map((t) => [t.order_id, t]));
  const active = state.active_order_id ? orders.get(state.active_order_id) : null;
  buildDisplayNumbers(state.orders, state.tickets || []);
  const mode = state.execution_backend?.mode || "-";

  $("s-mission").textContent = MISSION_LABEL[state.mission_state] || state.mission_state;
  $("s-location").textContent = place(state.current_location);
  $("s-battery").textContent = `${state.battery_percent.toFixed(1)}%`;
  const eb = state.execution_backend || {};
  const manip = eb.manipulation_service_state;
  $("s-mode").textContent = manip ? `${mode} · 조작 ${MANIP_LABEL[manip] || manip}` : mode;
  $("s-mode").title = eb.manipulation || mode;

  // 정렬 완료 신호 대기 (rollout-service backend). 주행 연동 전에는 운영자가 확인 후 누른다.
  const waiting = eb.waiting_alignment;
  $("align-banner").hidden = !waiting;
  if (waiting) {
    const next = waiting.station === "kitchen" ? "물따르기" : "컵 배치";
    $("align-text").textContent = `${waiting.station === "kitchen" ? "주방" : "테이블"} 정렬 완료 신호를 기다리는 중입니다. 차가 완전히 멈추고 정렬됐는지 확인한 뒤 누르면 ${next}를 시작합니다.`;
    $("align-confirm").textContent = `정렬 완료 → ${next} 시작`;
    $("align-confirm").dataset.station = waiting.station;
  }

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
      el("b", `${place(active.table_id)} · ${drinkText(active, tickets)}`),
      el("span", `주문 ${no(active.order_id)}`, "muted"),
      el("span", `단계: ${PHASE_LABEL[state.phase] || state.phase}${state.phase_attempt > 1 ? ` (${state.phase_attempt}번째 시도)` : ""}`),
    );
    if (eb.running_policy) head.append(el("b", `팔 정책 실행 중: ${eb.running_policy === "pour" ? "물따르기" : "컵 배치"}`));
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
    return row([String(i + 1), no(id), place(o.table_id), drinkText(o, tickets), time(o.created_at), cancel]);
  });
  $("queue").replaceChildren(...(queueRows.length ? queueRows : [emptyRow("대기 중인 주문이 없습니다.", 6)]));

  // 최근 주문(완료·실패·취소)
  const done = state.orders.filter((o) => ["SUCCEEDED", "FAILED", "CANCELED"].includes(o.state)).reverse().slice(0, 10);
  $("history").replaceChildren(...(done.length
    ? done.map((o) => row([no(o.order_id), place(o.table_id), drinkText(o, tickets), badge(o.state), time(o.completed_at), failureText(o.failure)]))
    : [emptyRow("아직 끝난 주문이 없습니다.", 6)]));

  // 직원이 가져다줄 음식(로봇은 음료만 맡는다)
  const staff = (state.tickets || []).filter((t) => t.staff_state === "PREPARING");
  $("staff-count").textContent = staff.length ? `${staff.length}건` : "";
  $("staff").replaceChildren(...(staff.length ? staff.map((t) => {
    const served = el("button", "서빙 완료", "btn small primary");
    served.type = "button";
    served.addEventListener("click", () => act(() => api(`/api/tickets/${encodeURIComponent(t.order_id)}/served`, {})));
    return row([no(t.order_id), place(t.table_id), itemsText(t), time(t.created_at), served]);
  }) : [emptyRow("직원이 가져다줄 음식이 없습니다.", 5)]));

  drawMap(active);

  // 이벤트
  $("events").replaceChildren(...state.events.slice(-30).reverse().map((e) => {
    const phase = e.completed_phase || e.failed_phase;
    const extra = [no(e.order_id), phase && PHASE_LABEL[phase], e.failure].filter(Boolean).join(" · ");
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
  if (confirm(`주문 ${no(id)}을(를) 취소할까요?`)) act(() => api(`/api/orders/${encodeURIComponent(id)}/cancel`, {}));
}

$("align-confirm").addEventListener("click", () => {
  const station = $("align-confirm").dataset.station;
  if (confirm("차가 멈추고 정렬된 것을 확인했나요? 누르면 팔이 움직입니다.")) {
    act(() => api("/api/align/confirm", { station }));
  }
});
$("pause").addEventListener("click", () => act(() => api("/api/control", { paused: !state?.auto_paused })));
$("abort").addEventListener("click", () => {
  const id = state?.active_order_id;
  if (id && confirm(`진행 중인 주문 ${no(id)}을(를) 중단할까요?\n로봇은 다음 주문 또는 충전소 복귀로 넘어갑니다.`)) {
    act(() => api(`/api/orders/${encodeURIComponent(id)}/cancel`, {}));
  }
});
$("f-submit").addEventListener("click", () => act(async () => {
  // 주문 번호(YYMMDDNNN)는 서버가 붙인다.
  const order = { table_id: $("f-table").value, drink: $("f-drink").value };
  const result = await api("/api/orders", order);
  $("f-msg").textContent = `${result.order.order_id} · ${place(order.table_id)} ${DRINKS[order.drink]} 주문을 등록했습니다.`;
}));

// ---------- 지도: 교실 지도(classroom) 위에 충전소·급수대·테이블과 로봇 위치 ----------
// 실시간 위치는 서버의 --robot-pose-url(파이 주행 대시보드)에서 온다. 없으면 현재 단계로 추정해 표시한다.
const MAP = { meta: null, img: null };
fetch("/map/classroom.json").then((r) => r.json()).then((meta) => {
  const img = new Image();
  img.onload = () => { MAP.meta = meta; MAP.img = img; if (state) render(); };
  img.src = "/map/classroom.png";
}).catch(() => { $("map-note").textContent = "지도를 불러오지 못했습니다."; });

const STATION_COLOR = { dock: "#2b8a5a", kitchen: "#e8590c" };
function stationOf(id) {
  if (!MAP.meta || !id) return null;
  const stations = MAP.meta.stations;
  // 실제 지도에는 아직 1번 테이블만 등록돼 있다. 다른 테이블은 1번 자리로 대신 표시한다.
  return stations.find((s) => s.id === id) || (/^table_\d+$/.test(id) ? stations.find((s) => s.id === "table_1") : null);
}
// 추정 위치: 이동 단계가 시작된 뒤 지난 시간으로 출발지 -> 목적지 사이를 채운다(실위치 연동 전 임시).
const NAV_TARGET = { NAVIGATE_KITCHEN: "kitchen", NAVIGATE_DOCK: "dock" };
const ASSUMED_SPEED_MPS = 0.3; // 실차 평균 속도 가정(측정값 아님)
const motion = { key: null, startedAt: 0, from: null };
let lastActive = null;
function estimatedPose(active) {
  const here = stationOf(state.current_location) || stationOf("dock");
  const target = state.phase === "NAVIGATE_TABLE" ? active?.table_id : NAV_TARGET[state.phase];
  const there = target ? stationOf(target) : null;
  const key = `${state.active_order_id}|${state.phase}|${state.phase_attempt}`;
  if (motion.key !== key) { motion.key = key; motion.startedAt = performance.now(); motion.from = here; }
  if (!there || there === motion.from) return { x_m: here.x_m, y_m: here.y_m, yaw_deg: here.yaw_rad * 180 / Math.PI };
  const from = motion.from;
  const dist = Math.hypot(there.x_m - from.x_m, there.y_m - from.y_m);
  const mode = state.execution_backend?.mode;
  // 데모·dry-run은 단계가 정해진 시간에 끝나므로 그 시간에 맞추고, 실차는 거리/가정 속도로 잡는다.
  const expectedMs = mode === "browser_demo" ? 2500 : mode === "simulation_dry_run" ? 750 : (dist / ASSUMED_SPEED_MPS) * 1000;
  const t = Math.min(0.97, (performance.now() - motion.startedAt) / expectedMs);
  return {
    x_m: from.x_m + (there.x_m - from.x_m) * t, y_m: from.y_m + (there.y_m - from.y_m) * t,
    yaw_deg: Math.atan2(there.y_m - from.y_m, there.x_m - from.x_m) * 180 / Math.PI, from, to: there,
  };
}
function drawMap(active) {
  lastActive = active;
  const canvas = $("map");
  if (!MAP.meta || !state) return;
  const { meta, img } = MAP;
  const scale = canvas.width / meta.width;
  if (canvas.height !== Math.round(meta.height * scale)) canvas.height = Math.round(meta.height * scale);
  const ctx = canvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
  const px = (x, y) => [((x - meta.origin.x_m) / meta.resolution_m) * scale, (meta.height - (y - meta.origin.y_m) / meta.resolution_m) * scale];
  ctx.font = "bold 12px sans-serif";
  for (const s of meta.stations) {
    const [x, y] = px(s.x_m, s.y_m);
    ctx.fillStyle = STATION_COLOR[s.id] || "#7a4fd1";
    ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#1f3553";
    const right = x > canvas.width * 0.6; // 오른쪽 끝 역은 이름을 왼쪽에 써서 잘리지 않게 한다
    ctx.textAlign = right ? "right" : "left";
    ctx.fillText(s.label, right ? x - 9 : x + 9, y - 8);
  }
  const rp = state.robot_pose;
  let pose = null;
  let label = "";
  if (rp && rp.live) { pose = rp; label = "실시간"; }
  else if (rp && rp.last) { pose = rp.last; label = "마지막 수신 위치"; }
  else { pose = estimatedPose(active); label = "추정"; }
  if (pose.from) {
    const [ax, ay] = px(pose.from.x_m, pose.from.y_m);
    const [bx, by] = px(pose.to.x_m, pose.to.y_m);
    ctx.save(); ctx.setLineDash([6, 5]); ctx.strokeStyle = "rgba(22,119,200,.7)"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.restore();
  }
  // 로봇: 퍼지는 원 + 파란 점 + 진행 방향 짧은 선
  const [rx, ry] = px(pose.x_m, pose.y_m);
  const live = label === "실시간";
  const color = live ? "22,119,200" : label === "추정" ? "22,119,200" : "120,134,150";
  const phase = (performance.now() % 1600) / 1600;
  ctx.fillStyle = `rgba(${color},${0.35 * (1 - phase)})`;
  ctx.beginPath(); ctx.arc(rx, ry, 8 + 14 * phase, 0, Math.PI * 2); ctx.fill();
  const yaw = -(pose.yaw_deg - (meta.origin.yaw_deg || 0)) * Math.PI / 180;
  ctx.strokeStyle = `rgb(${color})`; ctx.lineWidth = 3; ctx.lineCap = "round";
  ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(rx + Math.cos(yaw) * 15, ry + Math.sin(yaw) * 15); ctx.stroke();
  ctx.fillStyle = `rgb(${color})`; ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.arc(rx, ry, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  const src = $("map-src");
  src.textContent = live ? "· 실시간" : `· ${label}`;
  src.className = live ? "pose-live" : "pose-est";
  $("map-note").textContent = live
    ? `x ${pose.x_m.toFixed(2)} m, y ${pose.y_m.toFixed(2)} m, ${Math.round(pose.yaw_deg)}° (주행 대시보드${rp.age_s != null ? `, ${rp.age_s.toFixed(1)}초 전` : ""})`
    : rp ? `실시간 위치를 받지 못하고 있습니다: ${rp.error || "-"}`
      : "실시간 위치 연결 없음 — 진행 단계와 경과 시간으로 추정한 위치입니다. 서버를 --robot-pose-url 로 띄우면 실제 위치가 표시됩니다.";
}
// 1초마다 오는 상태 사이에도 점이 부드럽게 움직이도록 지도만 다시 그린다.
(function animateMap() {
  if (MAP.meta && state && !document.hidden) drawMap(lastActive);
  requestAnimationFrame(animateMap);
})();
$("demo-note").hidden = window.orderApiMode !== "demo";
$("t-reset").addEventListener("click", () => {
  if (confirm("데모 주문과 기록을 모두 지울까요?")) window.resetDemo().then(() => location.reload());
});
$("t-step").addEventListener("click", () => act(() => api("/api/step", { success: true })));
$("t-fail").addEventListener("click", () => act(() => api("/api/step", { success: false, failure: "OPERATOR_INJECTED" })));

refresh();
setInterval(refresh, 1000);
