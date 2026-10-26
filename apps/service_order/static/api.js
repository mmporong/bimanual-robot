"use strict";

// 주문·관제 화면 공통 API 연결.
// - 기본: 화면을 내려준 서버(같은 주소)의 /api 를 쓴다.
// - ?api=https://서버주소 : 다른 주소의 주문 서버를 쓴다(서버는 --cors-origin 필요). 브라우저에 기억된다.
// - ?api=demo 또는 config.js 의 ORDER_API_BASE="demo" : 서버 없이 브라우저 안에서 흉내 내는 데모.
//   데모 상태는 이 브라우저의 IndexedDB 에만 있어 같은 브라우저의 탭끼리만 공유된다.
(() => {
  const API_KEY = "bimanual-robot.api-base";
  const query = new URLSearchParams(location.search).get("api");
  if (query !== null) {
    try { query ? localStorage.setItem(API_KEY, query) : localStorage.removeItem(API_KEY); } catch { /* 기억 못 해도 이번 접속은 동작 */ }
  }
  let stored = null;
  try { stored = localStorage.getItem(API_KEY); } catch { /* 무시 */ }
  const base = (query || stored || window.ORDER_API_BASE || "").replace(/\/$/, "");
  const demo = base === "demo";

  class ApiError extends Error {
    constructor(message, status) { super(message); this.status = status; }
  }

  async function serverApi(path, options = {}) {
    const response = await fetch(base + path, { cache: "no-store", ...options });
    const body = await response.json().catch(() => ({}));
    if (!response.ok) throw new ApiError(body.detail || body.error || response.statusText, response.status);
    return body;
  }

  // ---------- 브라우저 데모 백엔드 ----------
  const DEMO_DB = "bimanual-robot-demo";
  const STEP_MS = 2500;
  const PHASES = ["NAVIGATE_KITCHEN", "ALIGN_KITCHEN", "GRASP_CUP", "GRASP_BOTTLE", "POUR", "RETURN_BOTTLE",
    "PLACE_DECK", "NAVIGATE_TABLE", "ALIGN_TABLE", "REGRASP_CUP", "SERVE"];
  const TABLES = ["table_1", "table_2", "table_3", "table_4"];
  const DRINKS = ["COLD_WATER", "HOT_WATER"];
  const now = () => new Date().toISOString();

  function fresh() {
    return { phase: "IDLE_AT_DOCK", phase_attempt: 0, active_order_id: null, current_location: "dock",
      battery_percent: 100, orders: [], events: [], auto_paused: false, sequence: 0, event_sequence: 0, last_tick: Date.now() };
  }
  // 손님·관제 탭이 같은 상태를 동시에 고치므로, 읽기-수정-쓰기를 IndexedDB 트랜잭션 하나로 묶어
  // 탭 사이 쓰기가 서로 덮어쓰지 않게 한다. IndexedDB 를 못 쓰면 이 탭 메모리에만 둔다.
  let dbPromise = null;
  let memoryState = null;
  function openDb() {
    if (!dbPromise) {
      dbPromise = new Promise((resolve) => {
        try {
          const request = indexedDB.open(DEMO_DB, 1);
          request.onupgradeneeded = () => request.result.createObjectStore("state");
          request.onsuccess = () => resolve(request.result);
          request.onerror = () => resolve(null);
        } catch { resolve(null); }
      });
    }
    return dbPromise;
  }
  async function withState(mutate) {
    const db = await openDb();
    if (!db) {
      memoryState = memoryState || fresh();
      return mutate(memoryState);
    }
    return new Promise((resolve, reject) => {
      const tx = db.transaction("state", "readwrite");
      const objects = tx.objectStore("state");
      let result;
      let failure = null;
      objects.get("current").onsuccess = (event) => {
        const s = event.target.result || fresh();
        try {
          result = mutate(s);
          objects.put(s, "current");
        } catch (error) {
          failure = error;
          tx.abort();
        }
      };
      tx.oncomplete = () => resolve(result);
      tx.onabort = () => reject(failure || tx.error);
    });
  }
  function record(s, kind, details = {}) {
    s.event_sequence += 1;
    s.events.push({ sequence_number: s.event_sequence, timestamp: now(), kind, phase: s.phase,
      active_order_id: s.active_order_id, battery_percent: s.battery_percent, ...details });
    s.events = s.events.slice(-200);
  }
  const queued = (s) => s.orders.filter((o) => o.state === "QUEUED")
    .sort((a, b) => b.priority - a.priority || a.sequence - b.sequence);
  const active = (s) => s.orders.find((o) => o.order_id === s.active_order_id) || null;

  function startNext(s) {
    const next = queued(s)[0];
    if (next) {
      next.state = "RUNNING";
      next.mission_id = `MISSION-${next.order_id}`;
      s.active_order_id = next.order_id;
      s.phase = "NAVIGATE_KITCHEN";
      s.phase_attempt = 1;
      record(s, "MISSION_STARTED", { order_id: next.order_id });
    } else {
      s.active_order_id = null;
      s.phase = s.current_location === "dock" ? "IDLE_AT_DOCK" : "NAVIGATE_DOCK";
      s.phase_attempt = 0;
      if (s.phase === "NAVIGATE_DOCK") record(s, "RETURN_TO_DOCK_STARTED");
    }
  }
  function finish(s, order, stateName, failure) {
    order.state = stateName;
    order.completed_at = now();
    if (failure) order.failure = failure;
    startNext(s);
  }
  function advance(s, success = true, failure = null) {
    const order = active(s);
    if (s.phase === "NAVIGATE_DOCK") {
      s.current_location = "dock";
      record(s, "DOCK_ARRIVED");
      startNext(s);
      return;
    }
    if (!order) { startNext(s); return; }
    const phase = s.phase;
    if (!success) {
      record(s, "MISSION_FAILED", { order_id: order.order_id, failed_phase: phase, failure });
      finish(s, order, "FAILED", `${phase}:${failure}`);
      return;
    }
    if (phase === "NAVIGATE_KITCHEN") s.current_location = "kitchen";
    if (phase === "NAVIGATE_TABLE") s.current_location = order.table_id;
    s.battery_percent = Math.max(0, +(s.battery_percent - 0.4).toFixed(1));
    record(s, "PHASE_SUCCEEDED", { completed_phase: phase, order_id: order.order_id });
    const index = PHASES.indexOf(phase);
    if (index + 1 < PHASES.length) {
      s.phase = PHASES[index + 1];
      s.phase_attempt = 1;
    } else {
      record(s, "MISSION_SUCCEEDED", { order_id: order.order_id });
      finish(s, order, "SUCCEEDED");
    }
  }
  function tick(s) {
    const steps = Math.min(20, Math.floor((Date.now() - s.last_tick) / STEP_MS));
    if (steps > 0) s.last_tick += steps * STEP_MS;
    if (s.auto_paused) { s.last_tick = Date.now(); return; }
    for (let i = 0; i < steps && s.phase !== "IDLE_AT_DOCK"; i += 1) advance(s);
  }
  function snapshot(s) {
    return {
      timestamp: now(),
      mission_state: s.active_order_id ? "RUNNING" : s.phase === "IDLE_AT_DOCK" ? "IDLE" : "RUNNING",
      phase: s.phase, phase_attempt: s.phase_attempt, active_order_id: s.active_order_id,
      current_location: s.current_location, battery_percent: s.battery_percent,
      queue: queued(s).map((o) => o.order_id), orders: s.orders, events: s.events,
      auto_paused: s.auto_paused,
      restaurant: { tables: TABLES.map((id) => ({ id })) },
      execution_backend: { mode: "browser_demo", hardware_accessed: false },
    };
  }

  async function demoApi(path, options = {}) {
    const payload = options.body ? JSON.parse(options.body) : {};
    return withState((s) => handle(s, path, payload));
  }

  function handle(s, path, payload) {
    tick(s);
    let result;
    const cancel = path.match(/^\/api\/orders\/([^/]+)\/cancel$/);
    if (path === "/api/state") {
      result = snapshot(s);
    } else if (path === "/api/orders") {
      if (!TABLES.includes(payload.table_id) || !DRINKS.includes(payload.drink)) throw new ApiError("invalid order", 400);
      let order = s.orders.find((o) => o.order_id === payload.order_id);
      if (!order) {
        s.sequence += 1;
        order = { order_id: payload.order_id, table_id: payload.table_id, drink: payload.drink, created_at: now(),
          priority: 0, sequence: s.sequence, state: "QUEUED", mission_id: null, failure: null, completed_at: null };
        s.orders.push(order);
        record(s, "ORDER_QUEUED", { order_id: order.order_id, table_id: order.table_id, drink: order.drink });
        if (!s.active_order_id) { startNext(s); s.last_tick = Date.now(); }
      }
      result = { order };
    } else if (cancel) {
      const order = s.orders.find((o) => o.order_id === decodeURIComponent(cancel[1]));
      if (!order) throw new ApiError("unknown order", 404);
      if (["QUEUED", "RUNNING"].includes(order.state)) {
        record(s, "ORDER_CANCELED", { order_id: order.order_id });
        if (order.state === "RUNNING") finish(s, order, "CANCELED");
        else { order.state = "CANCELED"; order.completed_at = now(); }
      }
      result = { order };
    } else if (path === "/api/control") {
      s.auto_paused = payload.paused === true;
      result = { auto_paused: s.auto_paused };
    } else if (path === "/api/step") {
      advance(s, payload.success !== false, payload.failure || null);
      result = { state: snapshot(s) };
    } else {
      throw new ApiError("not found", 404);
    }
    return result;
  }

  window.orderApi = demo ? demoApi : serverApi;
  window.orderApiMode = demo ? "demo" : base ? "remote" : "local";
  window.resetDemo = () => withState((s) => { Object.assign(s, fresh()); });
})();
