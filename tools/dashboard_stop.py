"""명시한 STS 팔 포트의 토크 OFF만 수행하는 대시보드 정지 제어."""

import json
import pathlib
import subprocess
import threading
import time

from servo.sts_bus import A_TORQUE, Bus


def open_stop_bus(port):
    # 다른 제어기와 패킷을 섞지 않는다. 점유 시 정지 실패를 표시한다.
    owners = subprocess.run(["fuser", port], capture_output=True, timeout=2)
    if owners.returncode == 0:
        raise RuntimeError("포트가 다른 제어기에 사용 중입니다. 제어기를 중단하고 전원을 차단하세요.")
    if owners.returncode != 1:
        raise RuntimeError("포트 점유 확인 실패")
    bus = Bus(port, timeout=0.05)
    try:
        bus.ser.exclusive = True
    except Exception:
        bus.close()
        raise
    return bus


class StopController:
    def __init__(self, ports=(), bus_factory=open_stop_bus, log_path=None):
        self.ports = tuple(ports)
        if len(self.ports) not in (0, 2) or len(set(self.ports)) != len(self.ports):
            raise ValueError("양팔 정지는 서로 다른 팔 포트 2개를 지정해야 합니다.")
        self.bus_factory = bus_factory
        self.log_path = pathlib.Path(log_path) if log_path else None
        self.lock = threading.Lock()
        self.run_lock = threading.Lock()
        self.state = {"enabled": bool(ports), "requested": False, "running": False,
                      "verified_off": False, "results": [], "log_error": None}

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def stop(self):
        # 반복 클릭도 다시 OFF를 보낸다. 이전 성공은 현재 토크 상태의 보증이 아니다.
        with self.run_lock:
            with self.lock:
                self.state.update(requested=True, running=True, verified_off=False)
            results = []
            buses = []
            try:
                for port in self.ports:
                    result = {"port": port, "servos": [], "error": None}
                    results.append(result)
                    try:
                        bus = self.bus_factory(port)
                        buses.append((bus, result))
                    except (Exception, SystemExit) as exc:
                        result["error"] = str(exc)
                # 전체 OFF 전송 뒤 검증한다. 한 ID 실패로 다른 ID/팔을 건너뛰지 않는다.
                for bus, result in buses:
                    for sid in range(1, 7):
                        item = {"id": sid, "ack": False, "torque": None, "error": None}
                        result["servos"].append(item)
                        try:
                            item["ack"] = bool(bus.write(sid, A_TORQUE, 0))
                        except (Exception, SystemExit) as exc:
                            item["error"] = str(exc)
                for bus, result in buses:
                    for item in result["servos"]:
                        try:
                            item["torque"] = bus.read(item["id"], A_TORQUE)
                        except (Exception, SystemExit) as exc:
                            item["error"] = str(exc)
            finally:
                for bus, result in buses:
                    try:
                        bus.close()
                    except (Exception, SystemExit) as exc:
                        result["error"] = str(exc)
                verified = len(self.ports) == 2 and len(results) == 2 and all(
                    not r["error"] and len(r["servos"]) == 6 and all(
                        s["torque"] == 0 and not s["error"] for s in r["servos"]
                    ) for r in results
                )
                with self.lock:
                    self.state.update(running=False, verified_off=verified,
                                      results=results, stopped_at=time.time(), log_error=None)
                if self.log_path:
                    try:
                        self.log_path.parent.mkdir(parents=True, exist_ok=True)
                        with self.log_path.open("a", encoding="utf-8") as handle:
                            handle.write(json.dumps(self.snapshot(), ensure_ascii=False) + "\n")
                    except OSError as exc:
                        with self.lock:
                            self.state["log_error"] = str(exc)
            return self.snapshot()
