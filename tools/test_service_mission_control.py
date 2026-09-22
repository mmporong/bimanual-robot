import json
from datetime import datetime, timezone

import pytest

from service_mission_control import (
    DryRunRuntime,
    MissionController,
    SERVICE_PHASES,
    load_control_config,
    run_batch,
)


NOW = datetime(2026, 9, 22, 3, 0, tzinfo=timezone.utc)


def controller(battery=100.0):
    return MissionController(battery_percent=battery, clock=lambda: NOW)


def order(order_id="ORD-001", table="table_1", drink="COLD_WATER", priority=0):
    return {
        "order_id": order_id,
        "table_id": table,
        "drink": drink,
        "created_at": "2026-09-22T12:00:00+09:00",
        "priority": priority,
    }


def finish_active(current):
    active = current.active_order_id
    for _ in range(len(SERVICE_PHASES)):
        assert current.active_order_id == active
        current.advance()


def test_duplicate_order_is_idempotent_and_conflict_is_rejected():
    current = controller()
    first, created = current.submit(order())
    repeated, repeated_created = current.submit(order())
    assert created and not repeated_created
    assert first is repeated
    assert len(current.orders) == 1
    with pytest.raises(ValueError, match="conflicts on table_id"):
        current.submit(order(table="table_2"))


@pytest.mark.parametrize("field,value", [
    ("order_id", "bad order id"),
    ("table_id", "table_9"),
    ("drink", "COFFEE"),
    ("priority", 10),
    ("created_at", "2026-09-22T12:00:00"),
])
def test_invalid_order_is_rejected(field, value):
    payload = order()
    payload[field] = value
    with pytest.raises(ValueError):
        controller().submit(payload)


def test_multiple_orders_continue_without_intermediate_dock_return():
    current = controller()
    current.submit(order("ORD-COLD", "table_3", "COLD_WATER"))
    current.submit(order("ORD-HOT", "table_2", "HOT_WATER"))
    finish_active(current)
    assert current.orders["ORD-COLD"].state == "SUCCEEDED"
    assert current.active_order_id == "ORD-HOT"
    assert current.phase == "NAVIGATE_KITCHEN"
    assert current.current_location == "table_3"
    kinds = [event["kind"] for event in current.events]
    first_success = kinds.index("MISSION_SUCCEEDED")
    assert "RETURN_TO_DOCK_STARTED" not in kinds[: first_success + 2]
    command = current.current_command()
    assert command["start"] == "table_3" and command["destination"] == "kitchen"
    current.advance()
    while current.phase != "GRASP_BOTTLE":
        current.advance()
    assert current.current_command()["bottle_id"] == "hot_bottle"


def test_priority_orders_are_selected_after_current_mission():
    current = controller()
    current.submit(order("ACTIVE", "table_1"))
    current.submit(order("LOW", "table_2", priority=0))
    current.submit(order("HIGH", "table_4", priority=7))
    assert current.snapshot()["queue"] == ["HIGH", "LOW"]
    finish_active(current)
    assert current.active_order_id == "HIGH"


def test_low_battery_charges_before_dispatch():
    current = controller(battery=16.0)
    queued, _ = current.submit(order())
    assert queued.state == "QUEUED"
    assert current.phase == "CHARGING"
    assert current.current_command()["kind"] == "charge"
    current.advance()
    assert current.battery_percent == current.config["battery"]["charge_target_percent"]
    assert current.active_order_id == queued.order_id
    assert current.phase == "NAVIGATE_KITCHEN"


def test_retryable_phase_retries_once_then_fails_closed():
    current = controller()
    current.submit(order())
    current.advance()
    assert current.phase == "ALIGN_KITCHEN"
    current.advance(success=False, failure="pose_tolerance")
    assert current.phase == "ALIGN_KITCHEN" and current.phase_attempt == 2
    current.advance(success=False, failure="pose_tolerance")
    assert current.orders["ORD-001"].state == "FAILED"
    assert current.orders["ORD-001"].failure == "ALIGN_KITCHEN:pose_tolerance"
    assert current.phase == "NAVIGATE_DOCK"


def test_nonretryable_pour_failure_stops_order_immediately():
    current = controller()
    current.submit(order())
    while current.phase != "POUR":
        current.advance()
    current.advance(success=False, failure="volume_below_target")
    assert current.orders["ORD-001"].state == "FAILED"
    assert current.phase == "NAVIGATE_DOCK"
    assert not any(event["kind"] == "PHASE_RETRY" for event in current.events)


def test_canceling_queued_order_does_not_interrupt_active_order():
    current = controller()
    current.submit(order("ACTIVE"))
    current.submit(order("QUEUED", "table_2"))
    current.cancel("QUEUED")
    assert current.orders["QUEUED"].state == "CANCELED"
    assert current.active_order_id == "ACTIVE"
    assert current.snapshot()["queue"] == []


def test_dry_run_completes_orders_returns_and_charges(tmp_path):
    batch = tmp_path / "orders.json"
    batch.write_text(json.dumps({
        "schema": "service_order_batch_v1",
        "orders": [order("ONE", "table_1"), order("TWO", "table_4", "HOT_WATER")],
    }))
    output = tmp_path / "run"
    snapshot = run_batch(batch, output, 100.0)
    assert snapshot["phase"] == "IDLE_AT_DOCK"
    assert snapshot["current_location"] == "dock"
    assert snapshot["battery_percent"] == 95.0
    assert {item["state"] for item in snapshot["orders"]} == {"SUCCEEDED"}
    assert (output / "snapshot.json").is_file()
    assert (output / "commands.json").is_file()
    assert (output / "events.jsonl").is_file()


def test_navigation_command_uses_registered_destinations_and_collision_route():
    current = controller()
    current.submit(order(table="table_4"))
    to_kitchen = current.current_command()
    assert to_kitchen["kind"] == "navigate"
    assert to_kitchen["start"] == "dock" and to_kitchen["destination"] == "kitchen"
    assert len(to_kitchen["path_xy_m"]) > 1 and to_kitchen["route_length_m"] > 0
    assert not to_kitchen["hardware_accessed"]
    for _ in range(SERVICE_PHASES.index("NAVIGATE_TABLE")):
        current.advance()
    to_table = current.current_command()
    assert to_table["destination"] == "table_4"


def test_control_config_rejects_missing_phase_energy():
    config = load_control_config()
    del config["phase_energy_percent"]["POUR"]
    with pytest.raises(ValueError, match="every manipulation phase"):
        load_control_config_from_dict(config)


def load_control_config_from_dict(config, tmp_path=None):
    # Keep this helper local so production parsing is exercised through a file.
    from tempfile import TemporaryDirectory
    from pathlib import Path
    with TemporaryDirectory() as directory:
        path = Path(directory) / "config.json"
        path.write_text(json.dumps(config))
        return load_control_config(path)


def test_runtime_max_steps_is_fail_closed():
    current = controller()
    current.submit(order())
    runtime = DryRunRuntime(current)
    with pytest.raises(RuntimeError, match="max_steps"):
        runtime.run_until_idle(max_steps=1)
