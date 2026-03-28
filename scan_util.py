import asyncio
from pathlib import Path
from flask import Flask, jsonify, render_template, request
from bleak import BleakScanner

app = Flask(__name__)

OUTPUT_CONFIG = Path("sensor_addresses.txt")
SCAN_SECONDS = 3

WIT_SENSORS = [
    ("WIT1", "WIT sensor 1 (S1 = thigh)"),
    ("WIT2", "WIT sensor 2 (S2 = calf)"),
    ("WIT3", "WIT sensor 3 (S3 = arm)"),
    ("WIT4", "WIT sensor 4 (S4 = back)"),
]

OTHER_SENSORS = [
    ("HR", "Heart rate sensor"),
    ("CAD", "Cadence sensor"),
    ("SPD", "Speed sensor"),
    ("PWR", "Power meter"),
    ("DI2", "DI2 / derailleur"),
]

ALL_SENSOR_KEYS = [k for k, _ in WIT_SENSORS + OTHER_SENSORS]

saved_addresses = {k: "" for k in ALL_SENSOR_KEYS}


def normalize_addr(addr: str) -> str:
    return addr.strip().lower()


def normalize_name(name: str | None) -> str:
    return (name or "").strip()


async def scan_ble(timeout: float = SCAN_SECONDS):
    devices = await BleakScanner.discover(timeout=timeout)

    dedup: dict[str, dict] = {}
    for d in devices:
        addr = normalize_addr(d.address or "")
        raw_name = normalize_name(d.name)

        if not addr:
            continue

        # Ignore unnamed devices
        if not raw_name:
            continue

        dedup[addr] = {
            "address": addr,
            "name": raw_name,
        }

    out = list(dedup.values())
    out.sort(key=lambda x: (x["name"].lower(), x["address"]))
    return out


def save_config_file():
    lines = ["# Unified sensor address config", ""]
    for key in ALL_SENSOR_KEYS:
        lines.append(f"{key}={saved_addresses.get(key, '')}")
    OUTPUT_CONFIG.write_text("\n".join(lines) + "\n", encoding="utf-8")


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/intro", methods=["GET"])
def api_intro():
    intro = [
        "This utility connects two types of sensors:",
        "1. WIT giro sensors: one-by-one scan with WIT name filtering",
        "2. Other cycling sensors: full scan list, then choose by number",
        "",
        "Giro mapping used in firmware:",
        "  S1 = thigh",
        "  S2 = calf",
        "  S3 = arm",
        "  S4 = back",
        "",
        "Wake-up notes:",
        "- WIT: power on and move sensor",
        "- HR: wear strap and moisten contacts",
        "- CAD: rotate crank",
        "- SPD: spin wheel",
        "- PWR: spin crank / wake meter",
        "- DI2: wake drivetrain",
        "",
    ]
    return jsonify({"lines": intro})


@app.route("/api/sensors", methods=["GET"])
def api_sensors():
    sensors = [
        {"key": key, "label": desc}
        for key, desc in (WIT_SENSORS + OTHER_SENSORS)
    ]
    return jsonify(
        {
            "wit_sensors": [{"key": k, "label": d} for k, d in WIT_SENSORS],
            "other_sensors": [{"key": k, "label": d} for k, d in OTHER_SENSORS],
            "all_sensors": sensors,
            "saved_addresses": saved_addresses,
        }
    )


@app.route("/api/scan", methods=["GET"])
def api_scan():
    try:
        devices = asyncio.run(scan_ble())
        return jsonify({"ok": True, "devices": devices})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "devices": []}), 500


@app.route("/api/save-selection", methods=["POST"])
def api_save_selection():
    data = request.get_json(silent=True) or {}
    sensor_key = str(data.get("sensor_key", "")).strip()
    address = normalize_addr(str(data.get("address", "")))

    if sensor_key not in ALL_SENSOR_KEYS:
        return jsonify({"ok": False, "error": "Invalid sensor key"}), 400

    if not address:
        return jsonify({"ok": False, "error": "Missing address"}), 400

    saved_addresses[sensor_key] = address
    return jsonify({"ok": True, "saved_addresses": saved_addresses})


@app.route("/api/clear-selection", methods=["POST"])
def api_clear_selection():
    data = request.get_json(silent=True) or {}
    sensor_key = str(data.get("sensor_key", "")).strip()

    if sensor_key not in ALL_SENSOR_KEYS:
        return jsonify({"ok": False, "error": "Invalid sensor key"}), 400

    saved_addresses[sensor_key] = ""
    return jsonify({"ok": True, "saved_addresses": saved_addresses})


@app.route("/api/write-config", methods=["POST"])
def api_write_config():
    try:
        save_config_file()
        return jsonify(
            {
                "ok": True,
                "path": str(OUTPUT_CONFIG.resolve()),
                "saved_addresses": saved_addresses,
            }
        )
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5000, debug=True)