import asyncio
from pathlib import Path
from bleak import BleakScanner

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


def normalize_addr(addr: str) -> str:
    return addr.strip().lower()


def normalize_name(name: str | None) -> str:
    return (name or "").strip().lower()


async def print_intro():
    # Print these two instantly together
    print("AEROVIS SENSOR PAIRING UTILITY")
    print("===================================")

    await asyncio.sleep(1)

    # Line-by-line section
    lines = [
        "This utility connects two types of sensors:",
        "1. WIT giro sensors: one-by-one scan with WIT name filtering",
        "2. Other cycling sensors: full scan list, then choose by number",
        ""
    ]

    for line in lines:
        print(line)
        await asyncio.sleep(1)

    # 🚀 Print mapping ALL AT ONCE
    print("Giro mapping used in firmware:")
    print("  S1 = thigh")
    print("  S2 = calf")
    print("  S3 = arm")
    print("  S4 = back")

    await asyncio.sleep(1)

    # 🚀 Print wake-up guide ALL AT ONCE
    print("")
    print("Wake-up notes:")
    print("- WIT: power on and move sensor")
    print("- HR: wear strap and moisten contacts")
    print("- CAD: rotate crank")
    print("- SPD: spin wheel")
    print("- PWR: spin crank / wake meter")
    print("- DI2: wake drivetrain")
    print("")

    await asyncio.sleep(1)

    input("Press Enter to continue...")

async def scan_ble(timeout: float = SCAN_SECONDS):
    print(f"Scanning for {timeout} seconds...")
    devices = await BleakScanner.discover(timeout=timeout)

    dedup = {}
    for d in devices:
        addr = normalize_addr(d.address or "")
        raw_name = d.name or ""
        name = normalize_name(raw_name)

        if not addr:
            continue

        # Ignore unnamed devices
        if not name:
            continue

        dedup[addr] = {
            "address": addr,
            "name": name,
            "raw_name": raw_name,
        }

    out = list(dedup.values())
    out.sort(key=lambda x: (x["name"], x["address"]))
    return out


def print_devices(devices):
    if not devices:
        print("No named BLE devices found.")
        return

    for i, d in enumerate(devices):
        print(f"[{i:02d}] {d['raw_name']} | {d['address']}")


async def scan_wit_one_by_one(sensor_key: str, sensor_desc: str) -> str:
    while True:
        print(f"\n=== Scan for {sensor_key}: {sensor_desc} ===")
        print("Wake only this WIT sensor if possible.")

        devices = await scan_ble()

        print("\nDetected BLE devices (named only):")
        print_devices(devices)

        if not devices:
            raw = input("\nNo devices found. Press Enter to rescan or type s to skip: ").strip().lower()
            if raw == "s":
                return ""
            continue

        raw = input(
            f"\nEnter index to save for {sensor_key}, r to rescan, or s to skip: "
        ).strip().lower()

        if raw == "s":
            return ""
        if raw == "r":
            continue
        if raw.isdigit():
            idx = int(raw)
            if 0 <= idx < len(devices):
                return devices[idx]["address"]

        print("Invalid input.")


async def scan_and_select_from_list(sensor_key: str, sensor_desc: str) -> str:
    while True:
        print(f"\n=== Scan for {sensor_key}: {sensor_desc} ===")
        print("Wake only the target cycling sensor if possible.")

        devices = await scan_ble()

        print("\nDetected BLE devices (named only):")
        print_devices(devices)

        if not devices:
            raw = input("\nNo devices found. Press Enter to rescan or type s to skip: ").strip().lower()
            if raw == "s":
                return ""
            continue

        raw = input(
            f"\nEnter the index number to save for {sensor_key}, r to rescan, or s to skip: "
        ).strip().lower()

        if raw == "s":
            return ""
        if raw == "r":
            continue
        if raw.isdigit():
            idx = int(raw)
            if 0 <= idx < len(devices):
                return devices[idx]["address"]

        print("Invalid input.")


def print_saved_addresses(selected: dict[str, str]):
    print("\nSaved addresses:")
    print("================")
    for key, _ in WIT_SENSORS + OTHER_SENSORS:
        print(f"{key}={selected.get(key, '') or '(not assigned)'}")


def save_config(selected: dict[str, str]):
    config_lines = ["# Unified sensor address config", ""]
    for key, _ in WIT_SENSORS + OTHER_SENSORS:
        config_lines.append(f"{key}={selected.get(key, '')}")

    OUTPUT_CONFIG.write_text("\n".join(config_lines) + "\n", encoding="utf-8")


async def main():
    await print_intro()

    selected = {}

    print("\nStarting WIT sensor pairing...\n")
    for key, desc in WIT_SENSORS:
        addr = await scan_wit_one_by_one(key, desc)
        selected[key] = addr
        if addr:
            print(f"Saved {key} = {addr}")
        else:
            print(f"Skipped {key}")

    print("\nStarting other cycling sensor pairing...\n")
    for key, desc in OTHER_SENSORS:
        addr = await scan_and_select_from_list(key, desc)
        selected[key] = addr
        if addr:
            print(f"Saved {key} = {addr}")
        else:
            print(f"Skipped {key}")

    save_config(selected)
    print_saved_addresses(selected)

    print(f"\nSaved address file to: {OUTPUT_CONFIG.resolve()}")
    print("Done.")


if __name__ == "__main__":
    asyncio.run(main())