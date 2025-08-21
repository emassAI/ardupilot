# pip install pymavlink
# Usage:
#   python log_pwm_and_metrics.py --source udp:127.0.0.1:14550 --out logs/telemetry.csv
#   python log_pwm_and_metrics.py --source ./flight.tlog --out logs/extract.csv

import argparse, csv, math, os, time
from datetime import datetime
from pymavlink import mavutil

def parse_args():
    p = argparse.ArgumentParser(description="Log PWM + key flight metrics to CSV")
    p.add_argument("--source", required=True, help="udp:127.0.0.1:14550 or path/to/flight.tlog")
    # p.add_argument("--out", default="telemetry.csv", help="Output CSV file")
    return p.parse_args()

def sum_valid_mv(arr):
    # BATTERY_STATUS.voltages[] entries are mV; 0 or 65535 mean unused.
    return sum(v for v in arr if v not in (0, 0xFFFF))

def main():
    args = parse_args()
    filename = f"telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

    m = mavutil.mavlink_connection(args.source)
    m.wait_heartbeat()
    print(f"Connected to {args.source} (sys {m.target_system}, comp {m.target_component})")

    # Keep latest values here; start as None to honor your completeness check.
    airspeed_val  = None   # m/s       (VFR_HUD.airspeed)
    altitude_msl  = None   # m         (GLOBAL_POSITION_INT.alt mm -> m)
    throttle      = None   # %         (VFR_HUD.throttle)
    climb_rate    = None   # m/s       (VFR_HUD.climb)
    # air_pressure  = None   # hPa       (SCALED_PRESSURE.press_abs)
    current       = None   # A         (SYS_STATUS.current_battery cA -> A)
    voltage       = None   # V         (SYS_STATUS.voltage_battery mV -> V, or BATTERY_STATUS)
    power         = None   # W         (computed V * A)
    tilt_x        = None   # deg roll  (ATTITUDE.roll rad -> deg)
    tilt_y        = None   # deg pitch (ATTITUDE.pitch rad -> deg)
    tilt_z        = None   # deg yaw   (ATTITUDE.yaw rad -> deg)
    # speed         = None   # m/s ground (VFR_HUD.groundspeed, else from GLOBAL_POSITION_INT vx/vy)
    rpm_map = {"Prop 1": None, "Prop 2": None, "Prop 3": None, "Prop 4": None}
    
    # For groundspeed fallback:
    vx = vy = None  # from GLOBAL_POSITION_INT (cm/s)

    # Open CSV and write header once
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Propeller", "PWM", "RPM"
            "Airspeed", "Altitude", "Throttle", "ClimbRate",
            "current_A", "voltage_V", "power_W",
            "Roll", "Pitch", "Yaw"
        ])
        csvfile.flush()

        try:
            while True:
                msg = m.recv_match(blocking=True, timeout=2)
                if msg is None:
                    continue
                mtype = msg.get_type()

                # ---- Update state from incoming messages ----
                if mtype == "VFR_HUD":
                    airspeed_val = float(msg.airspeed) if msg.airspeed is not None else None
                    throttle     = float(msg.throttle) if msg.throttle is not None else None
                    climb_rate   = float(msg.climb)    if msg.climb is not None else None
                    # speed from VFR_HUD when available
                    try:
                        speed = float(msg.groundspeed)
                    except AttributeError:
                        pass

                elif mtype == "GLOBAL_POSITION_INT":
                    altitude_msl = float(msg.alt) / 1000.0  # mm -> m
                    vx = float(msg.vx) / 100.0 if msg.vx is not None else vx  # cm/s -> m/s
                    vy = float(msg.vy) / 100.0 if msg.vy is not None else vy
                    # if speed is None and (vx is not None and vy is not None):
                    #     speed = math.hypot(vx, vy)

                elif mtype == "ATTITUDE":
                    # radians -> degrees
                    tilt_x = math.degrees(msg.roll)  if msg.roll  is not None else None
                    tilt_y = math.degrees(msg.pitch) if msg.pitch is not None else None
                    tilt_z = math.degrees(msg.yaw)   if msg.yaw   is not None else None

                elif mtype == "SYS_STATUS":
                    if msg.voltage_battery and msg.voltage_battery > 0:
                        voltage = float(msg.voltage_battery) / 1000.0  # mV -> V
                    if msg.current_battery is not None and msg.current_battery != -1:
                        current = float(msg.current_battery) / 100.0   # cA -> A
                    if (voltage is not None) and (current is not None):
                        power = voltage * current

                elif mtype == "BATTERY_STATUS":
                    # Fallbacks if SYS_STATUS not available
                    if (voltage is None) and hasattr(msg, "voltages"):
                        pack_mv = sum_valid_mv(msg.voltages)
                        if pack_mv > 0:
                            voltage = pack_mv / 1000.0
                    if (current is None) and (getattr(msg, "current_battery", -1) != -1):
                        current = float(msg.current_battery) / 100.0
                    if (voltage is not None) and (current is not None) and (power is None):
                        power = voltage * current

                # elif mtype == "SCALED_PRESSURE":
                #     air_pressure = float(msg.press_abs)  # hPa
                    
                elif mtype == "RPM":
                    rpm["Prop 1"] = getattr(msg, "rpm1", rpm["Prop 1"])
                    rpm["Prop 2"] = getattr(msg, "rpm2", rpm["Prop 2"])

                elif mtype == "ESC_TELEMETRY_1_TO_4":
                    try:
                        arr = list(msg.rpm)
                        for i, val in enumerate(arr[:4], start=1):
                            rpm[f"Prop {i}"] = float(val)
                    except Exception:
                        pass

                elif mtype == "ESC_STATUS":
                    try:
                        arr = list(msg.rpm)
                        for i, val in enumerate(arr[:4], start=1):
                            rpm[f"Prop {i}"] = float(val)
                    except Exception:
                        pass


                elif mtype == "ESC_TELEMETRY_1_TO_4":
                    # In pymavlink this is an array; guard in case field is absent
                    try:
                        arr = list(msg.rpm)  # typically length 4
                        for i, val in enumerate(arr[:4], start=1):
                            rpm_map[f"Prop {i}"] = float(val)
                    except Exception:
                        pass

                elif mtype == "ESC_STATUS":
                    # Some stacks send RPMs here, also as an array
                    try:
                        arr = list(msg.rpm)
                        for i, val in enumerate(arr[:4], start=1):
                            rpm_map[f"Prop {i}"] = float(val)
                    except Exception:
                        pass

                # ---- When a new SERVO_OUTPUT_RAW arrives, emit 4 rows (Prop 1..4) ----
                elif mtype == "SERVO_OUTPUT_RAW":
                    # capture PWM 1..4
                    pwm["Prop 1"] = getattr(msg, "servo1_raw", pwm["Prop 1"])
                    pwm["Prop 2"] = getattr(msg, "servo2_raw", pwm["Prop 2"])
                    pwm["Prop 3"] = getattr(msg, "servo3_raw", pwm["Prop 3"])
                    pwm["Prop 4"] = getattr(msg, "servo4_raw", pwm["Prop 4"])

                    # build ONE snapshot row with 4 RPM columns
                    row = [
                        ts_rel(msg),
                        pwm["Prop 1"], pwm["Prop 2"], pwm["Prop 3"], pwm["Prop 4"],
                        rpm["Prop 1"], rpm["Prop 2"], rpm["Prop 3"], rpm["Prop 4"],
                        airspeed_val, altitude_msl, throttle, climb_rate, air_pressure,
                        current, voltage, power,
                        tilt_x, tilt_y, tilt_z,
                        speed if speed is not None else (math.hypot(vx, vy) if (vx is not None and vy is not None) else None),
                    ]

                    print("Debug ->", row)

                    # require all fields present
                    if all(v is not None for v in row[1:]):
                        writer.writerow(row)
                        csvfile.flush()
                        print(f"Logged 1 row with 4 RPMs")
                    else:
                        print("Row Skipped due to incomplete data")

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nStopped. CSV saved to:", filename)

if __name__ == "__main__":
    main()
