# pip install pymavlink
# Usage:
#   python log_rpms_minimal.py --source udp:127.0.0.1:14550 --out logs/telemetry.csv --rate 5

import argparse, csv, math, os, time
from datetime import datetime
from pymavlink import mavutil

def parse_args():
    p = argparse.ArgumentParser(description="Log RPM1..4 + flight metrics to CSV")
    p.add_argument("--source", required=True, help="udp:127.0.0.1:14550 or path/to/flight.tlog")
    # p.add_argument("--out", default="telemetry.csv", help="Output CSV file")
    # p.add_argument("--rate", type=float, default=5.0, help="Rows per second (Hz)")
    return p.parse_args()

def sum_valid_mv(arr):
    return sum(v for v in arr if v not in (0, 0xFFFF))

def main():
    args = parse_args()
    filename = f"telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

    m = mavutil.mavlink_connection(args.source)
    m.wait_heartbeat()
    print(f"Connected to {args.source} (sys {m.target_system}, comp {m.target_component})")
    
    pwm = {"Prop 1": None, "Prop 2": None, "Prop 3": None, "Prop 4": None}
    airspeed = None                 # m/s (VFR_HUD.airspeed)
    throttle = None                 # %   (VFR_HUD.throttle)
    climb    = None                 # m/s (VFR_HUD.climb)
    current  = None                 # A   (SYS_STATUS.current_battery cA -> A)
    voltage  = None                 # V   (SYS_STATUS.voltage_battery mV -> V, or BATTERY_STATUS)
    power    = None                 # W   (computed V*I)
    roll     = None                 # rad (ATTITUDE.roll )
    pitch    = None                 # rad (ATTITUDE.pitch)
    yaw      = None                 # rad (ATTITUDE.yaw  )

    with open(filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "PWM1","PWM2","PWM3","PWM4",
            "Airspeed","Throttle","ClimbRate",
            "Current","Voltage","Power",
            "Roll","Pitch","Yaw"
        ])
        f.flush()

        try:
            while True:
                msg = m.recv_match(blocking=True, timeout=2)
                if not msg:
                    continue
                t = time.time()

                mt = msg.get_type()
                if mt == "VFR_HUD":
                    airspeed = float(msg.airspeed) if msg.airspeed is not None else None
                    throttle = float(msg.throttle) if msg.throttle is not None else None
                    climb    = float(msg.climb)    if msg.climb    is not None else None

                elif mt == "ATTITUDE":
                    if msg.roll  is not None: roll  = msg.roll
                    if msg.pitch is not None: pitch = msg.pitch
                    if msg.yaw   is not None: yaw   = msg.yaw

                elif mt == "SYS_STATUS":
                    if msg.voltage_battery and msg.voltage_battery > 0:
                        voltage = float(msg.voltage_battery) / 1000.0
                    if msg.current_battery is not None and msg.current_battery != -1:
                        current = float(msg.current_battery) / 100.0
                    if (voltage is not None) and (current is not None):
                        power = voltage * current

                elif mt == "BATTERY_STATUS":
                    if (voltage is None) and hasattr(msg, "voltages"):
                        pack_mv = sum_valid_mv(msg.voltages)
                        if pack_mv > 0:
                            voltage = pack_mv / 1000.0
                    if (current is None) and (getattr(msg, "current_battery", -1) != -1):
                        current = float(msg.current_battery) / 100.0
                    if (voltage is not None) and (current is not None) and (power is None):
                        power = voltage * current

                elif mt == "SERVO_OUTPUT_RAW":
                    # capture PWM 1..4
                    pwm["Prop 1"] = getattr(msg, "servo1_raw", pwm["Prop 1"])
                    pwm["Prop 2"] = getattr(msg, "servo2_raw", pwm["Prop 2"])
                    pwm["Prop 3"] = getattr(msg, "servo3_raw", pwm["Prop 3"])
                    pwm["Prop 4"] = getattr(msg, "servo4_raw", pwm["Prop 4"])

                    row = [
                        pwm["Prop 1"], pwm["Prop 2"], pwm["Prop 3"], pwm["Prop 4"],
                        airspeed, throttle, climb,
                        current, voltage, power,
                        roll, pitch, yaw,
                    ]

                    print("Debug ->", row)

                    # require all fields present
                    if all(v is not None for v in row[1:]):
                        w.writerow(row)
                        f.flush()
                        print(f"Logged 1 row with 4 PWMs")
                    else:
                        print("Row Skipped due to incomplete data")
            time.sleep(0.02)
        
        except KeyboardInterrupt:
            print("\nSaved to:", filename)

if __name__ == "__main__":
    main()
