# pip install pymavlink
# Usage:
#   python log_rpms_minimal.py --source udp:127.0.0.1:14550 --out logs/telemetry.csv --rate 5

'''
NOTE: PWM -> RPM Conversion is approximated based on these links: 
https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
https://colab.research.google.com/drive/1gRHsYeZZFHkedNIgAN7U-y4yvfyfVwQo
RPM = -2.75354937114324(e-5)*((PWM)**3) + 0.124618297156559*((PWM)**2) - 175.652241359806*(PWM) + 75977.1611641401
'''

import argparse, csv, math, os, time
from datetime import datetime
from pymavlink import mavutil

def parse_args():
    p = argparse.ArgumentParser(description="Log flight metrics to CSV")
    p.add_argument("--source", required=True, help="udp:127.0.0.1:14550 or path/to/flight.tlog")
    # p.add_argument("--out", default="telemetry.csv", help="Output CSV file")
    # p.add_argument("--rate", type=float, default=5.0, help="Rows per second (Hz)")
    return p.parse_args()

def sum_valid_mv(arr):
    return sum(v for v in arr if v not in (0, 0xFFFF))

def calculate_rpm(pwm):
    rpm = (-2.75354937114324e-5) * (pwm ** 3) + 0.124618297156559 * (pwm ** 2) - (175.652241359806 * pwm) + 75977.1611641401
    return rpm

def main():
    args = parse_args()
    filename = f"telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

    m = mavutil.mavlink_connection(args.source)
    m.wait_heartbeat()
    print(f"Connected to {args.source} (sys {m.target_system}, comp {m.target_component})")
    
    pwm = {"Prop 1": None, "Prop 2": None, "Prop 3": None, "Prop 4": None}
    rpm = {"Prop 1": None, "Prop 2": None, "Prop 3": None, "Prop 4": None}
    airspeed     = None             # airspeed      m/s   (VFR_HUD.airspeed)
    throttle     = None             # throttle      %     (VFR_HUD.throttle)
    climb        = None             # climb         m/s   (VFR_HUD.climb)
    current      = None             # current       A     (SYS_STATUS.current_battery cA -> A)
    voltage      = None             # voltage       V     (SYS_STATUS.voltage_battery mV -> V, or BATTERY_STATUS)
    power        = None             # power         W     (computed V*I)
    roll         = None             # roll          rad   (ATTITUDE.roll )
    pitch        = None             # pitch         rad   (ATTITUDE.pitch)
    yaw          = None             # yaw           rad   (ATTITUDE.yaw  )
    altitude_msl = None             # altitude_msl  m     (VFR_HUD.altitude)
    air_pressure = None             # air_pressure  hPa   (SCALED_PRESSURE.press_abs)
    # rpm1      = None
    # rpm2      = None
    # index    = None
    
    with open(filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "PWM1","PWM2","PWM3","PWM4",
            "RPM1","RPM2","RPM3","RPM4",
            "Airspeed","Throttle","ClimbRate",
            "Current","Voltage","Power",
            "Roll","Pitch","Yaw",
            "AltitudeMSL", "AirPressure",
            # "RPM-1","RPM-2"
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
                    altitude_msl = float(msg.alt) if msg.alt is not None else None
                
                elif mt == "SCALED_PRESSURE":
                    air_pressure = float(msg.press_abs) if msg.press_abs is not None else None

                ###################################
                # Test RPM
                
                # param set SERIAL5_PROTOCOL 38
                # param set SERIAL5_BAUD 500000
                # param set SERVO_FTW_MASK 15
                # param set SIM_FTOWESC_ENA 1
                
                # elif mt == "EFI_STATUS":
                #     rpm1 = msg.rpm if hasattr(msg, "rpm") else None
                    
                # elif mt == "RAW_RPM":
                #     index = msg.index if hasattr(msg, "index") else -1
                #     rpm2 = msg.rpm if hasattr(msg, "rpm") else 0 
                    
                # elif mt == "RPM":
                #     try:
                #         rpm1 = float(msg.rpm1) if msg.rpm1 is not None else None
                #         rpm2 = float(msg.rpm2) if msg.rpm2 is not None else None
                #     except Exception as e:
                #         print(f"Error parsing RPM: {e}")
                # Not working: THIS IS GIVING 0.0 FOR ALL RPMs!        
                # elif mt == "ESC_TELEMETRY_1_TO_4":
                #     try:
                #         arr = list(msg.rpm)
                #         for i, val in enumerate(arr[:4], start=1):
                #             rpm[f"Prop {i}"] = float(val)
                #     except Exception:
                #         pass
                ##################################
                
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
                    pwm["Prop 1"] = getattr(msg, "servo1_raw", 0)
                    pwm["Prop 2"] = getattr(msg, "servo2_raw", 0)
                    pwm["Prop 3"] = getattr(msg, "servo3_raw", 0)
                    pwm["Prop 4"] = getattr(msg, "servo4_raw", 0)
                    
                    rpm["Prop 1"] = calculate_rpm(pwm["Prop 1"])
                    rpm["Prop 2"] = calculate_rpm(pwm["Prop 2"])
                    rpm["Prop 3"] = calculate_rpm(pwm["Prop 3"])
                    rpm["Prop 4"] = calculate_rpm(pwm["Prop 4"])
                    
                    row = [
                        pwm["Prop 1"], pwm["Prop 2"], pwm["Prop 3"], pwm["Prop 4"],
                        rpm["Prop 1"], rpm["Prop 2"], rpm["Prop 3"], rpm["Prop 4"],
                        airspeed, throttle, climb,
                        current, voltage, power,
                        roll, pitch, yaw,
                        altitude_msl, air_pressure,
                        # rpm1, rpm2
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
        
        finally:
            f.close()
            print(f"File saved to: {os.path.abspath(filename)}")


if __name__ == "__main__":
    main()
