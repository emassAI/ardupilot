from pymavlink import mavutil
import csv
from datetime import datetime
import time
import os

#Connect to vehicle
print("Connecting to vehicle...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("Connected to vehicle")

print("Requesting MAVLink data streams at 50Hz...")
stream_types = [
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
]

for stream in stream_types:
    master.mav.request_data_stream_send(
    target_system=master.target_system,
    target_component=master.target_component,
    req_stream_id=stream,
    req_message_rate = 50 , #Hz
    start_stop = 1     #Start stream
)

#Setup CSV Logging
filename = f"telemetry_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
print("Will be saved at:",os.path.abspath(filename))

try:
    csvfile = open(filename,'w',newline='')
    writer = csv.writer(csvfile)

    #Write the header for CSV
    header = ["Propeller", "PWM",
    "Airspeed", "Altitude", "AirPressure",
    "Current","Voltage",
    "TiltX","TiltY","TiltZ","Speed"]
    
    writer.writerow(header)
    print(f"Header written: {header}")


    while True:
        t= time.time()

        #Read MAVLink messages
        servo = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        pressure = master.recv_match(type='SCALED_PRESSURE', blocking=False) 
        battery = master.recv_match(type='BATTERY_STATUS', blocking=False)
        attitude = master.recv_match(type='ATTITUDE', blocking=False)
        airspeed = master.recv_match(type='VFR_HUD', blocking=False) #gives airspeed

        # Extract values
        #rpm = getattr(servo,'servo1_raw', None)
        airspeed_val = getattr(airspeed,'airspeed',None)
        altitude = getattr(airspeed,'alt',None)             # <-- FIXED: use VFR_HUD.alt (meters)
        air_pressure = getattr(pressure,'press_abs', None)  
        current = battery.current_battery / 100.0 if battery and battery.current_battery != -1 else None
        voltage = battery.voltages[0] / 1000.0 if battery and battery.voltages[0] != 0 else None
        tilt_x = getattr(attitude,'roll', None)
        tilt_y = getattr(attitude,'pitch',None)
        tilt_z = getattr(attitude,'yaw', None)
        speed = getattr(airspeed, 'groundspeed',None)

        pwm_values = {
            "Prop 1": getattr(servo, "servo1_raw", None),
            "Prop 2": getattr(servo, "servo2_raw", None),
            "Prop 3": getattr(servo, "servo3_raw", None),
            "Prop 4": getattr(servo, "servo4_raw", None),
        }

        for prop, pwm in pwm_values.items():
            row = [prop, pwm, 
            airspeed_val, altitude, air_pressure, 
            current, voltage, 
            tilt_x, tilt_y, tilt_z, speed]
        
            print("Debug ->", row)
        
            if all(v is not None for v in row[1:]):
                writer.writerow(row)
                csvfile.flush()
                print(f"Logged: {row}")
            else:
                print("Incomplete data -skipping row")
        
        time.sleep(0.02) 

except KeyboardInterrupt:
    print("\n Logging interrupted by user.")

finally:
    csvfile.close()
    print(f"File saved to: {os.path.abspath(filename)}")