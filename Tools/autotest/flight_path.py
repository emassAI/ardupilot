# pip install pymavlink
from pymavlink import mavutil
import time, math

# ---- configuration ----
CONN_STR   = "udp:127.0.0.1:14540"  # ArduPilot SITL default
ALT_REL_M  = 60                     # relative altitude (m)
RADIUS_M   = 2.5                    # how close counts as “reached”
LOOPS      = 3                      # how many times to repeat the path
SETPOINT_HZ = 5                     # stream rate (Hz)

# small square path: offsets in meters (N,E)
OFFSETS_NE = [(0, 0), (0, 100), (100, 100), (100, 0), (0, 0)]

# ---- helpers ----
def meters_to_deg(lat_deg, north_m, east_m):
    dlat = north_m / 111111.0
    dlon = east_m  / (111111.0 * math.cos(math.radians(lat_deg)))
    return dlat, dlon

def set_guided(master):
    modes = master.mode_mapping()
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             modes["GUIDED"])

def arm(master, arm_it=True):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1 if arm_it else 0, 0,0,0,0,0,0)
    if arm_it: master.motors_armed_wait()
    else:      master.motors_disarmed_wait()

def get_position(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if not msg: return None
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    rel_alt = msg.relative_alt / 1000.0
    return lat, lon, rel_alt

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlamb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlamb/2)**2
    return 2*R*math.asin(math.sqrt(a))

def send_pos_target_global_int(master, lat_deg, lon_deg, alt_rel_m, yaw_deg=None):
    # Build type_mask: ignore everything except position (and yaw if provided)
    ignore = (1<<3)|(1<<4)|(1<<5) | (1<<6)|(1<<7)|(1<<8) | (1<<11)  # vel, acc, yaw_rate ignored
    if yaw_deg is None:
        ignore |= (1<<10)  # also ignore yaw
        yaw = 0.0
    else:
        yaw = math.radians(yaw_deg)

    master.mav.set_position_target_global_int_send(
        int(time.time()*1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        ignore,
        int(lat_deg * 1e7),
        int(lon_deg * 1e7),
        float(alt_rel_m),
        0,0,0,   # vx,vy,vz ignored
        0,0,0,   # ax,ay,az ignored
        yaw,      # yaw (rad) used only if not ignored
        0.0       # yaw_rate ignored
    )

# ---- main ----
def main():
    m = mavutil.mavlink_connection(CONN_STR)
    m.wait_heartbeat()
    print(f"HB from sys {m.target_system} comp {m.target_component}")
    
    # Set drone horizontal speed 4500 cm/s (= 45 m/s) 
    m.mav.param_set_send(m.target_system, m.target_component, b'WPNAV_SPEED', float(4500), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    
    # vertical speeds (1000 cm/s)
    m.mav.param_set_send(m.target_system, m.target_component, b'WPNAV_SPEED_UP', float(1000),  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)  # 3 m/s up
    m.mav.param_set_send(m.target_system, m.target_component, b'WPNAV_SPEED_DN', float(1000),  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)  # 2 m/s down
    
    # read "home" to anchor our offsets
    pos = get_position(m)
    if pos is None:
        raise RuntimeError("No GLOBAL_POSITION_INT yet.")
    home_lat, home_lon, _ = pos

    # build absolute waypoints from NE offsets
    waypoints = []
    for n,e in OFFSETS_NE:
        dlat, dlon = meters_to_deg(home_lat, n, e)
        waypoints.append((home_lat + dlat, home_lon + dlon, ALT_REL_M))

    # Guided + arm
    set_guided(m)
    arm(m, True)
    print("Armed; flying path…")

    period = 1.0 / SETPOINT_HZ
    for loop in range(1, LOOPS+1):
        print(f"\nLoop {loop}/{LOOPS}")
        for i,(lat,lon,alt) in enumerate(waypoints, start=1):
            print(f"  → WP {i}: {lat:.7f}, {lon:.7f}, {alt} m")
            # stream setpoint until reached
            last_send = 0.0
            while True:
                now = time.time()
                if now - last_send >= period:
                    send_pos_target_global_int(m, lat, lon, alt)
                    last_send = now
                cur = get_position(m)
                if cur:
                    d = haversine_m(cur[0], cur[1], lat, lon)
                    alt_err = abs(cur[2] - alt)
                    if d < RADIUS_M and alt_err < 1.5:
                        print(f"    reached (h={d:.1f} m, dz={alt_err:.1f} m)")
                        break

    print("Path loops complete. RTL…")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                            0, 0,0,0,0,0,0,0)

if __name__ == "__main__":
    main()
