from mavros_msgs.msg import Waypoint

MAV_GLOBAL_FRAME = 3
MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL      = 20

def waypoint(lat, lon, alt, delay):
    w = Waypoint()
    w.frame = MAV_GLOBAL_FRAME 
    w.command = MAV_CMD_WAYPOINT
    w.is_current = False
    w.autocontinue = True
    w.param1 = delay # Hold time in mession
    w.param2 = 2     # Position trashold in meters
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def rtl():
    w = Waypoint()
    w.command = MAV_CMD_RTL
    return w

def mission_planner(lat0, lon0, lat, lon):
    return [ waypoint(lat0, lon0, 20, 5)
           , waypoint(lat,  lon,  50, 10)
           , rtl() ]
