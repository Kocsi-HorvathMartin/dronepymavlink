from pymavlink import mavutil
import math
import time

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 33, 0,0,0,0,0,1)
    msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

def mode(mode):
    connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode)

def arm(arm):
        connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, arm, 0,0,0,0,0,0)
        msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
poz=akt_poz()
print(poz)

connection.mav.mission_count_send(connection.target_system,
                                    connection.target_component,
                                    4)
msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
print(msg)
connection.mav.mission_item_int_send(connection.target_system,
                                     connection.target_component,
                                     0,
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                     1,
                                     1,
                                     0.0,
                                     2.0,
                                     1.0,
                                     math.nan,
                                     poz.lat,
                                     poz.lon,
                                     0)
msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
print(msg)
connection.mav.mission_item_int_send(connection.target_system,
                                     connection.target_component,
                                     1,
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     0,
                                     1,
                                     0,
                                     0,
                                     0,
                                     math.nan,
                                     poz.lat,
                                     poz.lon,
                                     2)

msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
print(msg)
connection.mav.mission_item_int_send(connection.target_system,
                                     connection.target_component,
                                     2,
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                     0,
                                     1,
                                     0.0,
                                     2.0,
                                     1.0,
                                     math.nan,
                                     -353617755,
                                     1491651469,
                                     2.0)

msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
print(msg)
connection.mav.mission_item_int_send(connection.target_system,
                                     connection.target_component,
                                     3,
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                                     0,
                                     0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     math.nan,
                                     -353617755,
                                     1491651469,
                                     0.0)
msg=connection.recv_match(type='MISSION_ACK', blocking=True)
print(msg)
connection.mav.command_long_send(connection.target_system, 
                                connection.target_component,
                                mavutil.mavlink.MAV_CMD_MISSION_START,
                                0,0,0,0,0,0,0,0)
mode(4)
arm(1)
mode(3)