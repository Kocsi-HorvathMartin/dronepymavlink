from pymavlink import mavutil
import time
import os

def felszall():         #Felszállás
    connection.mav.command_long_send(connection.target_system,                      #GUIDED MODE-ba váltás
                                 connection.target_component,
                                 mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                 0, 1, 4, 0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    connection.mav.command_long_send(connection.target_system,                      #Armolás
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, 1, 0,0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    connection.mav.command_long_send(connection.target_system,                      #Felszállás
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                 0, 0,0,0,0,0,0, 2)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


def leszall():          #Leszállás
    connection.mav.command_long_send(connection.target_system,                       #Leszállás
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                                     0,0,0,0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    
    connection.mav.command_long_send(connection.target_system,                      #Disarmolás
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, 0, 0,0,0,0,0,0)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    
def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 33, 0,0,0,0,0,1)
    msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
#felszall()
poz=akt_poz()
print(poz)