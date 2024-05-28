from pymavlink import mavutil
import math

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 33, 0,0,0,0,0,1)
    msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

def mode(mode):          #Mód váltás azonosító alapján
    connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode)

def arm(arm):            #Armolás/disarmolás
        connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                 0, arm, 0,0,0,0,0,0)
        msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

def item(frame, command, current, autocontinue, param1, param2, param3, param4, param5, param6, param7):    #Egy mission elem hozzáfűzése az adott missionhöz
     global mission
     mission.append([frame, command,current, autocontinue, param1, param2, param3,param4,param5,param6,param7])

def feltolt():          #Mission feltöltése
    global mission
    n=len(mission)
    connection.mav.mission_count_send(connection.target_system,
                                    connection.target_component,
                                    n)
    for i in range(n):
        msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
        print(msg)
        connection.mav.mission_item_int_send(connection.target_system,
                                    connection.target_component,
                                    i,
                                    mission[i][0],
                                    mission[i][1],
                                    mission[i][2],
                                    mission[i][3],
                                    mission[i][4],
                                    mission[i][5],
                                    mission[i][6],
                                    mission[i][7],
                                    mission[i][8],
                                    mission[i][9],
                                    mission[i][10])
    msg=connection.recv_match(type='MISSION_ACK', blocking=True)
    print(msg)

def start():            #Mission elindítása
    connection.mav.command_long_send(connection.target_system, 
                                connection.target_component,
                                mavutil.mavlink.MAV_CMD_MISSION_START,
                                0,0,0,0,0,0,0,0)
    mode(4) #Guided mode
    arm(1)  #Armolás
    mode(3) #Auto mód

#-----Main-----
connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
poz=akt_poz()
mission=[]

item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,1,1,0.0,2.0,1.0,math.nan,poz.lat,poz.lon,0)         #Jelenlegi pozíció megadása
item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,1,0,0,0,math.nan,poz.lat,poz.lon,2)                #Felszállás
item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,1,0.0,2.0,1.0,math.nan,-353617755,1491651469,2.0) #Waypointra mozgás
item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0.0,0.0,0.0,math.nan,-353617755,1491651469,0.0)     #Leszállás

feltolt()
start()