from pymavlink import mavutil
import json

def formaz(coordinate):
     lon=round(coordinate[0],7)*pow(10,7)
     lat=round(coordinate[1],7)*pow(10,7)
     return int(lat),int(lon)

def beolvas(file_path):
    global mission
    # Open the JSON file with 'utf-8-sig' encoding
    with open(file_path, 'r', encoding='utf-8-sig') as file:
    # Load the JSON content
        data = json.load(file)

    # Now `data` contains the parsed JSON content
    features=data['features']
    for i in range(len(features)):
        print(i)
        coordinates=features[i]['geometry'][0]['horizontalProjection']['coordinates'][0]
        for j in range(len(coordinates)):
            lat,lon=formaz(coordinates[j])
            item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                 0,0,
                 len(coordinates),0,0,0,lat,lon,0)
    feltolt()

def item(frame, command, current, autocontinue, param1, param2, param3, param4, param5, param6, param7):    #Egy mission elem hozzáfűzése az adott missionhöz
     global mission
     mission.append([frame, command,current, autocontinue, param1, param2, param3,param4,param5,param6,param7])

def feltolt():          #Mission feltöltése
    global mission
    n=len(mission)
    connection.mav.mission_count_send(connection.target_system,
                                    connection.target_component,
                                    n,1,0)
    for i in range(n):
        msg=connection.recv_match(type='MISSION_REQUEST', blocking=True)
        #print(msg)
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
                                    mission[i][10],
                                    1)
    msg=connection.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    print(msg)

#-----Main-----
connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

connection.mav.mission_clear_all_send(connection.target_system, connection.target_component,1)
msg=connection.recv_match(type='MISSION_ACK',blocking=True)
print(msg)
mission=[]
beolvas('/home/kocsi-horvath/Documents/uav_202406181054.json')