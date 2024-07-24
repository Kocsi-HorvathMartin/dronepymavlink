from pymavlink import mavutil
import json

def akt_poz():              #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 33, 0,0,0,0,0,1)
    msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

def formaz(coordinate):     #Koordináták formázás (10**-7 és int alakra) és helyes sorrendbe rakása
     lon=round(coordinate[0],7)*pow(10,7)
     lat=round(coordinate[1],7)*pow(10,7)
     return int(lat),int(lon)

def beolvas(file_path):     #json fájlból fence coordináták kiolvasása és beírása a mission listába
    global mission
    # Open the JSON file with 'utf-8-sig' encoding
    with open(file_path, 'r', encoding='utf-8-sig') as file:
    # Load the JSON content
        data = json.load(file)
    return data

def fence_feltolt(data,poz):#Aktuális magásságnak megfelelő fencek első 67 pontjának hozzáadása feltöltésre
    poz=poz.relative_alt/1000
    k=0
    if poz<0:
        poz=0
    for i in range(len(data['features'])):
        felso=data['features'][i]['geometry'][0]['upperLimit']
        also=data['features'][i]['geometry'][0]['lowerLimit']
        if poz>=also and poz<=felso:
            coordinates=data['features'][i]['geometry'][0]['horizontalProjection']['coordinates'][0]
            if len(mission)+len(coordinates)<=67:
                k+=1
                for j in range(len(coordinates)):
                    lat,lon=formaz(coordinates[j])
                    item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                        0,0,
                        len(coordinates),0,0,0,lat,lon,0)
    print(k)

def item(frame, command, current, autocontinue, param1, param2, param3, param4, param5, param6, param7):    #Egy mission elem hozzáfűzése az adott missionhöz
     global mission
     mission.append([frame, command,current, autocontinue, param1, param2, param3,param4,param5,param6,param7])

def mission_feltolt():       #Mission feltöltése
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

connection.mav.mission_clear_all_send(connection.target_system, connection.target_component,1)              #Összes fence pont törlése
msg=connection.recv_match(type='MISSION_ACK',blocking=True)
print(msg)

data=beolvas('/home/kocsi-horvath/Documents/uav_202406181054.json')
poz=akt_poz()
mission=[]
fence_feltolt(data,poz)
if len(mission)!=0:
    mission_feltolt()
else:
    print("There is not any fence at this altitude: ", poz.relative_alt/1000,"m")