from pymavlink import mavutil
import json
from datetime import datetime

def validate_uas_zone(json_data):
    required_keys = {
        "title": str,
        "description": str,
        "features": list
    }

    feature_keys = {
        "identifier": str,
        "country": str,
        "name": str,
        "type": str,
        "restriction": str,
        "reason": list,
        "applicability": list,
        "zoneAuthority": list,
        "geometry": list,
        "message": str
    }

    geometry_keys = {
        "upperLimit": int,
        "lowerLimit": int,
        "uomDimensions": str,
        "upperVerticalReference": str,
        "lowerVerticalReference": str,
        "horizontalProjection": dict
    }

    horizontal_projection_keys = {
        "type": str,
        "coordinates": list
    }

    # Load JSON data if it's a string
    if isinstance(json_data, str):
        data = json.loads(json_data)
    else:
        data = json_data

    # Validate top-level keys
    for key, val_type in required_keys.items():
        if key not in data or not isinstance(data[key], val_type):
            return f"Missing or incorrect type for key: {key}"

    # Validate features array
    features = data["features"]
    if not features or not isinstance(features, list):
        return "Features list is missing or not a list"

    for feature in features:
        for key, val_type in feature_keys.items():
            if key not in feature or not isinstance(feature[key], val_type):
                return f"Missing or incorrect type for key in feature: {key}"

        # Validate geometry list in each feature
        for geometry in feature["geometry"]:
            for key, val_type in geometry_keys.items():
                if key not in geometry or not isinstance(geometry[key], val_type):
                    return f"Missing or incorrect type for key in geometry: {key}"

            # Validate horizontal projection
            horizontal_projection = geometry["horizontalProjection"]
            for key, val_type in horizontal_projection_keys.items():
                if key not in horizontal_projection or not isinstance(horizontal_projection[key], val_type):
                    return f"Missing or incorrect type for key in horizontal projection: {key}"

    return "JSON data is valid."

#Koordináták formázás (10**7 és int alakra) és helyes sorrendbe rakása
def formaz(coordinate):
     lon=round(coordinate[0],7)*pow(10,7)
     lat=round(coordinate[1],7)*pow(10,7)
     return int(lat),int(lon)

def input_mission_time():
    startdate = input("Add meg a kezdő dátumot és időt (ÉÉÉÉ-HH-NN ÓÓ:PP:SS formátumban): ")
    enddate = input("Add meg a befejező dátumot és időt (ÉÉÉÉ-HH-NN ÓÓ:PP:SS formátumban): ")

    # Dátum konvertálása datetime formátumba
    try:
        mission_startdate = datetime.strptime(startdate, "%Y-%m-%d %H:%M:%S")
        mission_enddate= datetime.strptime(enddate, "%Y-%m-%d %H:%M:%S")

        return mission_startdate, mission_enddate
    except ValueError:
        print("Hibás dátumformátum! Kérlek, használd az ÉÉÉÉ-HH-NN ÓÓ:PP:SS formátumot.")

#Ellenőrzi, hogy az adott fence érvényes-e a jelenlegi időpillanatra
def check_time(applicability):
    mission_start, mission_end = input_mission_time()
    startdate=applicability['startDateTime']
    enddate=applicability['endDateTime']
    startdate_format=datetime(int(startdate[0:4]),int(startdate[5:7]),int(startdate[8:10]),int(startdate[11:13]),int(startdate[14:16]),int(startdate[17:19]))
    enddate_format=datetime(int(enddate[0:4]),int(enddate[5:7]),int(enddate[8:10]),int(enddate[11:13]),int(enddate[14:16]),int(enddate[17:19]))
    if (mission_start>=startdate_format and mission_end<=enddate_format) or (mission_start>=startdate_format and mission_start<=enddate_format)or (mission_end>=startdate_format and mission_end<=enddate_format):
        return True
    else:
        return False

#json fájlból fence coordináták kiolvasása és beírása a mission listába
def beolvas(file_path):
    global mission
    # Open the JSON file with 'utf-8-sig' encoding
    with open(file_path, 'r', encoding='utf-8-sig') as file:
    # Load the JSON content
        data = json.load(file)
    return data

#Aktuális magásságnak és időpontnak megfelelő fencek első 67 pontjának hozzáadása feltöltésre várókhoz
def fence_feltolt(data):
    k=0
    for i in range(len(data['features'])):
        #if check_time(data['features'][i]['applicability'][0]):
        if True: #i==36 or i==0:
            id=i
            felso=data['features'][i]['geometry'][0]['upperLimit']
            also=data['features'][i]['geometry'][0]['lowerLimit']
            print(also," ",felso)
            coordinates=data['features'][i]['geometry'][0]['horizontalProjection']['coordinates'][0]
            k+=1
            for j in range(len(coordinates)):
                lat,lon=formaz(coordinates[j])
                item(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    0,0,len(coordinates),id,0,felso,lat,lon,also)
    print(k)

#Egy mission elem hozzáfűzése az adott missionhöz
def item(frame, command, current, autocontinue, param1, param2, param3, param4, param5, param6, param7):
     global mission
     mission.append([frame, command,current, autocontinue, param1, param2, param3,param4,param5,param6,param7])

#Mission feltöltése
def mission_feltolt():
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

def fence_enable():
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                                     0,1,0,0,0,0,0,0
    )
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(msg)



file_path='/home/kocsi-horvath/Documents/uav_202411121012.json'

#-----Main-----
data=beolvas(file_path)
if(validate_uas_zone(data)=="JSON data is valid."):
    connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

    connection.mav.mission_clear_all_send(connection.target_system, connection.target_component,1)              #Összes fence pont törlése
    msg=connection.recv_match(type='MISSION_ACK',blocking=True)
    print(msg)

    fence_enable()

    data=beolvas(file_path)
    mission=[]
    fence_feltolt(data)
    if len(mission)!=0:
        mission_feltolt()
    else:
        print("Nincs megfelelő elem")
    print("Kész")