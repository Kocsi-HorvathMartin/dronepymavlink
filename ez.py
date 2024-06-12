from pymavlink import mavutil
import math
import os
import time

def get_line_params(x1, y1, x2, y2):                    #Eyenes egyenlete 2 pontra
    if x1 == x2:  # vertical line
        return None, x1
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def find_intersection(x1, y1, x2, y2, x3, y3, x4, y4):  #Van-e két szakasznak metszéspontja
    global inside
    m1, b1 = get_line_params(x1, y1, x2, y2)
    m2, b2 = get_line_params(x3, y3, x4, y4)
    
    if m1 is None:  # First line is vertical
        x = b1
        if x3 == x4:  # Second line is also vertical
            return None
        y = m2 * x + b2
    elif m2 is None:  # Second line is vertical
        x = b2
        y = m1 * x + b1
    else:
        if m1 == m2:  # Parallel lines
            return None
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1

    if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2) and \
       min(x3, x4) <= x <= max(x3, x4) and min(y3, y4) <= y <= max(y3, y4):
        return True
    return False

def head_irany(fok, lat, lon):              #headingnek megfelelő irányba történő elmozdulás
    fok = fok / 100
    if fok >= 0 and fok <= 90:
        lat += 2700 * math.cos(math.radians(fok))
        lon += 2700 * math.sin(math.radians(fok))
    elif fok > 90 and fok <= 180:
        lat += 2700 * math.cos(math.radians(fok))
        lon += 2700 * math.sin(math.radians(fok))
    elif fok > 180 and fok <= 270:
        lat += 2700 * math.cos(math.radians(fok))
        lon += 2700 * math.sin(math.radians(fok))
    elif fok > 270 and fok < 360:
        lat += 2700 * math.cos(math.radians(fok))
        lon += 2700 * math.sin(math.radians(fok))
    return lat, lon

def akt_poz(connection):                    #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                     0, 33, 0, 0, 0, 0, 0, 1)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

def is_point_in_polygon(point, polygon):    #Egy pont ellenőrzése hogy a poligonon belül taláható-e
    inside=False
    x, y = point
    n = len(polygon)

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def fence_download():                       #Fence letöltése a drónról 
    global connection, x, y, polygon
    connection.mav.mission_request_list_send(connection.target_system,
                                         connection.target_component,
                                         1)
    msg = connection.recv_match(type='MISSION_COUNT', blocking=True)
    
    for i in range(msg.count):
        connection.mav.mission_request_int_send(connection.target_system,
                                                connection.target_component,
                                                i, msg.mission_type)
        elem = connection.recv_match(type='MISSION_ITEM_INT', blocking=True)
        if elem is not None:
            x.append(elem.x)
            y.append(elem.y)
            polygon.append((elem.x, elem.y))
        else:
            print(f"Failed to receive mission item {i}")

connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

poz = akt_poz(connection)
polygon = []
x = []
y = []

fence_download()

if polygon:
    polygon.append(polygon[0])
    x.append(x[0])
    y.append(y[0])
else:
    print("Nincs kapott polygon pont")


while True:
    vizsgalt=False
    poz = akt_poz(connection)
    pont_hely = head_irany(poz.hdg, poz.lat, poz.lon)
    if is_point_in_polygon((poz.lat, poz.lon), polygon) or is_point_in_polygon(pont_hely, polygon):
        vizsgalt=True
    else:
        for i in range(len(polygon) - 1):
            if find_intersection(pont_hely[0], pont_hely[1], poz.lat, poz.lon, x[i], y[i], x[i+1], y[i+1]):
                vizsgalt=True
                break
    print(vizsgalt)
    time.sleep(0.1)
    os.system('clear')