from pymavlink import mavutil
import math
import time

def is_point_in_polygon(point, polygon):
    """
    Determine if the point is inside the polygon using the Ray Casting algorithm.

    Parameters:
    point: tuple, a point represented as (x, y).
    polygon: list of tuples, the polygon represented as a list of (x, y) points.

    Returns:
    True if the point is inside the polygon, False otherwise.
    """
    x, y = point
    n = len(polygon)
    inside = False

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

def egyenlet(x1,y1,x2,y2):
    if x1==x2:
        m=None
        b=x1
        return m,b
    elif x1<x2:
        m=(y2-y1)/(x2-x1)
        b=y1-m*x1
        return m,b
    else:
        m=(y1-y2)/(x1-x2)
        b=y1-m*x1
        return m,b

def head_irany(fok):    #headingnek megfelelő irányba történő elmozdulás
    global poz
    lat=poz.lat
    lon=poz.lon
    #egyseg=1/(111320*math.cos(math.radians(poz.lat*math.pow(10,-7))))
    fok=fok/100
    if fok>=0 and fok<=90:
        lat+=270*math.cos(math.radians(fok))
        lon+=270*math.sin(math.radians(fok))
    elif fok>90 and fok<=180:
        lat-=270*math.cos(math.radians(fok))
        lon-=270*math.sin(math.radians(fok))
    elif fok>180 and fok<=270:
        lat+=270*math.cos(math.radians(fok))
        lon+=270*math.sin(math.radians(fok))
    elif fok>270 and fok<360:
        lat-=270*math.cos(math.radians(fok))
        lon-=270*math.sin(math.radians(fok))
    return lat, lon

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
poz=akt_poz()

connection.mav.mission_request_list_send(connection.target_system,
                                         connection.target_component,
                                         1)
msg=connection.recv_match(type='MISSION_COUNT', blocking=True)
#print(msg)
polygon=[]
x=[]
y=[]
for i in range(msg.count):
    connection.mav.mission_request_int_send(connection.target_system,
                                             connection.target_component,
                                             i,msg.mission_type)
    elem=connection.recv_match(type='MISSION_ITEM_INT', blocking=True)
    print(elem)
    x.append(elem.x)
    y.append(elem.y)
    polygon.append((elem.x,elem.y))

x.append(x[0])
y.append(y[0])
print(x)
print(y)
szakasz_pol=[]
for i in range(0,msg.count):
    szakasz_pol.append(egyenlet(x[i],y[i],x[i+1],y[i+1]))
print(szakasz_pol)

pont_hely=head_irany(poz.hdg)
print(pont_hely)
szakasz_hely=egyenlet(pont_hely[0],pont_hely[1],poz.lat,poz.lon)
print(szakasz_hely)

while True:
    belul=False
    poz=akt_poz()
    point=(poz.lat,poz.lon)
    if not is_point_in_polygon(point,polygon):
        for i in range(30):
            head_irany(poz.hdg)
            point=(poz.lat,poz.lon)
            if is_point_in_polygon(point,polygon):
                belul=True
    else:
        belul=True
    print(belul)
    time.sleep(0.1)