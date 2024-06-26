from pymavlink import mavutil
import math
import matplotlib.pyplot as plt
import time
import os

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
    else:
        m=(y2-y1)/(x2-x1)
        b=y1-m*x1
        return m,b

def sorrend(a,b):
    if a<b:
        return a,b
    else:
        return b,a

def head_irany(fok):    #headingnek megfelelő irányba történő elmozdulás
    global poz
    lat=poz.lat
    lon=poz.lon
    #egyseg=1/(111320*math.cos(math.radians(poz.lat*math.pow(10,-7))))
    fok=fok/100
    if fok>=0 and fok<=90:
        lat+=2700*math.cos(math.radians(fok))
        lon+=2700*math.sin(math.radians(fok))
    elif fok>90 and fok<=180:
        lat+=2700*math.cos(math.radians(fok))
        lon+=2700*math.sin(math.radians(fok))
    elif fok>180 and fok<=270:
        lat+=2700*math.cos(math.radians(fok))
        lon+=2700*math.sin(math.radians(fok))
    elif fok>270 and fok<360:
        lat+=2700*math.cos(math.radians(fok))
        lon+=2700*math.sin(math.radians(fok))
    return lat, lon

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 33, 0,0,0,0,0,1)
    msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

def dron_szakaszon_van(x_m,y_m):
    global poz, pont_hely
    sorba=sorrend(poz.lat, pont_hely[0])
    if sorba[0]<=x_m and sorba[1]>=x_m:
        sorba=sorrend(poz.lon,pont_hely[1])
        if sorba[0]<=y_m and sorba[1]>=y_m:
            return True
    return False

def fence_szakaszon_van(i):
    global x,y,x_m,y_m,benne_van
    sorba=sorrend(x[i], x[i+1])
    if sorba[0]<=x_m and sorba[1]>=x_m:
        sorba=sorrend(y[i],y[i+1])
        if sorba[0]<=y_m and sorba[1]>=y_m:
            benne_van=True

def ellenoriz():
    global szakasz_pol, szakasz_hely, x, poz, benne_van, pont_hely

    for i in range(len(szakasz_pol)):
        if szakasz_pol[i][0]==None:
            sorba=sorrend(poz.lat,pont_hely[0])
            if sorba[0]<=x[i] and sorba[1]>=x[i]:
                benne_van=True
        elif szakasz_hely[0]==None:
            sorba=sorrend(x[i],x[i+1])
            if sorba[0]<=poz.lat and sorba[1]>=poz.lat:
                benne_van=True
        else:
            x_m=(szakasz_pol[i][1]-szakasz_hely[1])/(szakasz_pol[i][0]-szakasz_hely[0])
            y_m=szakasz_hely[0]*x_m+szakasz_hely[1]
            sorba=sorrend(poz.lat, pont_hely[0])
            plt.plot(x_m,y_m,'o')
            if dron_szakaszon_van(x_m,y_m):
                fence_szakaszon_van(i)

def fence_download():
    global connection, x, y, polygon
    connection.mav.mission_request_list_send(connection.target_system,
                                         connection.target_component,
                                         1)
    msg=connection.recv_match(type='MISSION_COUNT', blocking=True)

    for i in range(msg.count):
        connection.mav.mission_request_int_send(connection.target_system,
                                                connection.target_component,
                                                i,msg.mission_type)
        elem=connection.recv_match(type='MISSION_ITEM_INT', blocking=True)
        x.append(elem.x)
        y.append(elem.y)
        polygon.append((elem.x,elem.y))

def fence_szakaszok():
    global x, y, szakasz_pol
    x.append(x[0])
    y.append(y[0])
    for i in range(0,len(x)-1):
        szakasz_pol.append(egyenlet(x[i],y[i],x[i+1],y[i+1]))

def dron_szakasz():
    global pont_hely, szakasz_hely
    pont_hely=head_irany(poz.hdg)
    szakasz_hely=egyenlet(pont_hely[0],pont_hely[1],poz.lat,poz.lon)

connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

polygon=[]
x=[]
y=[]
fence_download()


szakasz_pol=[]
szakasz_hely=[]
pont_hely=[]
poz=akt_poz()
fence_szakaszok()
dron_szakasz()
benne_van=False
if is_point_in_polygon((poz.lat,poz.lon),polygon):
    benne_van=True
ellenoriz()
print(benne_van)

plt.plot(x,y)
#plt.plot(poz.lat,poz.lon,'or')
#plt.plot(pont_hely[0],pont_hely[1],'og')
plt.show()