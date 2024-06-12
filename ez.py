from pymavlink import mavutil
import math
import time
import matplotlib.pyplot as plt

def get_line_params(x1, y1, x2, y2):
    if x1 == x2:  # vertical line
        return None, x1
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def find_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
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
        return x, y
    else:
        return None

def head_irany(fok, lat, lon):    #headingnek megfelelő irányba történő elmozdulás
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

def akt_poz(connection):           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                     connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                     0, 33, 0, 0, 0, 0, 0, 1)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg

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

def plot_polygon_and_path(polygon, dron_pos, path_end, intersections):
    plt.figure()
    if polygon:
        plt.plot([p[0] for p in polygon] + [polygon[0][0]], [p[1] for p in polygon] + [polygon[0][1]], 'b-', label='Polygon')
    plt.plot([dron_pos[0], path_end[0]], [dron_pos[1], path_end[1]], 'r-', label='Drone Path')
    if intersections:
        plt.plot([i[0] for i in intersections], [i[1] for i in intersections], 'go', label='Intersections')
    plt.legend()
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Polygon and Drone Path')
    plt.grid(True)
    plt.show()

connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
poz = akt_poz(connection)

connection.mav.mission_request_list_send(connection.target_system,
                                         connection.target_component,
                                         1)
msg = connection.recv_match(type='MISSION_COUNT', blocking=True)

polygon = []
x = []
y = []

for i in range(msg.count):
    connection.mav.mission_request_int_send(connection.target_system,
                                             connection.target_component,
                                             i, msg.mission_type)
    elem = connection.recv_match(type='MISSION_ITEM_INT', blocking=True)
    if elem is not None:
        print(elem)
        x.append(elem.x)
        y.append(elem.y)
        polygon.append((elem.x, elem.y))
    else:
        print(f"Failed to receive mission item {i}")

if polygon:
    polygon.append(polygon[0])
    x.append(x[0])
    y.append(y[0])
else:
    print("Nincs kapott polygon pont")

print(x)
print(y)

szakasz_pol = []
for i in range(len(polygon) - 1):
    szakasz_pol.append(get_line_params(x[i], y[i], x[i+1], y[i+1]))
print(szakasz_pol)

pont_hely = head_irany(poz.hdg, poz.lat, poz.lon)
print(pont_hely)
szakasz_hely = get_line_params(pont_hely[0], pont_hely[1], poz.lat, poz.lon)
print(szakasz_hely)

intersections = []
for i in range(len(polygon) - 1):
    intersection = find_intersection(pont_hely[0], pont_hely[1], poz.lat, poz.lon, x[i], y[i], x[i+1], y[i+1])
    if intersection:
        intersections.append(intersection)

start_inside = is_point_in_polygon((poz.lat, poz.lon), polygon)
end_inside = is_point_in_polygon(pont_hely, polygon)

if intersection!=None or start_inside or end_inside:
    print("True")
else:
    print("False")


if polygon:
    plot_polygon_and_path(polygon, (poz.lat, poz.lon), pont_hely, intersections)
else:
    print("Nincs polygonon.")
