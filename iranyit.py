from pymavlink import mavutil
from pynput import keyboard
import math

def mozgas():           #Drón mozgatása x,y,z változónak megfelelően
    global x,y,z
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,connection.target_system,
                                                                                      connection.target_component, 
                                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                                                                                      int(0b110111111000), x, y, z, 10, 10, 5, 0, 0, 0, 0, 0))
    yaw(-1)

def akt_poz():           #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 32, 0,0,0,0,0,1)
    msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg

def hatar_szog(szog):   #Heading szögének határolása
    if szog>=360:
        szog-=360
    if szog<0:
        szog+=360
    return szog

def head_irany(fok):    #headingnek megfelelő irányba történő elmozdulás
    global x,y,z
    msg=akt_poz()
    z=msg.z
    fok=hatar_szog(fok)
    fok=round(math.radians(fok),2)
    y+=0.1*math.sin(fok)
    x+=0.1*math.cos(fok)
    mozgas()
    
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
                                 0, 0,0,0,0,0,0, 10)
    msg=connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    yaw(0)


def yaw(irany):         #Yaw elfordítás az adott iránynak megfelelően
    global angle
    angle=hatar_szog(angle)
    connection.mav.command_long_send(connection.target_system, 
                                         connection.target_component, 
                                         mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
                                         0, angle,100,irany,0,0,0,0)

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
    global z
    z=0.0

def stop():             #Megállás jelenlegi pozícióba
    global x,y,z
    msg=akt_poz()
    print(msg)

    x=msg.x
    y=msg.y
    z=msg.z
    mozgas()

def on_press(key):      #Gomb lenyomások kezelése
    global x,y,z,angle
    print(x)
    print(y)
    try:
        current_keys.add(key.char)
        if {'w', 'a'} == current_keys:      #Előre és balra elmozdulás: w+a
            head_irany(angle-45)
        
        elif {'w', 'd'} == current_keys:    #Előre és jobbra elmozdulás: w+d
            head_irany(angle+45)

        elif {'s', 'a'} == current_keys:    #Hátra és balra elmozdulás: s+a
            head_irany(angle+225)
        
        elif {'s', 'd'} == current_keys:    #Hátra és jobbra elmozdulás: s+d
            head_irany(angle-225)
        
        elif key.char=='w':    #Mozgás előre W nyomásra
            head_irany(angle)
        
        elif key.char=='s':    #Mozgás hátra S nyomásra
            head_irany(angle-180)

        elif key.char=='d':    #Mozgás jobbra D nyomásra
            head_irany(angle+90)

        elif key.char=='a':    #Mozgás balra A nyomásra
            head_irany(angle-90)
        
        elif key.char=='r':    #Mozgás fel R nyomásra
            z-=0.1
            mozgas()

        elif key.char=='f':    #Mozgás le F nyomásra
            z+=0.1
            mozgas()
        
        elif key.char=='e':    #Yaw jobbra E nyomásra
            angle+=1
            yaw(1)

        elif key.char=='q':    #Yaw balra Q nyomásra
            angle-=1
            yaw(-1)
        
        elif key.char=='k':    #Felszállás 1 nyomásra
            felszall()

        elif key.char=='l':    #Leszállás 0 nyomásra
            leszall()
        elif key.char=='c':    #Cél koordináta megadása
            x=float(input("X: "))
            y=float(input("Y: "))
            z=float(input("Z: "))
            z*=(-1)
            mozgas()
        else:
            print("No such a key")
    except AttributeError:
        print()

def on_release(key):    #Gomb felengedések kezelése
    global x,y,z
    try:
        current_keys.remove(key.char)
        if key.char=='w':       #Bármely vezérlő gomb felengedésénél megállás az aktuális pozícióba
            stop()
            
        if key.char=='s':
            stop()

        if key.char=='d':
            stop()

        if key.char=='a':
            stop()
            
        if key.char=='r':
            stop()

        if key.char=='f':
            stop()
    except AttributeError:
        print()
    if key == keyboard.Key.esc:     # Stop listener
                return False

x=0.0
y=0.0
z=0.0
angle=0
connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
print("Waiting for position...")
stop()
print("Felszállás k gomb lenyomásával!")
print("Leszállás l gomb lenyomásával!")
print("Koordináta megadása c gomb lenyomásával!")
current_keys = set()
if z<0:
    yaw(-1)

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()