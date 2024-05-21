from pymavlink import mavutil
from pynput import keyboard
import math

def mozgas():
    global x,y,z
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,connection.target_system,
                                                                                      connection.target_component, 
                                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                                                                                      int(0b110111111000), x, y, z, 10, 10, 0, 0, 0, 0, 0, 0))

def hatar_szog(szog):
    if szog>=360:
        szog-=360
    if szog<0:
        szog+=360
    return szog

def head_irany(fok):        #headingnek megfelelő irányba történő elmozdulás
    global x,y
    fok=hatar_szog(fok)
    fok=round(math.radians(fok),2)
    if math.sin(fok)!=0 and math.cos(fok)!=0:
        if fok<math.pi and fok>math.pi/2:
            x-=0.1/math.sin(fok)
            y-=0.1/math.cos(fok)
        elif fok<2*math.pi and fok>math.pi*1.5:
            x-=0.1/math.sin(fok)
            y-=0.1/math.cos(fok)
        else:
            x+=0.1/math.sin(fok)
            y+=0.1/math.cos(fok)
    elif math.sin(fok)==0:
        x+=0.1
    else:
        y+=0.1
    mozgas()
    
def felszall():
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
    global z
    z=-10.0

def yaw(irany):                                                                        #Yaw elfordítás és annak iránya
    global angle
    angle=hatar_szog(angle)
    connection.mav.command_long_send(connection.target_system, 
                                         connection.target_component, 
                                         mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
                                         0, angle,100,irany,0,0,0,0)

def leszall():
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

def stop():                 #Megállás jelenlegi pozícióba
    global x,y,z
    msg=None
    connection.mav.request_data_stream_send(connection.target_system, connection.target_component, 
                                        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 
                                        100, 1)
    while msg==None:
        msg=connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    connection.mav.request_data_stream_send(connection.target_system, connection.target_component, 
                                            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 
                                            100, 0)
    x=msg.x
    y=msg.y
    z=msg.z
    mozgas()
    yaw(0)

def on_press(key):
    global x,y,z,angle
    print(x)
    print(y)
    try:
        if key.char=='w':    #Mozgás előre W nyomásra
            head_irany(angle)
        
        if key.char=='s':    #Mozgás hátra S nyomásra
            head_irany(angle-180)

        if key.char=='d':    #Mozgás jobbra D nyomásra
            head_irany(angle+90)

        if key.char=='a':    #Mozgás balra A nyomásra
            head_irany(angle-90)
        
        if key.char=='r':    #Mozgás fel R nyomásra
            z-=0.1
            mozgas()

        if key.char=='f':    #Mozgás le F nyomásra
            z+=0.1
            mozgas()
        
        if key.char=='e':    #Yaw jobbra E nyomásra
            angle+=1
            yaw(1)

        if key.char=='q':    #Yaw balra Q nyomásra
            angle-=1
            yaw(-1)
        
        if key.char=='1':    #Felszállás 1 nyomásra
            felszall()

        if key.char=='0':    #Leszállás 0 nyomásra
            leszall()
    except:
        print("No such a key")

def on_release(key):
    global x,y,z
    if key.char=='w':
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
    if key == keyboard.Key.esc:     # Stop listener
            return False




connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

print("Felszállás 1-es gomb lenyomásával!")
print("Leszállás 0-ás gomb lenyomásával!")
x=0.0
y=0.0
z=-10.0
angle=0

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