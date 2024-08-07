from pymavlink import mavutil

def akt_poz():              #Jelenlegi pozícióba
    connection.mav.command_long_send(connection.target_system,
                                 connection.target_component, 
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 
                                 0, 65, 0,0,0,0,0,1)
    msg=connection.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.1)
    return msg

#-----Main-----
connection=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

msg=akt_poz()
print(msg.rssi)