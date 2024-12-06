from pymavlink import mavutil

# Start a connection listening on a UDP port
# connected_drone = mavutil.mavlink_connection('udpin:localhost:14540')
connected_drone = mavutil.mavlink_connection('COM5', 9600)

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
connected_drone.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connected_drone.target_system, connected_drone.target_component))

# Once connected, use 'the_connection' to get and send messages

def get_messages():
    while True:
        msg = connected_drone.recv_match(blocking=True)
        # print(msg, "\n")

        if msg.get_type() == "SYS_STATUS":
            # Process SYS_STATUS message
            sys_status = msg
            print(sys_status)

def get_sensor_heath():
    sys_status = connected_drone.recv_match(type="SYS_STATUS", blocking=True)
    print(sys_status)

def mag_cal():
    global ack_msg
    try:
        connected_drone.mav.command_long_send(connected_drone.target_system, connected_drone.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack_msg = connected_drone.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.result == 0:
            print("Armed successfully!")
        else:
            print(f"Arming failed: {ack_msg.result}")
    except Exception as e:
        print(f"Error sending arm command: {e}")
    finally:
        print(ack_msg)


get_messages()