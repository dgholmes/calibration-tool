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

def get_status(val, sensor_bitmask):
    if val & sensor_bitmask != 0:
        return "Healthy"
    else:
        return "Unhealthy"

def get_sensor_heath():
    MAV_SYS_STATUS_SENSOR_3D_GYRO = 0x01
    MAV_SYS_STATUS_SENSOR_3D_ACCEL = 0x02
    MAV_SYS_STATUS_SENSOR_3D_MAG = 0x04
    MAV_SYS_STATUS_SENSOR_GPS = 0x20
    MAV_SYS_STATUS_SENSOR_BATTERY = 0x2000000

    while True:
        sys_status = connected_drone.recv_match(type="SYS_STATUS", blocking=True)
        # print(sys_status)
        sensors_health = sys_status.onboard_control_sensors_health
        print(bin(sys_status.onboard_control_sensors_health))
        print(f"3D_GYRO is {get_status(sensors_health, MAV_SYS_STATUS_SENSOR_3D_GYRO)}")
        print(f"3D_ACCEL is {get_status(sensors_health, MAV_SYS_STATUS_SENSOR_3D_ACCEL)}")
        print(f"3D_MAG is {get_status(sensors_health, MAV_SYS_STATUS_SENSOR_3D_MAG)}")
        print(f"GPS is {get_status(sensors_health, MAV_SYS_STATUS_SENSOR_GPS)}")
        print(f"BATTERY is {get_status(sensors_health, MAV_SYS_STATUS_SENSOR_BATTERY)}")

def get_sensor_status():
    MAV_SYS_STATUS_SENSOR = {
        0: "3D gyro",
        1: "3D accelerometer",
        2: "3D magnetometer",
        3: "Absolute pressure",
        4: "Differential pressure",
        5: "GPS",
        6: "Optical flow",
        7: "Computer vision position",
        8: "Laser-based position",
        9: "External ground truth",
        10: "Angular rate control",
        11: "Attitude stabilization",
        12: "Yaw position control",
        13: "Altitude control",
        14: "XY position control",
        15: "Motor outputs/control",
        16: "RC receiver",
        17: "2nd 3D gyro",
        18: "2nd 3D accelerometer",
        19: "2nd 3D magnetometer",
        20: "Geofence",
        21: "AHRS subsystem health",
        22: "Terrain subsystem health",
        23: "Motors are reversed",
        24: "Logging",
        25: "Battery",
        26: "Proximity",
        27: "Satellite Communication",
        28: "Pre-arm check status",
        29: "Avoidance/collision prevention",
        30: "Propulsion",
        31: "Extended bit-field"
    }

    while True:
        sys_status = connected_drone.recv_match(type="SYS_STATUS", blocking=True)
        # print(sys_status)
        sensors_status = sys_status.onboard_control_sensors_health

        # Decode
        for bit, description in MAV_SYS_STATUS_SENSOR.items():
            status = "Healthy" if (sensors_status & (1 << bit)) else "Unhealthy"
            print(f"{description}: {status}")
        print('\n')



def mag_cal():
    global ack_msg
    try:
        connected_drone.mav.command_long_send(connected_drone.target_system, connected_drone.target_component,
                                     mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION , 0,
                                              0, 0, 0, 0, 2, 0, 0)
        ack_msg = connected_drone.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.result == 0:
            print("Armed successfully!")
        else:
            print(f"Arming failed: {ack_msg.result}")
    except Exception as e:
        print(f"Error sending arm command: {e}")
    finally:
        print(ack_msg)


mag_cal()