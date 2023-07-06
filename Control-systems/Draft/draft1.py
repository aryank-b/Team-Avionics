#test file 1 for control code
#### DEPENDENCIES #####
from dronekit import connect,VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
from pymavlink import mavutil
import cv2
from pyzbar.pyzbar import decode, ZBarSymbol
import numpy as np
import sys

##### FUNCTIONS #####

def qr_read():
    delay = 1
    window_name = 'frame'

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        sys.exit()

    while True:
        ret, frame = cap.read()
        flag = False
        #cv2.imshow(window_name, frame)
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (thresh, blackAndWhiteFrame) = cv2.threshold(grayFrame, 127, 255, cv2.THRESH_BINARY)
        if ret:
            for d in decode(blackAndWhiteFrame, symbols=[ZBarSymbol.QRCODE]):
                s = d.data.decode()
                k,n, flag = get_coords(s)
                blackAndWhiteFrame = cv2.rectangle(blackAndWhiteFrame, (d.rect.left, d.rect.top),
                                    (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (0, 255, 0), 3)
                blackAndWhiteFrame = cv2.putText(blackAndWhiteFrame, s, (d.rect.left, d.rect.top + d.rect.height),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
                cx = int(d.rect.left + d.rect.width/2)
                cy = int(d.rect.top + d.rect.height/2)
                blackAndWhiteFrame = cv2.circle(blackAndWhiteFrame, (cx, cy), 5, (25, 25, 25), 3)
                print(cx, cy)
            cv2.imshow("bw frame", blackAndWhiteFrame)

            for d in decode(frame, symbols=[ZBarSymbol.QRCODE]):
                s = d.data.decode()
                k,n, flag = get_coords(s)
                frame = cv2.rectangle(frame, (d.rect.left, d.rect.top),
                                    (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (0, 255, 0), 3)
                frame = cv2.putText(frame, s, (d.rect.left, d.rect.top + d.rect.height),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
                cx = int(d.rect.left + d.rect.width/2)
                cy = int(d.rect.top + d.rect.height/2)
                frame = cv2.circle(frame, (cx, cy), 5, (25, 25, 25), 3)
                print(cx, cy)
            cv2.imshow('frame', frame)

        if (cv2.waitKey(delay) & 0xFF == ord('q')) or flag:
            break

    cv2.destroyWindow('frame')
    cv2.destroyWindow('bw frame')
    return k,n

def get_coords(s):
    coords = []
    if(s!=''):
        x = s.split(",")
        n = np.size(x)
        if(n==4):
            coords.append(float(s.split(",")[0][2:]))
            coords.append(float(s.split(",")[1][:-1]))
            coords.append(float(s.split(",")[2][1:]))
            coords.append(float(s.split(",")[3][:-2]))
            coords = np.array(coords)
        elif(n==2):
            coords.append(float(s.split(",")[0][1:]))
            coords.append(float(s.split(",")[1][:-1]))
            coords = np.array(coords)
        return coords,n, True
    else:
        return coords,n, False
def get_distance_accurate(loc1, loc2):
    """Get ground distance between two locations."""
    dist1 = loc1.lat - loc2[0]
    dist1 = 111111.1 * dist1 
    dist2 = loc1.lon - loc2[1]
    dist2 = 111111.1 * dist2 
    return math.sqrt(dist1**2 + dist2**2)
    return mp_util.gps_distance(loc1.lat, loc1.lon, loc2[0], loc2[1])
    return math.sqrt(math.pow(mp_util.gps_distance(loc1.lat, loc1.lon, loc2[0], loc2[1]),2)+math.pow(loc1.alt-loc2[2],2))

def connectMyCopter() :

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    if not connection_string:
        connection_string = 'tcp:192.168.56.1:5762' 
    # if not connection_string:
    #     import dronekit_sitl
    #     sitl = dronekit_sitl.start_default()
    #     connection_string = sitl.connection_string()

    try:
        print("\nConnecting to vehicle on: %s" % connection_string)
        vehicle = connect(connection_string, wait_ready=True)
    except socket.error:
        print ('No server exists!')

    # Bad TTY connection
    except OSError as e:
        print ('No serial exists!')

    # API Error
    except APIException:
        print ('Timeout!')

    # Other error
    except:
        print ('Some other error!')

    return vehicle
##>> python file.py --connect 127.0.0.1:14550
##>> python file.py

def get_all_attributes():
    # Get all vehicle attributes (state)
    print("\nGet all vehicle attribute values:")
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print("   Major version number: %s" % vehicle.version.major)
    print("   Minor version number: %s" % vehicle.version.minor)
    print("   Patch version number: %s" % vehicle.version.patch)
    print("   Release type: %s" % vehicle.version.release_type())
    print("   Release version: %s" % vehicle.version.release_version())
    print("   Stable release?: %s" % vehicle.version.is_stable())
    print(" Autopilot capabilities")
    print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
    print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
    print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
    print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
    print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
    print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
    print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
    print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
    print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
    print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
    print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" Battery: %s" % vehicle.battery)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print(" Heading: %s" % vehicle.heading)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    print(" Airspeed: %s" % vehicle.airspeed)    # settable
    print(" Mode: %s" % vehicle.mode.name)    # settable
    print(" Armed: %s" % vehicle.armed)    # settable
    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print(" Waiting for home location ...")
    # We have a home location, so print it!        
    print("\n Home location: %s" % vehicle.home_location)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    vehicle.wait_for_armable()
    print("used wait_for_armable")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    if vehicle.armed:
        print("DISARMING")
        while not vehicle.mode=="STABILIZE":
            print(" Waiting for mode change...")
            print(" Mode: %s" % vehicle.mode.name)    # settable
            vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(1)
        while vehicle.armed:
            print(" Waiting for disarming...")
            #rtl_land()
            print(" Mode: %s" % vehicle.mode.name)    # settable
            time.sleep(1)
            vehicle.armed =False
        time.sleep(5)

    print("Arming motors")
    vehicle.wait_for_mode("GUIDED")
    print("used wait_for_mode")
    # Copter should arm in GUIDED mode
    while not vehicle.mode=="GUIDED":
        print(" Waiting for mode change...")
        print(" Mode: %s" % vehicle.mode.name)    # settable
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
    vehicle.armed = True


    
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        print(" Mode: %s" % vehicle.mode.name)    # settable
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_altitude(aTargetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def wait_simple_goto(lat =-35.361354,lon = 149.165218,alt= 7,epsilon=1):
    print("Going towards first point for 30 seconds ...")
    point = LocationGlobalRelative(lat,lon,alt)
    vehicle.simple_goto(point)
    # Assuming you have a loop running to continuously monitor the drone's position
    while True:
        current_location = vehicle.location.global_frame

        # Calculate the distance between the current location and target location
        distance = get_distance_accurate(current_location,(lat,lon,alt))

        # Set a threshold distance, below which we consider the drone has reached the location
        threshold_distance = epsilon  # Adjust this value as per your requirement

        # Check if the drone has reached the target location
        if distance < threshold_distance:
            print("Drone has reached the target location.")
            break

        # Optionally, you can print the current distance for debugging purposes
        print("Distance to target: ", distance)
        time.sleep(1)

    # sleep so we can see the change in map
#Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(velocity_x, velocity_y, velocity_z,duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions 
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity
        0, 0, 0,  # x, y, z acceleration 
        0, 0)  # yaw, yaw_rate
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#Send a velocity command with +x being true NORTH of Earth.
def send_local_ned_velocity(velocity_x, velocity_y, velocity_z,duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions 
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity
        0, 0, 0,  # x, y, z acceleration 
        0, 0)  # yaw, yaw_rate
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        print("time moved : ",x+1)
    
#Send a velocity command with +x being true NORTH of Earth.
def send_global_ned_velocity(velocity_x, velocity_y, velocity_z,duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions 
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity
        0, 0, 0,  # x, y, z acceleration 
        0, 0)  # yaw, yaw_rate
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
        print("time moved : ",x+1)

def rtl_land():
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    # # Get the home location of the drone
    # home_location = vehicle.home_location

    # # Set the target location to the home location
    # target_location = LocationGlobalRelative(home_location.lat, home_location.lon, home_location.alt)

    # time.sleep(30)
    print("waiting to land")
    vehicle.wait_for_alt(0.01)
    time.sleep(1)

def land_close():
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    # # Get the home location of the drone
    # home_location = vehicle.home_location

    # # Set the target location to the home location
    # target_location = LocationGlobalRelative(home_location.lat, home_location.lon, home_location.alt)

    # time.sleep(30)
    print("waiting to land")
    while vehicle.armed:
        time.sleep(1)
    #Close vehicle object before exiting script
    print("\nClose vehicle object")
    vehicle.close()

    print("Completed")

### STAGES SCRIPT ###

def stage_1(alt):
    print("Stage-1 Started")
    wait_simple_goto(19.133652, 72.913380)
    pt = [0,0,0]
    pt[0] = (19.133934, 72.913124)
    pt[1] = (19.134114, 72.913514)
    pt[2] = (19.134257, 72.913099)
    print("waypoint reached:",1)
    for i in range(3):
        # obtain point from qr function using the following
        # search qr by bounding box 
        # move towards it 
        # lower altitude if needed 
        # scan
        pt, n = qr_read()
        if(n==2):
            wait_simple_goto(pt[0],pt[1])
        # wait_simple_goto(pt[i][0],pt[i][1],alt)
        print("waypoint reached:", i+2)
    ##get direction from the next qr code
    ## process to acheive Vx Vy Vz
    send_global_ned_velocity(1,-3,0,10)

##### SCRIPT #####
# connecting
vehicle = connectMyCopter()
#time.sleep(5)
# check vehicle state
vehicle.wait_ready('autopilot_version')
get_all_attributes()

#check pre-arm then arm and takeoff to 10m
arm_and_takeoff(7)

#changing default airspeed for goto
print("Set default/target airspeed to 3")
vehicle.airspeed = 3
alt_s1 = 2
stage_1(alt_s1)

land_close()
