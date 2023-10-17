#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
connection_string = 'udp:127.0.0.1:14550'
print('Connecting to vehicle on: %s' % connection_string)
print('connection string: ' + connection_string)
vehicle = connect(connection_string, wait_ready=False)

velocity_x = 5
velocity_y = 0
velocity_z = 0


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
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


arm_and_takeoff(10)

print ("sending geofence messages")

# mission count
msg = vehicle.message_factory.mission_count_encode(
    0, 0,  # target_system, target_component
    # mission type
    mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
    1)  # count
vehicle.send_mavlink(msg)
print(vehicle.home_location)
# send fence point
msg = vehicle.message_factory.mission_item_int_encode(
    0, 0,  # target_system, target_component
    0,  # seq
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
    mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,  # command
    # current
    1,
    # autocontinue
    1,
    # param1 / radius
    100,
    # param2
    0,
    # param3
    0,
    # param4
    0,
    # x
    # int(-35.36277334 * 10000000.0),
    # int(-35.36277344 * 10000000.0),
    int(-35.36324506 * 10000000.0),


    # vehicle.home_location.lat,
    # y
    int(149.16523168 * 10000000.0),
    # int(149.16536671 * 10000000.0),
    # vehicle.home_location.lon,
    # z
    0,
    # mission_type
    mavutil.mavlink.MAV_MISSION_TYPE_FENCE
)

# msg = vehicle.message_factory.command_long_encode(
#     0, 0,    # target_system, target_component
#     mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
#     1, #confirmation
#     0,    # param 1, yaw in degrees
#     0,          # param 2, yaw speed deg/s
#     1,          # param 3, direction -1 ccw, 1 cw
#     0, # param 4, relative offset 1, absolute angle 0
#     0, 0, 0)    # param 5 ~ 7 not used
# # send command to vehicle
vehicle.send_mavlink(msg)

print("message sent")
# send command to vehicle
vehicle.send_mavlink(msg)
time.sleep(30)