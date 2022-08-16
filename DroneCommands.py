from pymavlink import mavutil
import numpy as np
import matplotlib as plt
import sys
import time





def arm(target):
    manualControl(target,0,0,0,0)
    target.mav.command_long_send(target.target_system,target.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0,
                                 1, 0, 0, 0, 0, 0, 0)
    #target.motors_armed_wait()

    
    
def disarm(target):
    target.mav.command_long_send(target.target_system,target.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0,
                                 0, 0, 0, 0, 0, 0, 0)
    #target.motors_disarmed_wait()


def isArmed(target):
    target.recv_match(type='HEARTBEAT', blocking=False)
    return bool(target.motors_armed())

def takeoff(target,alt):
    target.mav.command_long_send(target.target_system,target.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 0, 0, 0, 0, 0, 0, 0, alt)
    msg = target.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)

def land(target):
    target.mav.command_long_send(target.target_system,target.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND,
                                 0, 0, 0, 0, 0, 0, 0, 0)
    msg = target.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)


def manualControl(target,thrust,roll,pitch,yaw):
    #                        up,left,front,clockwise
    target.mav.manual_control_send(target.target_system,
                                   -pitch,-roll,thrust,-yaw,0)


def setMode(target,mode):
    if mode not in target.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(target.mode_mapping().keys()))
        sys.exit(1)
    mode_id = target.mode_mapping()[mode]
    target.mav.set_mode_send(
    target.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
   
