import asyncio
import xml.etree.ElementTree as ET
import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import qtm
import time
import sys
from pymavlink import mavutil
from datetime import datetime

from Controller import *
from DroneCommands import *
from ReadTSV import *
from QualisysRT import *
from DroneObject import *



print('Test started!')

#
#t = np.linspace(-2,15,100)
#y = np.zeros(len(t))
#
#for i in range(len(t)):
#    y[i] = ramp(100,10,t[i])
#
#plt.plot(t,y)
#plt.show()

print((-350)%360)
# print(datetime.today())
# x = np.array([1.0,2.0,np.nan,1.0,3.0,np.nan,4.0])
# x = x[~np.isnan(x)]
# print(x)

# THETESTDRONE = drone('TESTDRONE',14550,"QTM\Qualisys\Data\Trajectory3_6D.tsv")
# #
# tstart= time.perf_counter()
# t = tstart
# ##
# ##setMode(THETESTDRONE.MAVLinkConnection,'STABILIZE')
# ##arm(THETESTDRONE.MAVLinkConnection)
# dt = 0
# while(t < 7):      
#    tic = time.perf_counter()
#    t = tic - tstart
#    
#    print('\n')
#    if t < 1:
#                                #        X,    Y,    Z,    R,    P,    Y    
#        THETESTDRONE.MeasPosition = [    0,    0, 1000,    0,    0,    0]
#        THETESTDRONE.RefPosition  = [    0,    0, 2000,    0,    0,    0]
#        print('HAUT')    
#    elif t < 2:
#                                #        X,    Y,    Z,    R,    P,    Y    
#        THETESTDRONE.MeasPosition = [    0,    0, 1000,    0,    0,    180]
#        THETESTDRONE.RefPosition  = [    0, 1000, 1000,    0,    0,    180]
#        print('GAUCHE')
#    elif t < 3:
#                                #        X,    Y,    Z,    R,    P,    Y  
#        THETESTDRONE.MeasPosition = [    0,    0, 1000,    0,    0,    180]
#        THETESTDRONE.RefPosition  = [-1000,    0, 1000,    0,    0,    180]
#        print('ARRIERE')
#    elif t < 4:
#                                #        X,    Y,    Z,    R,    P,    Y  
#        THETESTDRONE.MeasPosition = [    0,    0, 1000,    0,    0,    180]
#        THETESTDRONE.RefPosition  = [    0,-1000, 1000,    0,    0,    180]
#        print('DROITE')
#    elif t < 5:
#                                #        X,    Y,    Z,    R,    P,    Y   
#        THETESTDRONE.MeasPosition = [    0,    0, 1000,    0,    0,    180]
#        THETESTDRONE.RefPosition  = [ 1000,    0, 1000,    0,    0,    180]
#        print('AVANT')
#    elif t < 6:
#                                #        X,    Y,    Z,    R,    P,    Y   
#        THETESTDRONE.MeasPosition = [    0,    0,    0,    0,    0,   90]
#        THETESTDRONE.RefPosition  = [ 1000,    0,    0,    0,    0,   269]
#        print('HORLOGER')
#    elif t < 7:
#                                #        X,    Y,    Z,    R,    P,    Y   
#        THETESTDRONE.MeasPosition = [    0,    0,    0,    0,    0,   90]
#        THETESTDRONE.RefPosition  = [ 1000,    0,    0,    0,    0,  271]
#        print('ANTI-HORLOGER')
#    
#    [T,R,P,Y,EE] = DroneController(THETESTDRONE,0.1)
#    #manualControl(THETESTDRONE.MAVLinkConnection,T,R,P,Y)
#    
#    toc = time.perf_counter()
#    dt = toc - tic
#    print('Current time:',t,'s')
#    print('Time to execute step:',dt,'s')
#    time.sleep(.5-dt)
#


# drone = mavutil.mavlink_connection('udpin:localhost:14550')
# 
# drone.wait_heartbeat()
# print("Hearbeat from system (system %u component %u)" % (drone.target_system,drone.target_component))
# 
# time.sleep(3)
# 
# setMode(drone,'GUIDED')
# 
# arm(drone)
# drone.motors_armed_wait()
# 
# takeoff(drone,10)
# time.sleep(9)
# setMode(drone,'LOITER')
# 
# for i in range(0,10):      #haut    gauche  arriere horloger
#     manualControl(drone,    0,      000,     -000,    1000)
#     print(i)
#     time.sleep(.1)
# time.sleep(5)
# disarm(drone)


#print(isArmed(drone))
#for i in range(0,30): #droite
#    manualControl(drone,500,-1000,    0,0)
#    time.sleep(.1)
#time.sleep(3)     
#for i in range(0,30): #arriere
#    manualControl(drone,500,    0,+1000,0)
#    time.sleep(.1)
#time.sleep(3)     
#for i in range(0,30): #gauche
#    manualControl(drone,500,+1000,    0,0)
#    time.sleep(.1)
#time.sleep(3)
#print(isArmed(drone))

#land(drone)
#time.sleep(5)
#disarm(drone)
#time.sleep(5)
#print(isArmed(drone))

#while True:
#    msg = drone.recv_match(blocking=True)
#    #msg = drone.recv_match(type='NAV_CONTROLLER_OUTPUT',blocking=True)
#    #msg = drone.recv_match(type='ATTITUDE',blocking=True)
#    print(msg)
    
    













#async def main():
#    
#    
#    map = plt.figure()
#    map_ax = Axes3D(map)
#    map_ax.autoscale(enable=True, axis='both', tight=True)
#
#    # # # Setting the axes properties
#    maxRange = 5000
#    map_ax.set_xlim3d([-maxRange, maxRange])
#    map_ax.set_ylim3d([-maxRange, maxRange])
#    map_ax.set_zlim3d([0.0, 2*maxRange])
#
#    hl, = map_ax.plot3D([], [], [],'red')
#    update_line(hl, ([],[],[]))
#
#
#    # Connect to qtm
#    connection = await qtm.connect("127.0.0.1")
#
#    # Connection failed?
#    if connection is None:
#        print("Failed to connect")
#        return
#
#    # Take control of qtm, context manager will automatically release control after scope end
#    async with qtm.TakeControl(connection, ""):
#
#        realtime = False
#
#        if realtime:
#            # Start new realtime
#            await connection.new()
#        else:
#            # Load qtm file
#            await connection.load(QTM_FILE)
#
#            # start rtfromfile
#            await connection.start(rtfromfile=True)
#
#    # Get 6dof settings from qtm
#    xml_string = await connection.get_parameters(parameters=["6d"])
#    body_index = create_body_index(xml_string)
#               
#
#    tstart = time.perf_counter()
#    while(time.perf_counter()-tstart < 20):
#        await asyncio.sleep(0.1)
#        packet = await connection.get_current_frame(components = ["6d"])
#        position,rotation = Get6dof(packet,"Drone",body_index)
#        update_line(hl, (position.x,position.y, position.z))

    # Wait asynchronously 5 seconds

#    
#    
#if __name__ == "__main__":
#    # Run our asynchronous function until complete
#    asyncio.get_event_loop().run_until_complete(main())
#










#
#Trajectory = ReadTrajectory("QTM\Qualisys\Data\Trajectory3_6D.tsv")
#PlotTrajectory(Trajectory)



#        
#
#body1 = body('cucul')
#drone1 = drone("caca",14550,"QTM\Qualisys\Data\Trajectory3_6D.tsv")
#
#print(type(drone1).__name__)
#print(type(body1).__name__)
#
#
#
#print(len(body.bodies))
#for self in body.bodies:
#    print(self.name)
#    
#print(len(drone.drones))    
#for dr in drone.drones:
#    print(dr.name)

