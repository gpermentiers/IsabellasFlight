import asyncio
import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import qtm
import time
from pymavlink import mavutil
from datetime import datetime

from Controller import *
from DroneCommands import *
from ReadTSV import *
from QualisysRT import *
from DroneObject import *


setupWindow = False
tStop = 90
Freq  = 10
Tperiod = 1/Freq

async def main():
    
    # Connect to qtm
    connection = await qtm.connect("127.0.0.1")

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection,""):

        realtime = True

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load('Trajectory2.qtm')

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    bodies = []
    bodies.append(drone('DroneElCapitan'))
    
#     for bodyName in body_index:
#         if bodyName[:5] == 'Drone':
#             bodies.append(drone(bodyName,14550,"QTM\Qualisys\Data\Measurement12_6D.tsv"))
#         else:
#             bodies.append(body(bodyName))
        
    #hl = startPlot()
    
    measgraph  = np.zeros((tStop*Freq,6))
    refgraph  = np.zeros((tStop*Freq,6))
    tgraph = np.zeros(tStop*Freq)
    
    tstart= time.perf_counter()
    t = tstart
    counter = 0
    sumdt = 0
    dt = 0
    
    Mission = True
    
    while(Mission):

        
        tic = time.perf_counter()
        t = tic - tstart
        print('\nCurrent time:',t,'s')
        
        counter += 1
        
        packet = await connection.get_current_frame(components = ["6d"])
     
        for bd in bodies:
            
            bd.MeasPosition = Get6dof(packet,bd.name,body_index)
           
            if bd.isDrone:
                
                if (any(np.isnan(bd.MeasPosition)) & (bd.state != 'lostLocal')):
                    bd.previousState = bd.state
                    bd.state = 'lostLocal'
                
                measgraph[counter] = bd.MeasPosition
                refgraph[counter] = bd.RefPosition
                tgraph[counter] = t                
                          
                #bd.gainsTuningFSM(t,Tperiod)
                bd.FSM(t,Tperiod) 
                
                if (bd.state == 'stop') | (t > tStop) :
                    Mission = False
                    
        toc = time.perf_counter()
        dt = toc - tic
        sumdt += dt
        print('Time to execute step:',dt,'s\n------------------------------------------')
        await asyncio.sleep(Tperiod-(time.perf_counter()-tic))
    
    print('\nMean frequency:',counter/t,'Hz')
    print('Mean time step:',sumdt/counter,'s')
    
    
    plt.plot(tgraph[:counter],measgraph[:counter,2],label='Measurement')
    plt.plot(tgraph[:counter],refgraph[:counter,2],label='Reference',color='red')
    plt.title('Graph of altitude (PID gains = '+str(bodies[0].CtrlParam['PZ'])+', '+str(bodies[0].CtrlParam['IZ'])+', '+str(bodies[0].CtrlParam['DZ'])+')')
    plt.xlabel('time [$s$]')
    plt.ylabel('Altitude [$mm$]')
    plt.legend()
    plt.savefig('graph/altitude '+str(datetime.today())[:19].replace(":","-")+'.png')
    plt.show()
    
    plt.plot(tgraph[:counter],measgraph[:counter,0],label='Measurement')
    plt.plot(tgraph[:counter],refgraph[:counter,0],label='Reference',color='red')
    plt.title('Graph of X (PID gains = '+str(bodies[0].CtrlParam['PH'])+', '+str(bodies[0].CtrlParam['IH'])+', '+str(bodies[0].CtrlParam['DH'])+')')
    plt.xlabel('time [$s$]')
    plt.ylabel('X [$mm$]')
    plt.legend()
    plt.savefig('graph/X '+str(datetime.today())[:19].replace(":","-")+'.png')
    plt.show()
    
    plt.plot(tgraph[:counter],measgraph[:counter,1],label='Measurement')
    plt.plot(tgraph[:counter],refgraph[:counter,1],label='Reference',color='red')
    plt.title('Graph of Y (PID gains = '+str(bodies[0].CtrlParam['PH'])+', '+str(bodies[0].CtrlParam['IH'])+', '+str(bodies[0].CtrlParam['DH'])+')')
    plt.xlabel('time [$s$]')
    plt.ylabel('Y [$mm$]')
    plt.legend()
    plt.savefig('graph/Y '+str(datetime.today())[:19].replace(":","-")+'.png')
    plt.show()
    
    plt.plot(tgraph[:counter],measgraph[:counter,5],label='Measurement')
    plt.plot(tgraph[:counter],refgraph[:counter,5],label='Reference',color='red')
    plt.title('Graph of yaw (P gain = '+str(bodies[0].CtrlParam['Pyaw'])+')')
    plt.xlabel('time [$s$]')
    plt.ylabel('Yaw [°]')
    plt.legend()
    plt.savefig('graph/Yaw '+str(datetime.today())[:19].replace(":","-")+'.png')
    plt.show()
    
    PlotTrajectories(measgraph[1:counter],refgraph[1:counter])

   
if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())

