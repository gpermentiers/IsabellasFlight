import numpy as np
from pymavlink import mavutil

from Controller import *
from DroneCommands import *
from ReadTSV import *
from QualisysRT import *


class body:

    def __init__(self,Name):
        self.name = Name
        self.MeasPosition = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.isDrone = False
    
class drone(body):    

    def __init__(self,Name):
        body.__init__(self,Name)
        
        self.isDrone = True
        self.state = 'start'
        self.previousState = ''
        
        
        if self.name == 'DroneIsabella':
            self.MAVLinkPort = 14550
            self.TrajectoryFile = "QTM\Qualisys\Data\CircleSquare_6D.tsv"
            self.CtrlParam = {'PZ'         :0.15,    'IZ'       :  0.155,   'DZ'    :0.17,
                              'PH'         :0.25,    'IH'       :  0.015,   'DH'    :0.65,
                              'Pyaw'       :5.00}
            
        elif self.name == 'DroneElCapitan':
            self.MAVLinkPort = 14550            
            self.TrajectoryFile = "QTM\Qualisys\Data\CircleSquare_6D.tsv"
            self.CtrlParam = {'PZ'         :0.06,    'IZ'       :  0.040,   'DZ'    :0.18,
                              'PH'         :0.15,    'IH'       :  0.015,   'DH'    :0.30,
                              'Pyaw'       :5.00}
            
        else:
            self.MAVLinkPort = 14550
            self.TrajectoryFile = "QTM\Qualisys\Data\CircleSquare_6D.tsv"
            self.CtrlParam = {'PZ'         :0.15,    'IZ'       :  0.155,   'DZ'    :0.17,
                              'PH'         :0.25,    'IH'       :  0.015,   'DH'    :0.65,
                              'Pyaw'       :5.00}
                    
        print('Connecting to MAVlink')
        print('udpin:localhost:'+str(self.MAVLinkPort))
        self.MAVLinkConnection = mavutil.mavlink_connection('udpin:localhost:'+str(self.MAVLinkPort))
        print('Waiting for HeartBeat')
        self.MAVLinkConnection.wait_heartbeat()
        print("Hearbeat from system (system %u component %u)" % (self.MAVLinkConnection.target_system,self.MAVLinkConnection.target_component))
        
               
        self.NFrame, self.TrajFreq, self.tt, self.Trajectory = ReadTrajectory(self.TrajectoryFile,'Drone')
        self.ti,self.tf = self.tt[0],self.tt[-1]
        self.duration = self.tf - self.ti
        
        
        self.CtrlVar   = {'Xint'       :  0,    'Xprv'      :  0,
                          'Yint'       :  0,    'Yprv'      :  0,
                          'Zint'       :  0,    'Zprv'      :  0,
                          'Yawint'     :  0,    'Yawprv'    :  0,
                          'EEprv'      :  0}
        
        self.tStart = 0
        self.RefPosition   = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.StartPosition = [0.0,0.0,0.0,0.0,0.0,0.0]    #used to start curves, has to be reset before starting them
        self.HomePosition  = [0.0,0.0,0.0,0.0,0.0,0.0]    #position before takeoff, used to land. MUST NOT BE ALTERED
        
        
        
        
  
    def gainsTuningFSM(self,t,dt):
        
        print(self.name,'state is',self.state)
        
        if self.state == 'start':            
            setMode(self.MAVLinkConnection,'STABILIZE')
            arm(self.MAVLinkConnection)
            self.state = 'arm'
            self.RefPosition  = self.MeasPosition[:]
            self.HomePosition = self.MeasPosition[:]
            
        elif self.state == 'arm':
            self.RefPosition = self.MeasPosition[:]
            self.RefPosition[2] = 1000
            if isArmed(self.MAVLinkConnection):
                self.state = 'takeOff'
            
        elif self.state == 'takeOff':              
                
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)                        
            if (t > 25):
                self.RefPosition = self.MeasPosition[:]
                self.RefPosition[0] = 0
                self.RefPosition[1] = 0
                self.RefPosition[2] = 1000
                self.state = 'gotoStart'
            
                
        elif self.state == 'gotoStart':
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if (t > 35):
                self.tStart = t
                self.RefPosition = self.MeasPosition[:]
                self.StartPosition = self.RefPosition[:]
                self.state = 'sine'
                        
        elif self.state == 'ramp':
            self.RefPosition[2] = 1000 + 50*(t-self.tStart)
            self.RefPosition[1] = 0 + 100*(t-self.tStart)
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if (t > 40):
                self.RefPosition = self.MeasPosition[:]
                self.RefPosition[2] = 500
                self.state = 'fall'
   
        elif self.state == 'sine':
            self.RefPosition[2] = self.StartPosition[2] + 500*np.sin(0.5*(t-self.tStart))            
            self.RefPosition[1] = self.StartPosition[1] + 200*(t-self.tStart)
            self.RefPosition[0] = self.StartPosition[0] + 500*np.sin(0.5*(t-self.tStart))
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            
            if (t > 50):
                self.RefPosition = self.MeasPosition[:]
                self.tStart = t
                self.StartPosition = self.RefPosition[:]
                self.state = 'land'
        
        elif self.state == 'spin':
            #self.RefPosition[5] = (self.StartPosition[5] - 10*(t-self.tStart))%360
            self.RefPosition[5] = self.StartPosition[5] + 170
            if self.RefPosition[5] > 180:
                self.RefPosition[5] = self.RefPosition[5] - 360
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            
            if (t > 50):
                self.tStart = t
                self.RefPosition = self.MeasPosition[:]
                self.StartPosition = self.RefPosition[:]
                self.state = 'land'
                
                
                
        elif self.state == 'land':
            self.RefPosition[2] = self.StartPosition[2] - 250*(t-self.tStart)
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if ((self.MeasPosition[2]<self.HomePosition[2]+20) & (self.MeasPosition[3] < 2) & (self.MeasPosition[4] < 2)):
                self.state = 'disarm'
                
            
        elif self.state == 'disarm':
            disarm(self.MAVLinkConnection)
            manualControl(self.MAVLinkConnection,0,0,0,0)
            if not isArmed(self.MAVLinkConnection):
                self.state = 'stop'            
            
        elif self.state == 'stop':            
            pass
        
        elif self.state == 'outOfBounds':            
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            
        elif self.state == 'lostLocal':            
            print('LOST LOCALIZATION!!!')
            manualControl(self.MAVLinkConnection,350,0,0,0)
            if (not any(np.isnan(self.MeasPosition))):
                #self.state = self.previousState
                self.state = 'lostLocal'
                
        elif self.state == 'test':            
            print('test')
    
     
    
#--------------------------------------------------------
            
            
            
    def FSM(self,t,dt):
        
        print(self.name,'state is',self.state)
        
        if self.state == 'start':            
            setMode(self.MAVLinkConnection,'STABILIZE')
            arm(self.MAVLinkConnection)
            self.RefPosition = self.MeasPosition[:]
            self.HomePosition = self.MeasPosition[:]
            self.state = 'arm'
            
        elif self.state == 'arm':
            self.RefPosition = self.MeasPosition[:]
            self.RefPosition[2] = 1000
            if isArmed(self.MAVLinkConnection):
                self.state = 'takeOff'
            
        elif self.state == 'takeOff':              
                
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)                
            if (((EE < 150) & (dEE < 20) )| (t > 15)):          
                self.RefPosition = self.Trajectory[0]
                self.state = 'gotoStart'
            
                
        elif self.state == 'gotoStart':
            
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if (t > 35):
                self.RefPosition = self.Trajectory[0]
                self.state = 'ready'
            
        elif self.state == 'ready':          
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if (t > 36):
                self.tStart = t
                self.state = 'followTraj'
            
        elif self.state == 'followTraj':
            if(t > self.duration+self.tStart):                
                self.RefPosition = self.MeasPosition[:]
                self.tStart = t
                self.StartPosition = self.RefPosition[:]
                self.state = 'land'
            else:
                i = int((t-self.tStart)*self.TrajFreq)
                self.RefPosition = self.Trajectory[i]
                print('Reference',self.RefPosition,'read at time:',self.tt[i])
                [T,R,P,Y,EE,dEE] = DroneController(self,dt)
                manualControl(self.MAVLinkConnection,T,R,P,Y)
                            
        elif self.state == 'land':
            self.RefPosition[2] = self.StartPosition[2] - 250*(t-self.tStart)
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            if ((self.MeasPosition[2]<self.HomePosition[2]+30) & (self.MeasPosition[3] < 10) & (self.MeasPosition[4] < 10)):
                self.state = 'disarm'             
            
        elif self.state == 'disarm':
            disarm(self.MAVLinkConnection)
            manualControl(self.MAVLinkConnection,0,0,0,0)
            if not isArmed(self.MAVLinkConnection):
                self.state = 'stop'            
            
        elif self.state == 'stop':            
            pass
                                
        elif self.state == 'outOfBounds':            
            [T,R,P,Y,EE,dEE] = DroneController(self,dt)
            manualControl(self.MAVLinkConnection,T,R,P,Y)
            
        elif self.state == 'lostLocal':            
            print('LOST LOCALIZATION!!!')
            manualControl(self.MAVLinkConnection,350,0,0,0)
            if (not any(np.isnan(self.MeasPosition))):
                self.state = self.previousState
                
        elif self.state == 'test':            
            print('test')
            
            




def ramp(distance,duration,time):
    
    if (time < 0):
        return 0
    if (time > duration):
        return distance
    else:
        return (distance/duration)*time
    

            