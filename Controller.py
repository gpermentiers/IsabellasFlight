import numpy as np
import time

    
def DroneController(Drone,dt):      
    
    Xmeas     = Drone.MeasPosition[0]
    Ymeas     = Drone.MeasPosition[1]
    Zmeas     = Drone.MeasPosition[2]
    ROLLmeas  = Drone.MeasPosition[3]
    PITCHmeas = Drone.MeasPosition[4]
    YAWmeas   = Drone.MeasPosition[5]
    
    Xref  = Drone.RefPosition[0]
    Yref  = Drone.RefPosition[1]
    Zref  = Drone.RefPosition[2]
    YAWref = Drone.RefPosition[5]
    if YAWref > 180:
        YAWref = YAWref - 360
    if YAWref < -180:
        YAWref = YAWref + 360
    
    Xerror = Xref - Xmeas
    Yerror = Yref - Ymeas
    Zerror = Zref - Zmeas
    YAWerror = YAWref - YAWmeas
    if YAWerror > 180:
        YAWerror = YAWerror - 360
    if YAWerror < -180:
        YAWerror = YAWerror + 360
    
    XerrorD   = (Xerror   - Drone.CtrlVar['Xprv'])  /dt
    YerrorD   = (Yerror   - Drone.CtrlVar['Yprv'])  /dt
    ZerrorD   = (Zerror   - Drone.CtrlVar['Zprv'])  /dt
    YAWerrorD = (YAWerror - Drone.CtrlVar['Yawprv'])/dt
    
    
    Drone.CtrlVar['Xint']   = Drone.CtrlVar['Xint']   + Xerror*dt
    Drone.CtrlVar['Yint']   = Drone.CtrlVar['Yint']   + Yerror*dt
    Drone.CtrlVar['Zint']   = Drone.CtrlVar['Zint']   + Zerror*dt
    Drone.CtrlVar['Yawint'] = Drone.CtrlVar['Yawint'] + YAWerror*dt
    
    XerrorI   = Drone.CtrlVar['Xint'] 
    YerrorI   = Drone.CtrlVar['Yint'] 
    ZerrorI   = Drone.CtrlVar['Zint'] 
    YAWerrorI = Drone.CtrlVar['Yawint']    
#     print('Ze',Zerror)
#     print('Zde',ZerrorD)
#     print('Zie',ZerrorI)
    EuclidianError = np.sqrt(Xerror**2 + Yerror**2 + Zerror**2)
    dEE = (EuclidianError - Drone.CtrlVar['EEprv'])/dt
        
    XComp  =  Drone.CtrlParam['PH']*Xerror + Drone.CtrlParam['IH']*XerrorI + Drone.CtrlParam['DH']*XerrorD
    YComp  =  Drone.CtrlParam['PH']*Yerror + Drone.CtrlParam['IH']*YerrorI + Drone.CtrlParam['DH']*YerrorD
    
    Thrust =  Drone.CtrlParam['PZ']*Zerror + Drone.CtrlParam['IZ']*ZerrorI + Drone.CtrlParam['DZ']*ZerrorD  
    Pitch  =  -np.cos(np.deg2rad(YAWmeas))*XComp - np.sin(np.deg2rad(YAWmeas))*YComp
    Roll   =  -np.sin(np.deg2rad(YAWmeas))*XComp + np.cos(np.deg2rad(YAWmeas))*YComp
    Yaw    =  Drone.CtrlParam['Pyaw'] *YAWerror
        
    thrust_range = 200
    tilt_range = 400
    yaw_range = 180
    
    Thrust = int(min(max(Thrust,500-thrust_range),500+thrust_range)) 
    Roll   = int(min(max(Roll,-tilt_range),tilt_range))
    Pitch  = int(min(max(Pitch,-tilt_range),tilt_range))
    Yaw    = int(min(max(Yaw,-yaw_range),yaw_range))
    
    Drone.CtrlVar['Xprv']   = Xerror
    Drone.CtrlVar['Yprv']   = Yerror
    Drone.CtrlVar['Zprv']   = Zerror
    Drone.CtrlVar['Yawprv'] = YAWerror    
    Drone.CtrlVar['EEprv']  = EuclidianError
    
    print('TRPY:',Thrust,Roll,Pitch,Yaw)
    print('EE,dEE:',EuclidianError,dEE)
    return [Thrust,Roll,Pitch,Yaw,EuclidianError,dEE]
