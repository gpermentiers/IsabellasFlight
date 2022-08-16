import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


    
def ReadTrajectory(filename,body):

    with open(filename) as file:
        NFrame = int(file.readline().split()[1])
        NCamera = int(file.readline().split()[1])
        NBodies = int(file.readline().split()[1])
        Freq = int(file.readline().split()[1])
        NAnalog = int(file.readline().split()[1])
        AnFreq = int(file.readline().split()[1])
        Desc = file.readline().split()[1:]
        TimeStamp = file.readline().split()[1:]
        DataInc = file.readline().split()[1:]
        BodyNames = file.readline().split()[1:]
        BodyFilter = file.readline().split()[1:]
        TransOrigin = file.readline().split()[1:]
        RotOrigin = file.readline().split()[1:]
        Header = file.readline().split()[:]
        
        DATA   = np.array(list(list(float(w) for w in file.readline().split()[1:]) for i in range(NFrame))) # x,y,z, r,p,w 
        
        
        TimeTable    = DATA[:,0]
        for i in range(int(NBodies)):
            if body == BodyNames[i]:
                Trajectory   = DATA[:,1+16*i:7+16*i]
    return NFrame,Freq,TimeTable,Trajectory 



def PlotTrajectory(Trajectory):
    
    X = Trajectory[:,0]
    Y = Trajectory[:,1]
    Z = Trajectory[:,2]
    
    map = plt.figure()
    ax = Axes3D(map)
    ax.autoscale(enable=True, axis='both', tight=True)
    ax.plot3D(X,Y,Z,'navy')
    
    X = X[~np.isnan(X)]
    Y = Y[~np.isnan(Y)]
    Z = Z[~np.isnan(Z)]
    
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(0, 2*max_range)
    
    ax.set_title('Trajectory')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    plt.show()

def PlotTrajectories(Trajectory,Reference):
    
    X = Trajectory[:,0]
    Y = Trajectory[:,1]
    Z = Trajectory[:,2]
    
    Xr = Reference[:,0]
    Yr = Reference[:,1]
    Zr = Reference[:,2]
    
    map = plt.figure()
    ax = Axes3D(map)
    ax.autoscale(enable=True, axis='both', tight=True)
    ax.plot3D(X,Y,Z,'blue')
    ax.plot3D(Xr,Yr,Zr,'red')
    
    X = X[~np.isnan(X)]
    Y = Y[~np.isnan(Y)]
    Z = Z[~np.isnan(Z)]
    
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(0, 2*max_range)
    
    ax.set_title('Trajectory')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    plt.show()
    
    
    
    
def main():
    NFrame,Freq,TimeTable,Trajectory = ReadTrajectory("QTM\Qualisys\Data\TestHover_6D.tsv",'Drone')
    PlotTrajectory(Trajectory)
    #NFrame,Freq,TimeTable,Reference = ReadTrajectory("QTM\Qualisys\Data\Measurement26_6D.tsv",'Drone')
    #PlotTrajectories(Trajectory,Reference)
    

if __name__ == "__main__":
    main()