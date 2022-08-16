import asyncio
import xml.etree.ElementTree as ET
import numpy as np
import scipy.signal
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import qtm
import time
import sys
from pymavlink import mavutil

from Controller import *
from DroneCommands import *
from ReadTSV import *
from QualisysRT import *
from DroneObject import *







Folder = 'hover'
Target = "Drone"
clr = 'navy'

NFrame,Freq,tgraph,measgraph = ReadTrajectory("QTM\Qualisys\Data\TestHover_6D.tsv",Target)

# if Target == "Drone":
#     b, a = scipy.signal.butter(3, 0.2, 'highpass')
#     for i in range(6):
#         measgraph[:,i] = scipy.signal.filtfilt(b, a, measgraph[:,i])


plt.plot(tgraph[:],measgraph[:,2],label='Mean = '+str(np.mean(measgraph[:,2]))[:6]+' mm',color = clr)
plt.title('Graph of altitude')
plt.xlabel('time [$s$]')
plt.ylabel('Altitude [$mm$]')
plt.ylim(np.mean(measgraph[:,2])-6*np.std(measgraph[:,2]),np.mean(measgraph[:,2])+6*np.std(measgraph[:,2]))
plt.legend(loc=1)
print('Mean of altitude:',np.mean(measgraph[:,2]))
plt.savefig(Folder+'/char'+Target+'altitudetime.png')
plt.show()

plt.hist(measgraph[:,2],label='Standard deviation = '+str(np.std(measgraph[:,2]))[:6]+' mm',color = clr)
plt.title('Histogram of altitude')
plt.xlabel('Altitude [$mm$]')
plt.ylabel('# []')
plt.legend(loc=1)
print('Standard deviation of altitude:',np.std(measgraph[:,2]))
plt.savefig(Folder+'/char'+Target+'altitudehist.png')
plt.show()


plt.plot(tgraph[:],measgraph[:,0],label='Mean = '+str(np.mean(measgraph[:,0]))[:6]+' mm',color = clr)
plt.title('Graph of X')
plt.xlabel('time [$s$]')
plt.ylabel('X [$mm$]')
plt.ylim(np.mean(measgraph[:,0])-6*np.std(measgraph[:,0]),np.mean(measgraph[:,0])+6*np.std(measgraph[:,0]))
plt.legend(loc=1)
print('Mean of X:',np.mean(measgraph[:,0]))
plt.savefig(Folder+'/char'+Target+'Xtime.png')
plt.show()

plt.hist(measgraph[:,0],label='Standard deviation = '+str(np.std(measgraph[:,0]))[:6]+' mm',color = clr)
plt.title('Histogram of X')
plt.xlabel('X [$mm$]')
plt.ylabel('# []')
plt.legend(loc=1)
print('Standard deviation of X:',np.std(measgraph[:,0]))
plt.savefig(Folder+'/char'+Target+'Xhist.png')
plt.show()


plt.plot(tgraph[:],measgraph[:,1],label='Mean = '+str(np.mean(measgraph[:,1]))[:6]+' mm',color = clr)
plt.title('Graph of Y')
plt.xlabel('time [$s$]')
plt.ylabel('Y [$mm$]')
plt.ylim(np.mean(measgraph[:,1])-6*np.std(measgraph[:,1]),np.mean(measgraph[:,1])+6*np.std(measgraph[:,1]))
plt.legend(loc=1)
print('Mean of Y:',np.mean(measgraph[:,1]))
plt.savefig(Folder+'/char'+Target+'Ytime.png')
plt.show()

plt.hist(measgraph[:,1],label='Standard deviation = '+str(np.std(measgraph[:,1]))[:6]+' mm',color = clr)
plt.title('Histogram of Y')
plt.xlabel('Y [$mm$]')
plt.ylabel('# []')
plt.legend(loc=1)
print('Standard deviation of Y:',np.std(measgraph[:,1]))
plt.savefig(Folder+'/char'+Target+'Yhist.png')
plt.show()


plt.plot(tgraph[:],measgraph[:,5],label='Mean = '+str(np.mean(measgraph[:,5]))[:6]+' °',color = clr)
plt.title('Graph of yaw')
plt.xlabel('time [$s$]')
plt.ylabel('Yaw [°]')
plt.ylim(np.mean(measgraph[:,5])-6*np.std(measgraph[:,5]),np.mean(measgraph[:,5])+6*np.std(measgraph[:,5]))
plt.legend(loc=1)
print('Mean of Yaw:',np.mean(measgraph[:,5]))
plt.savefig(Folder+'/char'+Target+'Yawtime.png')
plt.show()

plt.hist(measgraph[:,5],label='Standard deviation = '+str(np.std(measgraph[:,5]))[:6]+' °',color = clr)
plt.title('Histogram of Yaw')
plt.xlabel('Yaw [°]')
plt.ylabel('# []')
plt.legend(loc=1)
print('Standard deviation of Yaw:',np.std(measgraph[:,5]))
plt.savefig(Folder+'/char'+Target+'Yawhist.png')
plt.show()

    





