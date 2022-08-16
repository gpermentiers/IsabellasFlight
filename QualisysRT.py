import xml.etree.ElementTree as ET
import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import qtm
import time

QTM_FILE = 'Trajectory3.qtm'

def startPlot():
    map = plt.figure()
    map_ax = Axes3D(map)
    map_ax.autoscale(enable=True, axis='both', tight=True)

    # # # Setting the axes properties
    maxRange = 5000
    map_ax.set_xlim3d([-maxRange, maxRange])
    map_ax.set_ylim3d([-maxRange, maxRange])
    map_ax.set_zlim3d([0.0, 2*maxRange])

    hl, = map_ax.plot3D([], [], [],'red')
    update_line(hl, ([],[],[]))
    return hl
    
    
def update_line(hl, new_data):
    xdata, ydata, zdata = hl._verts3d
    hl.set_xdata(list(np.append(xdata, new_data[0])))
    hl.set_ydata(list(np.append(ydata, new_data[1])))
    hl.set_3d_properties(list(np.append(zdata, new_data[2])))
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)
    
    
def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def Get6dof(packet,wanted_body,body_index):
    info, bodies = packet.get_6d()
    
    #print("Framenumber: {} - Body count: {}".format(packet.framenumber, info.body_count))

    if wanted_body is not None and wanted_body in body_index:
        # Extract one specific body
        wanted_index = body_index[wanted_body]
        position, rotation = bodies[wanted_index]
        #print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
        rotAngles = RotMatrice2RotAngle(rotation.matrix)
        MeasPosition = [position.x,position.y, position.z,rotAngles[0],rotAngles[1],rotAngles[2]]
        return MeasPosition
        
    else:
        print(wanted_body,'does not exist!')
        return np.empty(6)

def RotMatrice2RotAngle(matrix):
    Pitch =  np.arcsin(matrix[6])
    Roll  = -np.arccos(min(max(matrix[8]/np.cos(Pitch),-1),1))*np.sign(matrix[7])
    Yaw   = -np.arccos(min(max(matrix[0]/np.cos(Pitch),-1),1))*np.sign(matrix[3])
    return np.rad2deg([Roll,Pitch,Yaw])

