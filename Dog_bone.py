from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm
from sys import platform
from point_in_polygon import wn_PnPoly
from pathplanner import *
%config InlineBackend.figure_format = 'retina'
%matplotlib inline

from muscle_printing import *

TOOL = { # ratio between extruders
    CONDUCTIVE:[0, 0.5, 0, 0.5],
    MUSCLE:[0.5, 0, 0,  0.5],
    SKIN:[0, 0.5, 0.5,  0]
}
D = 22.63
# d = 1.75 # gauge14
d = .99  # gauge18

gcode_para = { # G code parameters
    'D':D,
    'd':d,
    'applied_percentage':1,
    'TOOL':TOOL,
    'dump_location':[-200, -200, 55],
    'feedrate_move':1000,
    'feedrate_quickmove':1800,
    'feedrate_extrude':(d**2) / (3**2),
    'total_volume_change':3800.0, # [mm^3]
    'max_reuse_volume':1000,
    }
###################################################################################################

def add_Z(path_list,z,tool):
    if type(path_list)!=list:
        path_list = [path_list]
    if type(z) == np.ndarray:
        new_path_list = [tool]
        for z_k in z:
            for path in path_list:
                path_with_z =np.hstack([path,np.ones((path.shape[0],1))*z_k])
                new_path_list.append(path_with_z)
    else:
        new_path_list = [tool]
        for path in path_list:
            path_with_z =np.hstack([path,np.ones((path.shape[0],1))*z])
            new_path_list.append(path_with_z)
    return(new_path_list)
def height_vector(num_layers,layer_height,z_start):
    cylinder_height = num_layers*layer_height
    num = num_layers + 1
    #n = np.linspace(0,10,num)
    n = np.arange(0,100)
    k_n = 0.2  # .1 for printing just silicon(worked perfeclty for cylinder)/.2 for printing silicon and ethanol
    A_final = 0.8 # .8 for printing just silicon
    d_n = A_final+np.exp(-k_n*n)*(1-A_final)
    k = np.linspace(z_start,cylinder_height+z_start,num)
    d_n = d_n[0:len(k)]
    k = k*d_n
    #k = z_start-d + d * d_n.cumsum()
    return(k)
    
skin = 2
muscle  = 1
num_layers = 15
z_start = 4.7
l1=20
l3 = 30
l2=50
h1=10
h2=20

# For ASTM D412-16 Die A
# l1=59
# l3 = 74 # for 45 degree angle from inner to outer rectangle
# #l3 = 84.98 # for 30 degree angle from inner to outer rectangle
# l2=140

# h1=12
# h2=25

# For ASTM D412-16 Die C
#l1=59
#l3 = 74 # for 45 degree angle from inner to outer rectangle
#l3 = 84.98 # for 30 degree angle from inner to outer rectangle
#l2=115

#h1=6
#h2=25

# height of ASTM D412-16 Die A is 3 mm
# d = .99
z = height_vector(num_layers,d,z_start)
x = 0 # offset in x-direction
y = 0 # offset in y-direction
dog_bone = np.array(
[[l1/2+x,h1/2+y],[l3/2+x,h2/2+y],[l2/2+x,h2/2+y],[l2/2+x,-h2/2+y],[l3/2+x,-h2/2+y],[l1/2+x,-h1/2+y],[-l1/2+x,-h1/2+y],[-l3/2+x,-h2/2+y],[-l2/2+x,-h2/2+y],[-l2/2+x,h2/2+y],
[-l3/2+x,h2/2+y],[-l1/2+x,h1/2+y],[l1/2+x,h1/2+y]]
)
path_list = polygon_path(dog_bone,d,plotting=True,angle=-np.pi/4)       
lines_list_diag_single = add_Z(path_list,z,skin)
path_list = polygon_path(dog_bone,d,plotting=True,angle=0)
lines_list_vert = add_Z(path_list,z,skin)
path_list = polygon_path(dog_bone,d,plotting=True,angle=np.pi/2)
lines_list_horz = add_Z(path_list,z,skin)    

#script to cris-cross the diagonal pattern
def criss_cross(dog_bone,d,num_layers,z,tool):
    path_list_1 = polygon_path(dog_bone,d,plotting=False,angle=-np.pi/4)
    path_list_2 = polygon_path(dog_bone,d,plotting=False,angle=np.pi/4)
    lines_list_diag = [add_Z(path_list_1,z[0],tool)[0]]
    for i in range(num_layers+1):
        if i % 2 == 0:
            path = path_list_1
        else:
            path = path_list_2
        for j in range(1,len(add_Z(path,z[i],tool))):
            lines_list_diag.append(add_Z(path,z[i],tool)[j])
    lines_list_diag = [lines_list_diag]
    return(lines_list_diag)

lines_list_diag = criss_cross(dog_bone,d,num_layers,z,skin)

def optimize_horz_path(dog_bone,d,z,tool):
    new_path_list = []
    path_list = polygon_path(dog_bone,d,plotting=False,angle=np.pi/2)
    new_path_list.append(path_list[0])
    new_path_list.append(path_list[2][::-1])
    new_path_list.append(path_list[1][::-1])
    #new_path_list.append(path_list[3][::-1])
    lines_list_horz = add_Z(new_path_list,z,tool)
    return(lines_list_horz)

# script for 3 types (diag,horz,vert) dog_bones in one print
lines_list_diag = []
lines_list_horz = []
lines_list_vert = []
lines_list = []
for j in range(-1,2):
    for i in range(-2,3):
        x = (l2+10)*j
        if j == -1 or j == 1:
            y = (h2+10)*(i*-1)
        else:
            y = (h2+10)*i
        dog_bone = np.array(
        [[l1/2+x,h1/2+y],[l3/2+x,h2/2+y],[l2/2+x,h2/2+y],[l2/2+x,-h2/2+y],[l3/2+x,-h2/2+y],[l1/2+x,-h1/2+y],[-l1/2+x,-h1/2+y],[-l3/2+x,-h2/2+y],[-l2/2+x,-h2/2+y],[-l2/2+x,h2/2+y],
        [-l3/2+x,h2/2+y],[-l1/2+x,h1/2+y],[l1/2+x,h1/2+y]]
        )
        if j == -1:            
            path_list_vert = polygon_path(dog_bone,d,plotting=False,angle=0)
            lines_list_vert.append(add_Z(path_list_vert,z,skin))
            lines_list.append(add_Z(path_list_vert,z,skin))
        if j == 0:
            lines_list_diag.append(criss_cross(dog_bone,d,num_layers,z,skin))
            lines_list.append(criss_cross(dog_bone,d,num_layers,z,skin))
        if j == 1:
            lines_list_horz.append(optimize_horz_path(dog_bone,d,z,skin))
            lines_list.append(optimize_horz_path(dog_bone,d,z,skin))

for all patterns
for i in range(len(lines_list)):
    j = len(lines_list)-1-i
    if i == 0:
        g_code_strings = lines_list_to_gcode_strings(lines_list[j])
    else:
        g_code_strings.insert(0,lines_list_to_gcode_strings(lines_list[j])[0])

# dumping:
g_code_strings.insert(0,'G0 X-200.0000 Y-200.0000 Z15.0000 F1500\nG1 F0.4624\nG1 E0:3.5:3.5:0 F0.4624\nG4 P6000\nG0 F1800')
# initialization
init = initilize_parameters(4,is_absolute=True,is_mm=True,feedrate = 1500 )
g_code_strings.insert(0,init)

s = ''.join(g_code_strings)