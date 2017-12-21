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

# loop to add z dimension to path_list
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
#print(lines_list_diag)
lines_list_diag = criss_cross(dog_bone,d,num_layers,z,skin)
#lines_list_diag = [[add_Z(path_list_1,z[0])[0],add_Z(path_list_1,z[0])[1],add_Z(path_list_1,z[0])[2],add_Z(path_list_1,z[0])[3],add_Z(path_list_2,z[1])[1]]]
def optimize_horz_path(dog_bone,d,z,tool):
    new_path_list = []
    path_list = polygon_path(dog_bone,d,plotting=False,angle=np.pi/2)
    new_path_list.append(path_list[0])
    new_path_list.append(path_list[2][::-1])
    new_path_list.append(path_list[1][::-1])
    #new_path_list.append(path_list[3][::-1])
    lines_list_horz = add_Z(new_path_list,z,tool)
    return(lines_list_horz)

lines = dog_bone
num_offset = 1
offset_list = []
for k in range(num_offset):
    offset_list.append(offset_lines(lines,d*float(k+0.5),left = True))
offset_list = np.vstack(offset_list)

# plot offset walls
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(offset_list[:,0],offset_list[:,1])
ax.plot(dog_bone[:,0],dog_bone[:,1])
plt.axis("equal")
plt.grid()
plt.show()

# Add Z height to offset walls
offset_list = [offset_list]
lines_list_offset = add_Z(offset_list,z,skin)

def Optimize_muscle_tool_path(lines_list,z,num_layers):
    z_end = z[num_layers]+2
    for k in range(len(lines_list)):
        if lines_list[k][0] == MUSCLE:
            start_point = lines_list[k][1][0,:]
            start_point[2] = z_end
            lines_list[k][1] = np.vstack([start_point,lines_list[k][1]])
    return(lines_list)

# script for mulitple sets of walls and muscle in one print TEST
lines_list_vert = []
lines_list_walls = []
x = 0
num_offset = 2
for j in range(0,2):
    for i in range(0,3):
        y = (h2+10)*i
        dog_bone = np.array(
        [[-l2/2+x,h2/2+y],[-l3/2+x,h2/2+y],[-l1/2+x,h1/2+y],[l1/2+x,h1/2+y],[l3/2+x,h2/2+y],[l2/2+x,h2/2+y],[l2/2+x,-h2/2+y],[l3/2+x,-h2/2+y],[l1/2+x,-h1/2+y],[-l1/2+x,-h1/2+y],[-l3/2+x,-h2/2+y],[-l2/2+x,-h2/2+y],[-l2/2+x,h2/2+y]
        ])
        #dog_bone = np.array([[-l3/2+x,h1/2+y],[l3/2+x,h1/2+y],[l3/2+x,-h1/2+y],[-l3/2+x,-h1/2+y],[-l3/2+x,h1/2+y]])
        if j == 0:
            offset_list =np.vstack([offset_lines(dog_bone,d*float(k+0.5),left = True) for k in range(num_offset)])
            #lines_list_offset = add_Z(offset_list,z,skin)
            lines_list_walls.append(add_Z(offset_list,z,skin))
            #print(lines_list_walls)
        else:
            path_list_vert = polygon_path(offset_lines(dog_bone,d*0.2,left = False),d,plotting=False,angle=0)
            #print(path_list_vert)
            a = np.vstack((path_list_vert[0][0],path_list_vert[0]))
#             lines_list_walls.append(add_Z(a,z,muscle))
            lines_list_walls.append(add_Z(a,np.ones(z.shape)*z[-1]+5,muscle))
            
lines_list_walls = Optimize_muscle_tool_path(lines_list_walls,z,num_layers)



#lines_list_walls = [lines_list_walls]
g_code_strings = lines_list_to_gcode_strings(lines_list_walls,gcode_para)
for gcode in g_code_strings:
    print(gcode)
# dumping:
g_code_strings.insert(0,'G0 X-200.0000 Y-200.0000 Z15.0000 F1500\nG1 F{0:.3f}\nG1 E0:0:0:0\nG1 E0:9:9:0\nG4 P10000\nG0 F1800\n'.format(gcode_para['feedrate_extrude']))
# initialization
init = initilize_parameters(4,is_absolute=True,is_mm=True,feedrate = 1500 )
g_code_strings.insert(0,init)
#print(g_code_strings)
s = ''.join(g_code_strings)

%matplotlib notebook
plot_lines_list(lines_list_walls)
