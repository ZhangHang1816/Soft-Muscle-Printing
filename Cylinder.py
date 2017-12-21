from muscle_printing import *

TOOL = { # ratio between extruders
    CONDUCTIVE:[0, 0.5, 0, 0.5],
    MUSCLE:[0.5, 0, 0,  0.5],
    SKIN:[0, 0.5, 0.5,  0]}
D = 22.63
# d = 1.75 # gauge14
# d = 1.15  # gauge16
d = 0.99 # guage18



gcode_para = { # G code parameters
    'D':D,
    'd':d,
    'applied_percentage':1,
    'TOOL':TOOL,
    'dump_location':[-200, -200, 55],
    'feedrate_move':600,
    'feedrate_quickmove':1800,
    'feedrate_extrude':(d**2) / (3**2),
    'total_volume_change':3900.0, # [mm^3]
    'max_reuse_volume':500,
    }
###################################################################################################

def spiral(center,z,k,start,layer_width,resolution,material):
    """CENTER is the [x y] cordinates for the center of the cylinder. 
    z is the height of the layer from the build plate. K is the number of times 
    the spiral makes a 360 degree rotation. START is the location of the start point 
    of the sprial from the center in the y direction (is it typically a multiple of the 
    layer width or syringe tip diameter). LAYER WIDTH is the distance between lines in 
    spiral(typically the diameter of the syringe tip). Resolution is the number of straight 
    segments in 360 degress of the one sprial turn(recommended is 25 to 20).MATERIAL is a number               representing the type of material to be extruded, this will select the two extruders to be used
    during the print."""
    line_list = [material]
    end = start+layer_width*k
    n = k*20
    r = np.linspace(start,end,n)
    line  = np.empty((n,3))
    line_2  = np.empty((n,3))
    t = np.linspace(0,2*k*np.pi,n)-np.pi/2
    line[:,0] = center[0]+r*np.cos(t)
    line[:,1] = center[1]-r*np.sin(t)
    line[:,2] = z
    line_list.append(line)
    return line_list

def height_vector(num_layers,layer_height,z_start):
    cylinder_height = num_layers*layer_height
    num = int(num_layers + 1)
    #n = np.linspace(0,10,num)
    n = np.arange(0,100)
    k_n = 0.3
    A_final = 0.7
    d_n = A_final+np.exp(-k_n*n)*(1-A_final)
    k = np.linspace(z_start,cylinder_height+z_start,num)
    d_n = d_n[0:len(k)]
    print(d_n)
    k = k*d_n
    #k = z_start-d + d * d_n.cumsum()
    return(k)

# script to print points in the sprial
# current paramters work perfectly - print was succesfful 
# Feedrate of 800 is optimal for prefect print
# 16 Gauge tip was used
# 00-35 was used
# .4 precision on decimal places used in toolpath
import math
# intial parameters
center = [0,0]
# SKIN = 2 # defined in muscle_printing.py
# MUSCLE = 1
# CONDUCTIVE = 0
layer_height = d
layer_width = d
resolution = 25
z_start =3.75+d
num_layers = 20 # for height of x mm
cylinder_height = layer_height*num_layers

# Adjusting for sagging in z height
# d is vector containg the adjusted z height for each layer going up
d = height_vector(num_layers,layer_height,z_start)

# parameters for layer to cover entire diameter
k_full = 12 # will give a cylinder diamter of 40 mm
start_full = 0 

k_additional = 4
#paramters for layer to cover outer two circle of spiral
k_outside = 2
start_outside = (k_full-k_outside)*layer_width

base_layers = 1
lines_list_1 = [SKIN]
for i in range(num_layers-base_layers):
    if i==0:
        line = np.flip(spiral(center,d[i],k_full+k_additional,start_full,layer_width,resolution,SKIN)[1],0)
    elif i <= base_layers-1:        
        if i % 2 == 0:
            line = spiral(center,d[i],k_full,start_full,layer_width,resolution,SKIN)[1]
        elif i % 2 != 0:
            line = np.flip(spiral(center,d[i],k_full,start_full,layer_width,resolution,SKIN)[1],0)
    else:
        if i % 2 != 0:
            line = spiral(center,d[i],k_outside,start_outside,layer_width,resolution,SKIN)[1]
        if i % 2 == 0:
            line = np.flip(spiral(center,d[i],k_outside,start_outside,layer_width,resolution,SKIN)[1],0)
    lines_list_1.append(line)

# Muscle extrusion
# do same spiral motion for same number of layers but don't change the z height
k_middle = k_full - k_outside - 2
start_middle = 0
lines_list_2 = [MUSCLE]
for i in range(num_layers-base_layers-base_layers-1):
    print(i)
    if i == 0:
        line = spiral(center,d[num_layers-base_layers],k_middle,start_middle,layer_width,resolution,MUSCLE)[1]
    elif i % 2 == 0: 
        line = spiral(center,d[num_layers-base_layers],k_middle,start_middle,layer_width,resolution,MUSCLE)[1]
    elif i % 2 != 0: 
        line = np.flip(spiral(center,d[num_layers-base_layers],k_middle,start_middle,layer_width,resolution,SKIN)[1],0)
    lines_list_2.append(line)
    
# top layer - skin
lines_list_3 = [SKIN]
for i in range(num_layers-base_layers,num_layers):
    if i % 2 == 0:
        line = spiral(center,d[i],k_full,start_full,layer_width,resolution,SKIN)[1]
    elif i % 2 != 0:
        line = np.flip(spiral(center,d[i],k_full,start_full,layer_width,resolution,SKIN)[1],0)
    lines_list_3.append(line)

lines_list = [lines_list_1,lines_list_2]

%matplotlib notebook
plot_lines_list(lines_list)

g_code_strings = lines_list_to_gcode_strings(lines_list,gcode_para)
# dumping:
#g_code_strings.insert(0,'G0 X-200.0000 Y-200.0000 Z15.0000 F1500\nG1 F0.4624\nG1 E0:0:0:0 F0.4624\nG1 E0:9:9:0 F0.4624\nG4 P6000\nG0 F1800')

g_code_strings.insert(0,'G0 X-200.0000 Y-200.0000 Z15.0000 F1500\nG1 F{0:.3f}\nG1 E0:0:0:0\nG1 E0:9:9:0\nG4 P10000\nG0 F1800\n'.format(gcode_para['feedrate_extrude']))

# only skin:
# g_code_strings.insert(0,'G0 X-200.0000 Y-200.0000 Z15.0000 F1500\nG1 F0.4624\nG1 E0:0:0:0 F0.4624\nG1 E0:8:8:0 F0.4624\nG4 P6000\nG0 F1800')



# initialization
init = initilize_parameters(4,is_absolute=True,is_mm=True,feedrate = 1500 )
g_code_strings.insert(0,init)


s = ''.join(g_code_strings)