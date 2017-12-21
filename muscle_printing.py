from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm

# define constant
CONDUCTIVE = 0
MUSCLE = 1
SKIN = 2
NEXT_LAYER = 3
TOOL = {}
# color mapping for plotting:
COLORS = {}
COLORS[CONDUCTIVE] = 'b'
COLORS[MUSCLE] = 'r'
COLORS[SKIN] = 'g'

def ExtruderRatio(d=1.6, D=15.778, applied_percentage=1):
    """A function that calculates the extruder ratio. applied_ratio
    = (D/d)^2*applied_percentage
    D: mm, diameter of the cylinder
    d: mm, diameter of the extruder
    applied_percentage: A ratio factor that controls the extruding ratio."""
    ratio = d ** 2 / (D ** 2) # extruder ratio
    #applied_percentage: applied percentage for extruding
    applied_ratio = ratio * applied_percentage
    return applied_ratio

def initilize_parameters(tool_num, is_absolute=True, is_mm=True, feedrate=1500):
    """A function that intializes the tool, coordinate frame, and unit parameters by taking
    them as imputs to the function, respectively. is_absolute will choose absolute coodinate
    frame is given any value other than zero. Choose zero for realtive coordinates. is_mm will
    set units to millimeters if input is any value other than zero. Choose zero for units in
    inches."""
    t = 'T{0}'.format(tool_num)
    if is_absolute:
        frame = 'G90'
    else:
        frame = 'G91'
    if is_mm:
        unit = 'G21'
    else:
        unit = 'G20'
    return '{0}\n{1}\n{2}\nG0 F{3}\nM83\n'.format(t, frame, unit, feedrate)

def ToolPath(path, ratio=1, a=0.5):
    """generate a list given [[x0,y0,z0],[x1,y1,z1]...]"""
    b = 1 - a
    n = len(path)
    e_path = np.zeros(n)
    e_path[1:] = norm(path[1:, :2] - path[0:-1, :2], axis=1) * ratio
    return ''.join(['G1 X{0:0.4f} Y{1:0.4f} Z{2:0.4f} E{3:0.4f}:{4:.4f}\n'.\
        format(path[i, 0], path[i, 1], path[i, 2], e_path[i] * a, e_path[i] * b) for i in range(n)])


def star_design(layers, delta_z, z_start, scale):
    """function that will output an array of values that are coodinates of a star. 
    input paramter is the desired number of layers, the deisred change in z height per layer,
    and the starting z height""" 
    prime = np.array([[0,0,z_start],[0,scale * -2,z_start],[0,0,z_start],[0,scale * -2,z_start],[0,0,z_start]])
    A = np.vstack([np.array([[scale * 0,scale * 0,(z_start) + k * delta_z],[scale * 5,scale * 6,(z_start) + k * delta_z],[scale * 10,scale * 0,(z_start) + k * delta_z],[scale * 0,scale * 4,(z_start) + k * delta_z],[scale * 10,scale * 4,(z_start) + k * delta_z],[scale * 0,scale * 0,(z_start) + k * delta_z]]) 
    for k in range(0,layers + 1)])
    star = np.vstack([prime,A]) 
    return star
def single_relative_square(l,d):
    return [(0,l,0),(l,0,0),(0,-l - d,0),(-l - d,0,0)]
def square(start,l,d,n,up=False):
    """output array of path for a multilayer square
    start is the starting point
    """
    multi_relative_square_list = [(0,0,0)]
    for k in range(n):
        multi_relative_square_list.extend(single_relative_square(l + d * 2 * k,d))
    multi_square_list = np.cumsum(multi_relative_square_list,axis=0) + start
    if up:
        multi_square_list = np.vstack((multi_square_list,multi_square_list[-1,:] + [0,0,d]))
    return multi_square_list,l + d * 2 * n

     

def clear_mixer(ratio,
                print_location,
                previous_extruders,
                next_extruders,
                dump_location=[-100,-200,33],
                retract_distance=0,
                dump_distance=12,
                prim_distance=0,
                feedrate_move=1000,
                feedrate_extrude=1,
                feedrate_quickmove=1500,
                utility='normal'):
    """A funtion that moves the extruder head to a specified location to empty out mixer
    a new materail combination. previous_extruders are the current tools being used in 1x4 array where
    1 is active and 0 is inactive. next_extruders are the tools to be used in the next phase 
    in 1x4 array where 1 is active and 0 is inactive.""" 
    ext_tool = dump_distance * np.array(next_extruders)
    ret_tool = retract_distance * np.array(previous_extruders) * ratio
    prim_tool = prim_distance * np.array(next_extruders) * ratio
    if 'normal' in utility:
        # retract->goto dump position->dump material->negative
        # priming(retract)->go to print position->priming->wait
        return ''.join(['G0 F{0:d}\n'.format(feedrate_quickmove),
            'G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(dump_location[0],dump_location[1],dump_location[2]),
            'G1 F{0:.4f}\n'.format(feedrate_extrude),
            'G1 E{0:0.4f}:{1:0.4f}:{2:0.4f}:{3:0.4f}\n'.format(ext_tool[0],ext_tool[1],ext_tool[2],ext_tool[3]),
            'G4 P{0:d}\n'.format(10000),    
            'G0 F{0:d}\n'.format(feedrate_quickmove),
            'G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(print_location[0],print_location[1],print_location[2]),
            'G1 F{0:d}\n'.format(feedrate_move),
            'G4 P{0:d}\n'.format(200)])#wait for 200ms
    elif 'last' in utility:# retract->goto dump position->dump material
        return ''.join(['G0 F{0:d}\n'.format(feedrate_quickmove),
            'G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(dump_location[0],dump_location[1],dump_location[2]),
            'G1 F{0:.4f}\n'.format(feedrate_extrude),
            'G1 E{0:0.4f}:{1:0.4f}:{2:0.4f}:{3:0.4f}\n'.format(ext_tool[0],ext_tool[1],ext_tool[2],ext_tool[3])])#wait for 500ms
    elif 'init' in utility:
        return ''.join(['G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(dump_location[0],dump_location[1],dump_location[2]),
            'G1 F{0:.4f}\n'.format(feedrate_extrude),
            'G1 E{0:0.4f}:{1:0.4f}:{2:0.4f}:{3:0.4f}\n'.format(ext_tool[0],ext_tool[1],ext_tool[2],ext_tool[3]),
            'G1 E{0:0.4f}:{1:0.4f}:{2:0.4f}:{3:0.4f}\n'.format(-prim_tool[0],-prim_tool[1],-prim_tool[2],-prim_tool[3]),
            'G1 F{0:d}\n'.format(feedrate_quickmove),
            'G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(print_location[0],print_location[1],print_location[2]),
            'G1 F{0:d}\n'.format(feedrate_extrude),
            'G1 E{0:0.4f}:{1:0.4f}:{2:0.4f}:{3:0.4f}\n'.format(prim_tool[0],prim_tool[1],prim_tool[2],prim_tool[3]),
            'G4 P{0:d}\n'.format(10000),
            'G1 F{0:d}\n'.format(feedrate_quickmove),
            'G0 X{0:0.4f} Y{1:0.4f} Z{2:0.4f}\n'.format(print_location[0],print_location[1],print_location[2]),
            'G1 F{0:d}\n'.format(feedrate_move),
            'G4 P{0:d}\n'.format(200)])

def ToolPath4(path,ratio,mix_ratio,feedrate_move=1000,feedrate_quickmove=1500,**kwargs):
    """generate a list of G-code given [[x0,y0,z0],[x1,y1,z1]...]
    for example ratio = 1, mix_ration = [0,0.5,0.5,0]
    optional parameters:
        is_g0 is a numpy bool array specifying wether the point in the path use fast move (G0)
        for example, is_g0 = np.array([True,False,False,False,....])
    """
    if np.shape(path)[0] < 2:# if path points are less than 2
        return ''
    n = len(path)
    e_path = np.zeros(n)
    gcode_list = [None] * n
    if 'is_g0' in kwargs:
        is_g0 = kwargs['is_g0']
    else:
        is_g0 = None
    e_path[1:] = norm(path[1:,:2] - path[0:-1,:2],axis=1) * ratio
    if 'start_point' in kwargs:# if it is extruding continously from a previous start_point
        start_point = kwargs['start_point']
        e_path[0] = norm(path[0,:2] - start_point[:2]) * ratio
        gcode_list[0] = 'G1 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} E{3:0.6f}:{4:.6f}:{5:.6f}:{6:.6f} F{7:d}\n'.\
                    format(path[0,0],path[0,1],path[0,2],e_path[0] * mix_ratio[0],e_path[0] * mix_ratio[1],e_path[0] * mix_ratio[2],e_path[0] * mix_ratio[3],feedrate_move)
    else:
        gcode_list[0] = 'G1 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} F{3:d}\n'.\
                    format(path[0,0],path[0,1],path[0,2],feedrate_quickmove)
    if is_g0 is not None:
        for i in range(1,n):
            if is_g0[i]:
                # gcode_list[i] = 'G1 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} F{3:d}\n'.\
                #         format(path[i,0],path[i,1],path[i,2],feedrate_quickmove)
                gcode_list[i] = 'G0 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} E{3:0.6f}:{4:.6f}:{5:.6f}:{6:.6f} F{7:d}\n'.\
                        format(path[i,0],path[i,1],path[i,2],e_path[i] * mix_ratio[0],e_path[i] * mix_ratio[1],e_path[i] * mix_ratio[2],e_path[i] * mix_ratio[3],feedrate_quickmove)
            else:
                gcode_list[i] = 'G1 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} E{3:0.6f}:{4:.6f}:{5:.6f}:{6:.6f} F{7:d}\n'.\
                        format(path[i,0],path[i,1],path[i,2],e_path[i] * mix_ratio[0],e_path[i] * mix_ratio[1],e_path[i] * mix_ratio[2],e_path[i] * mix_ratio[3],feedrate_move)
    else:
        for i in range(1,n):
            gcode_list[i] = 'G1 X{0:0.6f} Y{1:0.6f} Z{2:0.6f} E{3:0.6f}:{4:.6f}:{5:.6f}:{6:.6f} F{7:d}\n'.\
                            format(path[i,0],path[i,1],path[i,2],e_path[i] * mix_ratio[0],e_path[i] * mix_ratio[1],e_path[i] * mix_ratio[2],e_path[i] * mix_ratio[3],feedrate_move)
    return ''.join(gcode_list) 

def combine_lines(lines):
    """
    combine a list of line arrays to a single numpy array, transist from one array to another array using G0,
    return the combined lines--unified_lines 
    and a numpy array--is_g0 specifying wether the point in the path use fast move (G0)
    """        
    n_points = [len(points) for points in lines[1:]]
    unified_lines = np.vstack(lines[1:])
    g0_index = np.zeros(len(n_points),dtype=int)
    g0_index[1:] = np.cumsum(n_points)[:-1]
    is_g0 = np.zeros(len(unified_lines),dtype=bool)
    is_g0[g0_index] = True
    return unified_lines,is_g0

def calculate_tool_change_index(unified_lines,max_reuse_distance):
    """calculate the index when tool change happens in unified_lines"""
    # calculate tool_change_index
    reuse_distance = 0
    tool_change_index = 0
    distances = norm(unified_lines[1:,:2] - unified_lines[:-1,:2],axis=1)
    for i, distance in reversed(list(enumerate(distances))):
        reuse_distance+=distance
        if reuse_distance > max_reuse_distance:
            tool_change_index = i
            break
    return tool_change_index,reuse_distance

def lines_list_to_gcode_strings(lines_list,gcode_para):
    """get g_code_strings from lines_list,
    lines_list is in the format of 
    [[Material#,nx3_numpy_array,nx3_numpy_array,nx3_numpy_array...],
    [Material#,nx3_numpy_array,nx3_numpy_array,nx3_numpy_array...],
    [Material#,nx3_numpy_array,nx3_numpy_array,nx3_numpy_array...],
    .....
    ]
    """
    # locals().update(gcode_para) #convert dictionary entries into variables
    D,d,applied_percentage = gcode_para['D'],gcode_para['d'],gcode_para['applied_percentage']
    dump_location,TOOL = gcode_para['dump_location'],gcode_para['TOOL']
    feedrate_move,feedrate_quickmove, feedrate_extrude = gcode_para['feedrate_move'],gcode_para['feedrate_quickmove'],gcode_para['feedrate_extrude']
    total_volume_change,max_reuse_volume = gcode_para['total_volume_change'],gcode_para['max_reuse_volume']
    syringe_ratio = ExtruderRatio(d=d, D=D, applied_percentage=1) # define syring plunger to tip ratio
    
    # want to utilize some portion of material that is still inside the mixer
    # before dumping it to the waste area, thus we need to determin how much
    # remainng material we can use, define max_reuse_distance as the maximum
    # distance of syring tip travel that we can reuse
    syringe_cross_section_area = D * D / 4. * np.pi  # [mm2]
    total_distance = total_volume_change / \
        syringe_cross_section_area / syringe_ratio
    max_reuse_distance = max_reuse_volume / \
        syringe_cross_section_area / syringe_ratio

    g_code_strings = []
    len_lines_list = len(lines_list)
    for m in range(len_lines_list):
        tool_initial = lines_list[m][0]
        print(tool_initial)
        unified_lines, is_g0 = combine_lines(lines_list[m])
        # calculate tool_change_index
        tool_change_index, reuse_distance = calculate_tool_change_index(unified_lines, max_reuse_distance)
        tool_change_point = unified_lines[tool_change_index, :]
        if m < (len_lines_list - 1):
            tool_rest = lines_list[m + 1][0]
            if tool_initial != tool_rest:  # if there is a tool change
                # path for initial extrusion
                lines_initial = unified_lines[:tool_change_index, :]
                is_g0_initial = is_g0[:tool_change_index]
                # path for extrusion after tool change
                lines_rest = unified_lines[tool_change_index:, :]
                is_g0_rest = is_g0[tool_change_index:]
                g_code_strings.append(ToolPath4(lines_initial, is_g0=is_g0_initial, ratio=syringe_ratio, mix_ratio=TOOL[tool_initial],feedrate_move=feedrate_move))
                g_code_strings.append(ToolPath4(lines_rest, is_g0=is_g0_rest, ratio=syringe_ratio, mix_ratio=TOOL[tool_rest], start_point=tool_change_point,feedrate_move=feedrate_move))

                g_code_strings.append(clear_mixer(ratio=syringe_ratio,
                                                  print_location=lines_list[m+1][1][0, :],
                                                  previous_extruders=TOOL[tool_rest],
                                                  next_extruders=TOOL[tool_rest],
                                                  dump_location=dump_location,
                                                  dump_distance=(total_distance - reuse_distance) * syringe_ratio,
                                                  feedrate_extrude=feedrate_extrude))
            else:
                g_code_strings.append(ToolPath4(unified_lines, is_g0=is_g0, ratio=syringe_ratio, mix_ratio=TOOL[tool_initial],feedrate_move=feedrate_move))
        else:
            # last item in lines_list
            g_code_strings.append(ToolPath4(unified_lines, is_g0=is_g0, ratio=syringe_ratio, mix_ratio=TOOL[tool_initial],feedrate_move=feedrate_move))
            g_code_strings.append(clear_mixer(ratio=syringe_ratio,
                                              print_location=dump_location,
                                              dump_location=dump_location,
                                              previous_extruders=TOOL[tool_initial],
                                              next_extruders=TOOL[tool_initial], dump_distance=0, utility='last'))
    return g_code_strings


def plot_lines_list(lines_list):
    """plot given a lines_list"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for lines in lines_list:
        for k in range(1,len(lines)):
            ax.plot(lines[k][:,0],lines[k][:,1],lines[k][:,2],c=COLORS[lines[0]])
            try:
                ax.plot([lines[k][-1,0],lines[k + 1][0,0]],[lines[k][-1,1],lines[k + 1][0,1]],[lines[k][-1,2],lines[k + 1][0,2]],'m',linewidth=.5)
            except IndexError:
                #last lines
                pass
    plt.show()

def circle(center,r,z,d=None):
    n = 50
    line = np.empty((n,3))
    if d is not None:
        t = np.linspace(0,2 * np.pi - np.arcsin(d / 2. / r),n) - np.pi
    else:
        t = np.linspace(0,2 * np.pi,n) - np.pi / 2
    line[:,0] = center[0] + r * np.cos(t)
    line[:,1] = center[1] - r * np.sin(t)
    line[:,2] = z
    return line
