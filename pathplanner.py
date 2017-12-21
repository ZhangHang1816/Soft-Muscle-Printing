import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm
from point_in_polygon import wn_PnPoly

def rotation_2d(t):
    return [[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]]


def polygon_path(lines,d, angle = 0,plotting = False):
    r = d/2
    
    if angle!=0:
        lines = np.dot(lines,rotation_2d(angle))
    
    #####################################################
    # sample from x and get intercetion points
    num_lines = len(lines)-1
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # for k in range(num_lines):
    #     ax.plot(lines[k:k+2,0],lines[k:k+2,1])
    # plt.grid();plt.show()

    x_max,y_max = lines.max(axis=0)
    x_min,y_min = lines.min(axis=0)
    x = np.arange(x_min+r,x_max,d) # x value of the sampling points
    num_x = len(x)
    y_list = [set() for k in range(num_x)]

    for k in range(num_lines): 
        if lines[k,0] != lines[k+1,0]: # not vertical line
            if lines[k,0] > lines[k+1,0]:
                x_lower,x_upper = lines[k+1,0],lines[k,0]
            elif lines[k,0] < lines[k+1,0]:
                x_lower,x_upper = lines[k,0],lines[k+1,0]
    #         print(x_lower,x_upper)
            for i,intercepted in enumerate((x>=x_lower)&(x<=x_upper)):
                if intercepted:
                    x_intercept = x[i]
                    y_intercept = (x_intercept-lines[k,0])/(lines[k+1,0]-lines[k,0])*(lines[k+1,1]-lines[k,1])+lines[k,1]
    #                 print(x_intercept,y_intercept)
    #                 y_list[i].append(y_intercept)
                    y_list[i].add(y_intercept)

    # remove repeating point, and sort in rising order    
    for k in range(num_x):
        y_list[k]= list(y_list[k])
        y_list[k].sort()


    # # plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # for k,y in enumerate(y_list):
    #     ax.plot([x[k]]*len(y),y,'*')
    # for k in range(num_lines):
    #     ax.plot(lines[k:k+2,0],lines[k:k+2,1])
    # plt.show()
    # # print(np.array(y_list))

    ########################

    # if the sampling point_x coinside with the endpoint of lines
    # need to figure out which path lies inside the polygon
    line_x_set=set(lines[:,0])
    for k in range(num_x):
    # for k in [-2]:
        if x[k] in line_x_set:
            y_list_k = []
            for i in range(len(y_list[k])-1):
                y_1 = y_list[k][i]
                y_2 = y_list[k][i+1]
                y_m = (y_1+y_2)/2 # middle point

                if wn_PnPoly([x[k],y_m], lines)!=0:
                    y_list_k.extend([y_1,y_2])

    # #             y_m = y_1
    # #             print(y_m)
    #             y_intercept_set = set()
    #             m=0
    #             for j in range(num_lines):
    #                 x_lower = min(lines[j,0],lines[j+1,0])
    #                 x_upper = max(lines[j,0],lines[j+1,0])
    #                 if x_lower!=x_upper: # don't count intercetion with vertical lines
    #                     if x_lower<=x[k]<=x_upper:
    #                         y_intercept = (x[k]-lines[j,0])/(lines[j+1,0]-lines[j,0])*(lines[j+1,1]-lines[j,1])+lines[j,1]
    #                         if y_m<y_intercept:
    #     #                         print(x_lower,x_upper,y_intercept)
    #                             y_intercept_set.add(y_intercept)
    #                             m+=1
    #             if m%2:
    # #             if len(y_intercept_set)%2:
    #                 y_list_k.extend([y_1,y_2])
            y_list[k] = y_list_k
    # # plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # for k,y in enumerate(y_list):
    #     ax.plot([x[k]]*len(y),y,'*')
    # for k in range(num_lines):
    #     ax.plot(lines[k:k+2,0],lines[k:k+2,1])
    # plt.show()
    # print(np.array(y_list))

    #######################################################
    # offset the path to accout for the radius of extrusion
    for k in range(num_x):    
        for i in range(len(y_list[k])//2):
                y_list[k][2*i]+=r
                y_list[k][2*i+1]-=r
                if y_list[k][2*i]>=y_list[k][2*i+1]:
                    # get average
                    y_list[k][2*i] = (y_list[k][2*i]+y_list[k][2*i+1])/2
                    y_list[k][2*i+1] = y_list[k][2*i]
    #     print(x[k],y_list[k])


    # ## plotting
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # # ax.plot(lines[:,0],lines[:,1])
    # for k in range(num_lines):
    #     ax.plot(lines[k:k+2,0],lines[k:k+2,1])
    # for k in range(num_x):
    #     for i in range(len(y_list[k])//2):
    #         y_1 = y_list[k][i*2]
    #         y_2 = y_list[k][i*2+1]
    #         if y_1<y_2:
    #             ax.plot([x[k]]*2,[y_1,y_2],'.-')
    #         elif y_1==y_2:
    #             ax.plot(x[k],y_1,'*')       
    # # print(np.array(y_list))
    # plt.grid();plt.show()
    ########################################################

    # add path to path_list
    num_interction = len(y_list[0])
    path_list = []
    path = []
    done = np.zeros(num_x,dtype=int)
    go_up = True
    k=0
    while np.sum(done)<num_x:
            if len(y_list[k]):
                if len(y_list[k])==num_interction:
                    y_1 = y_list[k][0]
                    y_2 = y_list[k][1]
                    
                    try:
                        if min(abs(path[-1][1]-y_1),abs(path[-1][1]-y_2))>2*d:
                            # if the two points are too far away
                            if path:# if path is not empty
                                path_list.append(path)
                                path = []
#                                 num_interction = len(y_list[k])
                    
                    except IndexError:
                        pass
                    
                    if y_1>=y_2:
                        path.extend([[x[k],(y_1+y_2)/2]])
    #                     print([[x[k],(y_1+y_2)/2]])
                    elif go_up:
                        path.extend([[x[k],y_1],[x[k],y_2]])
                    else:
                        path.extend([[x[k],y_2],[x[k],y_1]])
                    y_list[k].pop(0)
                    y_list[k].pop(0)
                    k+=1

                    go_up = not go_up
                else:
                    if path:# if path is not empty
                        path_list.append(path)
                        path = []
                    num_interction = len(y_list[k])
            else:
                done[k]=1
                if path:# if path is not empty
                    path_list.append(path)
                    path = []
                num_interction = len(y_list[k])
                k+=1

    #             print(done)
            if k==num_x:
                k=0
                if path:# if path is not empty
                    path_list.append(path)
                    path = []
    path_list = [np.array(path)for path in path_list]

#     fig = plt.figure()
#     ax = fig.add_subplot(111)
#     for k in range(num_lines):
#         ax.plot(lines[k:k+2,0],lines[k:k+2,1])
#     for path in path_list:
#         ax.plot(path[:,0],path[:,1])
#     plt.grid();plt.show()

    ###################################################################
    # combine path
    k = 0
    while k<len(path_list):
        i = k+1
        while i< len(path_list):
            if norm(path_list[k][-1]-path_list[i][0])<=d*2:
                path_list[k]=np.vstack([path_list[k],path_list.pop(i)])
                i=0
            if norm(path_list[i][-1]-path_list[k][0])<=d*2:
                path_list[k]=np.vstack([path_list.pop(i),path_list[k]])
                i=0
            else:
                i+=1
        k+=1
    ######################
    if angle!=0:
        rotation_mat_inverse = rotation_2d(-angle)
        lines = np.dot(lines,rotation_mat_inverse)
        for k,path in enumerate(path_list):
            path_list[k] = np.dot(path,rotation_mat_inverse)       
    
    if plotting:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    #     for k in range(num_lines):
    #         ax.plot(lines[k:k+2,0],lines[k:k+2,1])
        ax.plot(lines[:,0],lines[:,1])
        for path in path_list:
            ax.plot(path[:,0],path[:,1])
            plt.axis('equal')
        plt.grid();plt.show()        
    return path_list
	
def offset_intersection_point(v1,v2,p,d,left=True):
    """cacluate the intersection point of the offset lines,
    v1 is the previous line vector, v2 is the current line vector
    p is the current point, d is the offset distance
    """
    phi = np.pi-np.arctan2(v2[1],v2[0])+np.arctan2(v1[1],v1[0])
#     print(phi*180/np.pi)
    if phi>=np.pi:
        vm = v1-v2
        theta = phi/2
    else:
        vm = v2-v1
        theta =np.pi-phi/2
    if not left:
        vm = -vm
#     print(theta*180/np.pi)
    vm = vm/norm(vm)
#     print(vm)
    pm = p+d/np.sin(theta)*vm
    return pm
# #     example:
# p0 = np.array([2,1])
# p = p0+np.array([1,0])
# p2 = p + np.array([1,-1])
# v1 = (p -p0)
# v1 = v1/norm(v1)
# v2 = p2-p
# v2 = v2/norm(v2)
# d = 1
# pm = offset_intersection_point(v1,v2,p,d)
# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.plot([p0[0],p[0]],[p0[1],p[1]])
# ax.plot([p[0],p2[0]],[p[1],p2[1]])
# ax.plot([p[0],pm[0]],[p[1],pm[1]])
# plt.grid()
# plt.axis('equal')
# plt.show()  


def offset_lines(lines,d,left = True):
    """
    Compute the offset lines from the lines, given d as offset distance
    """
    v_list = np.zeros_like(lines,dtype=float)
    for k in range(len(lines)):
        try:
            p1 = lines[k] # first point
            p2 = lines[k+1] # second point
            v = p2-p1
            v_list[k,:] = v/norm(v)
        except IndexError:
    #         print('reaching last point')
            v_list[k,:] = v_list[k-1,:]
    lines_offset = np.zeros_like(lines,dtype=float)
    if lines[0,0]==lines[-1,0] and lines[0,1]==lines[-1,1]:
        v1 = v_list[-1]
        v2 = v_list[0]
        lines_offset[0] = offset_intersection_point(v1,v2,lines[0],d,left=left)
        lines_offset[-1] = lines_offset[0]
    else:
        for k in [0,len(lines)-1]:
            n = np.array([-v_list[k,1],v_list[k,0]])
            lines_offset[k] = lines[k]+n/norm(n)*d
    for k in range(1,len(lines)-1):
        v1 = v_list[k-1]
        v2 = v_list[k]
        lines_offset[k] = offset_intersection_point(v1,v2,lines[k],d,left=left)
    return lines_offset