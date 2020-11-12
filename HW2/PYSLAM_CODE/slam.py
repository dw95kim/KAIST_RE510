import numpy as np
import csv

import g2o
from optimizer import PoseGraphOptimization
import icp

from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import math

information_constant = 3.9

def main():
    ############################################
    #                                          #
    #          DATA PROCESSING PART            #
    #                                          #
    ############################################
    
    # READ POSE DATA FROM CSV. posedata = [pose0,poes1,...]
    # pose0 = numpy matrix 4x4 (SE3)
    posedata = []
    with open('data/pose.csv', 'r') as csvfile:
        readcsv = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in readcsv:
            pose = []
            for i in range(len(row)):
                pose.append(float(row[i]))
            r = R.from_quat([pose[3],pose[4],pose[5],pose[6]]).as_dcm()
            t = np.array([[pose[0]],[pose[1]],[pose[2]]])
            RT = np.vstack((np.hstack((r,t)),np.array([0,0,0,1])))
            posedata.append(RT)

    # READ LIDAR DATA FROM CSV. lidardata = [lidar0,lidar1,...]
    # lidar0 = [[x0,x1,x2,x3,...],[y0,y1,y2,y3,...],[0,0,0,0,...],[1,1,1,1,...]] (2D in 3D! : Numpy matrix 4xn)
    lidardata = []
    with open('data/lidar.csv', 'r') as csvfile:
        readcsv = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in readcsv:
            x = []
            y = []
            pointNum = len(row) / 2
            for i in range(pointNum):
                x.append(float(row[2 * i])+0.30)
                y.append(row[2 * i + 1])
            lidardata.append(np.array([x, y, np.zeros(len(x)),np.ones(len(x))]).astype(np.float32))
            
            

    ############################################
    #                                          #
    #      SLAM MAIN PART (ASSIGNMENTS)        #
    #                                          #
    ############################################

    # nodes = [node0, node1, ...]
    # node0 = [pose0,lidar0,posediff with node before]
    nodes = [[posedata[0],lidardata[0],np.eye(4)]]
    
    for i in range(1,len(posedata)):
    ####################################################
    # ASSIGNMENTS 1 : CALCULATE POSE DIFF & MAKE NODE  #
    #                                                  #
    # Create nodes at distance or angle intervals.     #
    #                                                  #
    # POSEDIFF : 4x4 numpy matrix of pose difference   #
    # POSE BEFORE : LAST NODE'S POSE ( nodes[-1][0] )  #
    # POSE NOW    : CURRENT POSE     ( posedata[i]  )  #
    ####################################################

        np_pose_before = np.array(nodes[-1][0])
        np_pose_now = np.array(posedata[i])

        rot_dif = np.dot(np_pose_now[0:3, 0:3], np_pose_before[0:3, 0:3].T)
        pos_dif = np_pose_now[0:3, 3] - np_pose_before[0:3, 3]
        dis_dif = math.sqrt(pos_dif[0]**2 + pos_dif[1]**2 + pos_dif[2]**2)

        poseDiff = np.zeros([4,4])
        poseDiff[0:3, 0:3] = rot_dif #Calculate pose diff in 4x4 matrix
        poseDiff[0:3, 3] = pos_dif
        poseDiff[3, 3] = 1

        distDiff =  dis_dif #Calculate euclidean distance between two node (using posediff)

        yawDiff  = R.from_dcm(poseDiff[0:3,0:3]).as_euler('zyx')[0] # Robot is in 2D in this lab, so just use Yaw angle
        # If enough distance(0.1[m]) or angle(30[deg]) difference, create node
        if (distDiff > 0.1 or abs(yawDiff)/3.141592*180 > 30):
            nodes.append([posedata[i],lidardata[i],poseDiff])


    #############################################################################
    # ASSIGNMENTS 2 : ADD VERTEX AND ODOMETRY EDGE                              #
    #                                                                           #
    # Add vertex of each node and add odometry constraint edge                  #
    # to optimizer.                                                             #
    # FUNCTION1: optimizer.add_vertex(index, g2o.Isometry3d(pose),fixed)        #
    #            index : int, pose : numpy 4x4 mat, fixed : boolean             #
    # FUNCTION2: optimizer.add_edge([src,dst], g2o.Isometry3d(diff),information)#
    #            src: source index, dst: destination index                      #
    #            diff: diff mat (4x4) between source and destination            #   
    #            information: 6x6 numpy matrix of information                   #
    # TIP : You can set simple identity matrix for information.                 #
    #       It will work but not accurate.                                      #
    #############################################################################
    
    # Define optimizer
    optimizer = PoseGraphOptimization()

    #Add first node as a fixed vertex. (True = fixed, False = non-fixed)
    # nodes = [[pose0,lidar0,posediff0],[pose1,lidar1,posediff1], ...]
    # pose0 = numpy matrix 4x4 (SE3)
    optimizer.add_vertex(0, g2o.Isometry3d(nodes[0][0]),True)
    
    for i in range(1,len(nodes)):
        information_matrix = information_constant * np.eye(6)
        
        if (i < len(nodes)/3):
            optimizer.add_vertex(i, g2o.Isometry3d(nodes[i][0]),True)
        else:
            optimizer.add_vertex(i, g2o.Isometry3d(nodes[i][0]),False)
        
        if(R.from_dcm(nodes[i][2][0:3, 0:3]).as_euler('zyx')[0] > 45): 
            information_matrix[:,5] += 0.1
            information_matrix[5] += 0.1
            information_matrix[5, 5] -= 0.1

        if(np.linalg.norm(poseDiff[0:3, 3]) > 0.15):
            information_matrix[0] += 0.1
            information_matrix[:,0] += 0.1
            information_matrix[0, 0] -= 0.1

        optimizer.add_edge([i-1, i], g2o.Isometry3d(nodes[i][2]),
                           information=information_matrix)

    #############################################################################
    #                                                                           #
    # VISUALIZE LIDAR POINTS INTO GLOBAL. (BEFORE OPTIMIZATION)                 #
    #                                                                           #
    #############################################################################

    for i in range(0,len(nodes)):
        LiDAR = nodes[i][1][0:4]
        LiDAR = np.dot(nodes[i][0], LiDAR)
        plt.scatter(LiDAR[0], LiDAR[1], c='b', marker='o',s=0.2)
    plt.show()

    optimizer.save_g2o('beforeSLAM.g2o')


    for full_scale_iter in range(2):
        #############################################################################
        # ASSIGNMENTS 3 : FIND LOOP CLOSURE                                         #
        #                                                                           #
        # Simply, you can put all pair in the matching pair, it will be work.       #
        # How can you reduce pairs for less computation? (option)                   #
        #                                                                           #
        #############################################################################  

        # matchingPair = [[src0,dst0],[src1,dst2]...]

        matchingPair = []
        interval = 30

        for i in range(len(nodes)-interval):
            for j in range(i+interval, len(nodes)):
                src_pose = np.array(nodes[i][0])[0:3, 3]
                dst_pose = np.array(nodes[j][0])[0:3, 3]

                diff_src_dst_pose = dst_pose - src_pose
                src_dst_distance = math.sqrt(diff_src_dst_pose[0]**2 + diff_src_dst_pose[1]**2 + diff_src_dst_pose[2]**2)
                
                if(src_dst_distance < 0.5):
                    matchingPair.append([i, j])


        
        #####################################################################################
        # ASSIGNMENTS 4 : MATCHING PAIRS, OPTIMIZE!                                         #
        #                                                                                   #
        # FUNCTION1: T,D,I = icp.icp(dstPoints,srcPoints,tolerance,max_iterations)          #
        #            T : Transformation from src node to dst node (3x3 matrix:2D matching!) #
        #            D : Distances between corresponding points in srcPoints and dstPoints  #
        #            I : Total iterations           					                    #
        #											                                        #
        # Apply initial translation to dst point cloud! if not, icp will inaccurate         #
        #											                                        #
        #####################################################################################         

        # Drawing ROBOT position X, Y
        # for i in range(0,len(nodes)):
        #     pose = nodes[i][0][0:3, 3]
        #     plt.scatter(pose[0], pose[1], c='b', marker='o',s=0.2)
        # plt.show()


        opt_count = 1
        add_edge_count = 0

        for src,dst in matchingPair:
            srcLiDAR = nodes[src][1][0:4]
            dstLiDAR = nodes[dst][1][0:4]
            #GET SOURCE NODE POSITION (FROM VERTEX). srcRT = 4x4 matrix
            rt = optimizer.get_pose(src)
            srcRT = np.insert(rt.R, 3, rt.t, axis=1)
            srcRT = np.insert(srcRT, 3, [0, 0, 0, 1], axis=0)
            #GET DESTINATION NODE POSITION (FROM VERTEX). dstRT = 4x4 matrix
            rt = optimizer.get_pose(dst)
            dstRT = np.insert(rt.R, 3, rt.t, axis=1)
            dstRT = np.insert(dstRT, 3, [0, 0, 0, 1], axis=0)
            
            #PROCESS POINT CLOUD!
            srcPoint = srcLiDAR

            src_pose = np.array(nodes[src][0])[0:3, 3]
            dst_pose = np.array(nodes[dst][0])[0:3, 3]
            diff_src_dst_pose = dst_pose - src_pose

            m = dstLiDAR.shape[1]
            offset_pose = np.zeros([4, m])
            offset_pose[0] = diff_src_dst_pose[0]
            offset_pose[1] = diff_src_dst_pose[1]

            dstPoint = dstLiDAR + offset_pose #??
            # dstPoint = dstLiDAR

            #DON'T HAVE TO CHANGE MATCHING FUNCTION
            T, distances, iterations = icp.icp(dstPoint[0:2].T,srcPoint[0:2].T,
                                                tolerance=0.000001,max_iterations=100000)
            #### MAKE 3x3 matrix into 4x4 matrix ####
            T = np.insert(T, 2, [0, 0, 0], axis=1)
            T = np.insert(T, 2, [0, 0, 1, 0], axis=0)

            # print(sum(distances)/len(distances))

            # DRAWING FUNCTION FOR CHECKING ICP DONE WELL : Source blue, Dest green, Dest after ICP red
            # dist_condition = distances[distances<0.03]
            # if(len(dist_condition) > len(distances)/1.5 and sum(distances)/len(distances) < 0.05):
            #     dstTrans = np.dot(T, dstPoint)
            #     plt.scatter(dstPoint[0], dstPoint[1], c='g', marker='o',s=0.2)
            #     plt.scatter(srcPoint[0], srcPoint[1], c='b', marker='o',s=0.2)
            #     plt.scatter(dstTrans[0], dstTrans[1], c='r', marker='o',s=0.2)
            #     plt.show()
            
            np_pose_before = np.array(nodes[src][0])
            np_pose_now = np.dot(T, nodes[dst][0])

            rot_dif = np.dot(np_pose_now[0:3, 0:3], np_pose_before[0:3, 0:3].T)
            pos_dif = np_pose_now[0:3, 3] - np_pose_before[0:3, 3]

            poseDiff = np.zeros([4,4])
            poseDiff[0:3, 0:3] = rot_dif #Calculate pose diff in 4x4 matrix
            poseDiff[0:3, 3] = pos_dif
            poseDiff[3, 3] = 1

            yawDiff  = R.from_dcm(poseDiff[0:3,0:3]).as_euler('zyx')[0]

            # if(sum(distances)/len(distances) < 0.075):	# ADD CONDITION OF MATCHING SUCCESS (ex: mean of distances less then 0.05 [m])
            dist_condition = distances[distances<0.07]
            if (len(dist_condition) > len(distances)/2 and sum(distances)/len(distances) < 0.075):
            # if(len(dist_condition) > len(distances)/1.5 and sum(distances)/len(distances) < 0.05 and yawDiff/3.141592*180 < 20):
                information_matrix = information_constant * np.eye(6)
                
                if(R.from_dcm(poseDiff[0:3, 0:3]).as_euler('zyx')[0] > 45): 
                    information_matrix[:,5] += 0.1
                    information_matrix[5] += 0.1
                    information_matrix[5, 5] -= 0.1

                if(np.linalg.norm(poseDiff[0:3, 3]) > 0.75):
                    information_matrix[0] += 0.1
                    information_matrix[:,0] += 0.1
                    information_matrix[0, 0] -= 0.1

                add_edge_count += 1

                optimizer.add_edge([src,dst], g2o.Isometry3d(poseDiff), #??
                            information=information_matrix)
                optimizer.optimize()
            
            if(opt_count % 100 == 0):
                print(str(opt_count) + '/' + str(len(matchingPair)) + '/' + str(full_scale_iter))
            opt_count += 1
        

    #############################################################################
    #                                                                           #
    # VISUALIZE LIDAR POINTS INTO GLOBAL (AFTER OPTIMIZATION)                   #
    #                                                                           #
    #############################################################################

    print(add_edge_count)
    print(len(nodes))

    for i in range(0,len(nodes)):
        dstLiDAR = nodes[i][1][0:4]
        rt = optimizer.get_pose(i)
        T = np.insert(rt.R, 3, rt.t, axis=1)
        T = np.insert(T, 3, [0, 0, 0, 1], axis=0)
        dstLiDAR = np.dot(T, dstLiDAR)
        plt.scatter(dstLiDAR[0], dstLiDAR[1], c='b', marker='o',s=0.2)

        # if (i < 20):
        #     plt.scatter(dstLiDAR[0], dstLiDAR[1], c='b', marker='o',s=0.2)
        # elif (i < 40):
        #     plt.scatter(dstLiDAR[0], dstLiDAR[1], c='r', marker='o',s=0.2)
        # elif (i < 60):
        #     plt.scatter(dstLiDAR[0], dstLiDAR[1], c='g', marker='o',s=0.2)
        # elif (i < 80):
        #     plt.scatter(dstLiDAR[0], dstLiDAR[1], c='y', marker='o',s=0.2)
        # elif (i < 100):
        #     plt.scatter(dstLiDAR[0], dstLiDAR[1], c='b', marker='o',s=0.2)
        
    plt.show()
    
    optimizer.save_g2o('afterSLAM.g2o')



if __name__ == '__main__':
    main()
