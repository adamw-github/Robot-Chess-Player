#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
import time

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    #board_setup = all pieces on the board except the kings and rooks (6 total pieces to be setup by baxter)
    #board is setup from column 1 to 8, row H to A(baxter is sitting on white side)
    board_setup =  ['***K****', '****P*P*', '********', '********', '********', '********', '********', 'r**k***r']

    piece_positionmap = dict()
    piece_names = []
    rownames = 'HGFEDCBA'
    for row, each in enumerate(board_setup):
        # print row
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            #map each position to the correcct pose, using official chess notation
            piece_positionmap[rownames[col]+str(row+1)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference
            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))

    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files

    #Wait for Baxter to move before spawning anything
    ready_to_spawn = False
    rospy.set_param('ready_to_spawn', ready_to_spawn)
    while not ready_to_spawn:
        print("Waiting for Baxter to move to start before spawning...")
        ready_to_spawn = rospy.get_param('ready_to_spawn')
        time.sleep(1)
    #ready_to_spawn == True

    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    #Location to spawn piece at the side of the board
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    pose = deepcopy(board_pose)
    pose.position.x = board_pose.position.x + frame_dist + origin_piece + 4 * (2 * origin_piece)
    pose.position.y = board_pose.position.y -0.15 + frame_dist + origin_piece + 4 * (2 * origin_piece)
    pose.position.z += 0.018

    for piece in piece_names:
        if piece[0] in list_pieces:
            rospy.set_param('current_piece', piece)
            print srv_call(piece, pieces_xml[piece[0]], "/", pose, "world")
            print("Spawning " + piece)
            ready_to_spawn = False
            rospy.set_param('ready_to_spawn', ready_to_spawn) #not ready to spawn until piece has been moved by move_pieces.py
            try:
                #Wait for baxter to place the last piece before spawning new one
                while not ready_to_spawn:
                    print("waiting for baxter to place last piece...")
                    ready_to_spawn = rospy.get_param('ready_to_spawn')
                    time.sleep(1)
            except KeyboardInterrupt:
                break
            