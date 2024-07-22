#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander
import tf
import time

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        time.sleep(1)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def move_piece(self, piece, position, pnp, big=False):
        # Move to the desired starting angles
        pnp.move_to_start(starting_pose)

        listener = tf.TransformListener()
        trans, rot = None, None
        #end while when both trans and rot are not none
        while not (trans and rot):
            try:
                #if the piece is found, returns transformation and quaternion from current location to piece
                (trans,rot) = listener.lookupTransform('/world', '/'+piece, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        #position of target piece
        target_pose = Pose(
            position = Point(x=trans[0],y=trans[1], z=trans[2]-0.01),
            orientation = overhead_orientation
        )

        #position of target ending position
        ending_pose = Pose(
            position=Point(x=piece_positionmap[position][0], y=piece_positionmap[position][1], z=piece_positionmap[position][2]+0.01),
            orientation=overhead_orientation
        )

        #pick up target piece
        print("Picking piece; " + str(piece) + "...")
        pnp.pick(target_pose)
        #lifts arm higher to avoid sweeping pieces off the board when doing big moves
        if big:
            pnp._servo_to_pose(Pose(
            position = Point(x=trans[0],y=trans[1], z=trans[2]+0.25),
            orientation = overhead_orientation
        ))
        #place piece at target location
        print("Placing piece: " + str(piece) + "...")
        pnp.place(ending_pose)

#Wait for chess piece to spawn, and ready_to_spawn get changed to False
def wait_for_spawn(ready_to_spawn):
    while ready_to_spawn:
        print("Waiting to spawn next piece...")
        ready_to_spawn = rospy.get_param('ready_to_spawn')
        time.sleep(0.5)
    return ready_to_spawn


def main():
    global overhead_orientation
    global starting_pose
    global piece_positionmap
    #initialise constants
    limb = 'left'
    hover_distance = 0.18  # meters
    piece_positionmap = rospy.get_param('piece_target_position_map')
    print(piece_positionmap)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    #default starting pose before picking up a piece
    starting_pose = Pose(
        position=Point(x=0.6, y=0.4, z=0.35),
        orientation=overhead_orientation
    )

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")


    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)


    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    #Chosen pieces:
    #Baxter (white): King = K3, D2 pawn = P4, B2 pawn = P6
    #You (black): King = k3, A8 rook = r7, H8 rook = r0

    ready_to_spawn = rospy.get_param('ready_to_spawn')
    #move to start before spawning table
    pnp.move_to_start(starting_pose)
    ready_to_spawn = True
    rospy.set_param('ready_to_spawn', ready_to_spawn)

    #ready_to_spawn must be False before moving to first piece
    ready_to_spawn = wait_for_spawn(ready_to_spawn)

    #Move 6 pieces one by one (hardcoded)
    start_places = {'K3':'E1', 'P4':'D2', 'P6':'B2', 'k3':'E8', 'r0':'H8', 'r7':'A8'}
    for i in range(6):
        piece = rospy.get_param('current_piece')
        pnp.move_piece(piece, start_places[piece], pnp, big=True)
        ready_to_spawn = True
        rospy.set_param('ready_to_spawn', ready_to_spawn)
        #move arm to start position to avoid sweeping table
        pnp._servo_to_pose(starting_pose)
        if i <5:
            ready_to_spawn = wait_for_spawn(ready_to_spawn)

    #Once pieces are in place, do 4 set moves
    
    pnp.move_piece("P4", "D4",pnp)
    pnp.move_piece("r0", "H2",pnp, big=True)
    pnp.move_piece("P6", "B4",pnp)
    pnp.move_piece("r7", "A1",pnp, big=True)

    return 0

if __name__ == '__main__':
    sys.exit(main())
