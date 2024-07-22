#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

#get list of piece names from initial setup
input_linknames = rospy.get_param('piece_names')

# Global variable where these poses of each piece are stored
poses = None


def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_linknames
    global poses

    poses = {'world': link_states_msg.pose[0]} # get world link
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]
        if modelname in input_linknames:
            poses[modelname] = link_states_msg.pose[link_idx]



def main():
    global poses
    
    rospy.init_node('gazebo2tfframe')

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)
    rospy.loginfo('Spinning')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if poses is not None:
            try:
                for piece in poses:
                    if piece == 'world':
                        continue
                    pos = poses[piece].position
                    ori = poses[piece].orientation
                    rospy.loginfo(piece)
                    # Publish transformation given in pose
                    tfBroadcaster.sendTransform((pos.x, pos.y, pos.z - 0.93), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), piece, 'world')
            except:
                continue    
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()