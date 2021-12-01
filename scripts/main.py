import rospy
import os
import sys

import pysdf
import tf
import tf_conversions.posemath as pm

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties#spawn coke & table
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates, ModelStates
from tf.transformations import *

tfBroadcaster = tf.TransformBroadcaster()
submodelsToBeIgnored = []
uselink = {}
lastUpdateTime = None
updatePeriod = 0.05
model_cache = {}

#Var definitions
#Dictionary containing all relevant paths

#Default path for gazebo models
models_paths = os.path.expanduser('~/.gazebo/models/')

if 'GAZEBO_MODEL_PATH' in os.environ:
    models_paths = os.environ['GAZEBO_MODEL_PATH']

#Default path for the Care-o-bot gazebo models
cob_models_path = os.path.expanduser('~/ros/melodic/dev/devel/share/cob_gazebo_objects/')

if 'COB_MODEL_PATH' in os.environ:
    cob_models_path = os.environ['COB_MODEL_PATH']

object_locations = {"Table":str(models_paths) + "cafe_table/model.sdf","Coke":str(models_paths) + "coke_can/model.sdf","Chair":str(cob_models_path) + "objects/chair_ikea_borje.urdf","Paper":str(models_paths) + "paper/model.sdf"}

def make_pose(position,quaternion):
    #make a pose of object given xyz, and quaternion
    object_pose = Pose()
    object_pose.position.x = float(position[0])
    object_pose.position.y = float(position[1])
    object_pose.position.z = float(position[2])
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose
    
def spawn_object(name,model_path,position=[1,0,0],quaternion=[0,0,0,1],model_type = "sdf"):
    #Read xml file
    f = open(model_path)
    
    # spawn new model (Chair)
    req = SpawnModelRequest()
    req.model_name = name
    req.model_xml = f.read()
    f.close()
    req.initial_pose = make_pose(position,quaternion)

    rospy.wait_for_service('/gazebo/spawn_'+model_type+'_model',30)
    srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_'+model_type+'_model', SpawnModel)
    res = srv_spawn_model(req)

#This code is based on gazebo2tf_node.py from gazebo2rviz: https://github.com/andreasBihlmaier/gazebo2rviz
#At the time of writing their code did not provide an easy way to adapt the desired parts of their code to our own. 

def is_ignored(link_name):
    for ignored_submodel in submodelsToBeIgnored:
        if link_name.startswith(ignored_submodel + '::'):
            return True
        return False

def on_link_states_msg(link_states_msg):
    """Publishes human tfs to /tf ros topic"""
    global lastUpdateTime,uselink
    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
        return
    lastUpdateTime = rospy.get_rostime()

    poses = {'gazebo_world': identity_matrix()}#world coordinate frame
    relative_poses = {'gazebo_world': identity_matrix()}#relative to parent as defined in uselink frame
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        poses[link_name] = pysdf.pose_msg2homogeneous(link_states_msg.pose[link_idx])
        #print('%s:\n%s' % (link_name, poses[link_name]))
    for link_name in poses:
        if link_name in parent_link: #check if the link has a parent.
            parent_name = parent_link[link_name]
            parent_tf = poses[parent_name]
            relative_poses[link_name] = concatenate_matrices(inverse_matrix(parent_tf), poses[link_name])

    #link gazebo_world to world.
    parentinstance_link_name = 'gazebo_world'
    translation, quaternion = pysdf.homogeneous2translation_quaternion(poses[parentinstance_link_name])
    tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), parentinstance_link_name, "world")
    
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        #connect all links to gazebo world
        if link_name in parent_link:
            parentinstance_link_name = parent_link[link_name]
            #print('parentinstance:', parentinstance_link_name)
            if is_ignored(parentinstance_link_name):
                rospy.loginfo("Ignoring TF %s -> %s" % (parentinstance_link_name, link_name))
                continue
            rel_pose = relative_poses[link_name]#change relative_poses to poses here to broadcast in world frame coordinates
            translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_pose)
            #print('Publishing TF %s -> %s: t=%s q=%s' % (pysdf.sdf2tfname(parentinstance_link_name), pysdf.sdf2tfname(link_name), translation, quaternion))
            tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(uselink[link_name][1]), pysdf.sdf2tfname(uselink[link_name][2]))
            #tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(link_name), "world")
            lastUpdateTime = rospy.get_rostime()
            
def link_odom_world(data):
    """links odom_combined to world frame dynamically."""
    global lastUpdateTimeOdom
    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTimeOdom
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
        return
    lastUpdateTimeOdom = rospy.get_rostime()
    r_i = data.name.index('robot')#robot index
    r_p = data.pose[r_i]#robot pose
    r_p_h = pysdf.pose_msg2homogeneous(r_p)
    translation, quaternion = pysdf.homogeneous2translation_quaternion(r_p_h)
    tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), "odom_combined", "world")


if __name__ == "__main__":
    
    global  uselink,parent_link,model_name_path,lastUpdateTime,lastUpdateTimeOdom 
    
    #Spawn objects
    spawn_object("Paper",object_locations["Paper"], position=[1.07,-0.05,1],quaternion=[0,0,1,0])#q = R.from_euler('xyz',[0, 0, 180], degrees = True),from scipy.spatial.transform import Rotation as R
    spawn_object("Table", object_locations["Table"])
    spawn_object("Coke", object_locations["Coke"],position=[0.86,0.40,0.81])#[0.75,0.37,0.81] this position is the one programmed for the scenario. This one is for trialing the counterfactual code.
    spawn_object("Chair", object_locations["Chair"], position=[1.2,0.8,0],model_type = "urdf")
    
    #Broadcast TF's for gazebo objects

    uselink = {
        "actor::foot_l":["actor::lowerleg_l","foot_l","knee_l"],
        "actor::foot_r":["actor::lowerleg_r","foot_r","knee_r"],
        "actor::hand_l":["actor::lowerarm_l","hand_l","elbow_l"],
        "actor::hand_r":["actor::lowerarm_r","hand_r","elbow_r"],
        "actor::head":["actor::spine_03","neck","spine_upper"],
        "actor::head_end":["actor::head","head","neck"],
        "actor::lowerarm_l":["actor::upperarm_l","elbow_l","shoulder_l"],
        "actor::lowerarm_r":["actor::upperarm_r","elbow_r","shoulder_r"],
        "actor::lowerleg_l":["actor::upperleg_l","knee_l","hip_l"],
        "actor::lowerleg_r":["actor::upperleg_r","knee_r","hip_r"],
        "actor::root":["gazebo_world","person","world"],
        "actor::spine_03":["actor::root","spine_upper","person"],
        "actor::spine_01":["actor::spine_03","spine_lower","spine_upper"],
        "actor::upperarm_l":["actor::spine_03","shoulder_l","spine_upper"],
        "actor::upperarm_r":["actor::spine_03","shoulder_r","spine_upper"],
        "actor::upperleg_l":["actor::spine_01","hip_l","spine_lower"],
        "actor::upperleg_r":["actor::spine_01","hip_r","spine_lower"],
        "Coke::link":["gazebo_world","coke","world"]
    }
    parent_link = {k: uselink[k][0] for k in uselink}
    #convert     

    cur_path = os.path.dirname(__file__)#from
    
    model_name_path = {
            "actor":os.path.relpath('..\\worlds\\testfile.txt', cur_path),
    }
    
    rospy.init_node("simul_risk")
    
    lastUpdateTime = rospy.get_rostime()
    lastUpdateTimeOdom = rospy.get_rostime()
    linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_states_msg)
    odom_combined_linker = rospy.Subscriber('gazebo/model_states', ModelStates, link_odom_world)

    rospy.spin()
