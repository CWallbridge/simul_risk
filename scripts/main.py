import rospy
import os
import sys

import pysdf
import tf
import tf_conversions.posemath as pm

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties#spawn coke & table
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from tf.transformations import *

tfBroadcaster = tf.TransformBroadcaster()
submodelsToBeIgnored = []
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

object_locations = {"Table":str(models_paths) + "cafe_table/model.sdf","Coke":str(models_paths) + "coke_can/model.sdf","Chair":str(cob_models_path) + "objects/chair_ikea_borje.urdf"}

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

    global lastUpdateTime
    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
        return
    lastUpdateTime = rospy.get_rostime()

    poses = {'gazebo_world': identity_matrix()}
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        poses[link_name] = pysdf.pose_msg2homogeneous(link_states_msg.pose[link_idx])
        #print('%s:\n%s' % (link_name, poses[link_name]))

    for (link_idx, link_name) in enumerate(link_states_msg.name):
        #print(link_idx, link_name)
        modelinstance_name = link_name.split('::')[0]
        #print('modelinstance_name:', modelinstance_name)
        model_name = pysdf.name2modelname(modelinstance_name)
        #print('model_name:', model_name)
        if not model_name in model_cache:
            sdf = pysdf.SDF(model=model_name)
            model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
            if model_cache[model_name]:
                rospy.loginfo('Loaded model: %s' % model_cache[model_name].name)
            else:
                rospy.loginfo('Unable to load model: %s' % model_name)
        model = model_cache[model_name]
        link_name_in_model = link_name.replace(modelinstance_name + '::', '')
        if model:
            link = model.get_link(link_name_in_model)
            if link.tree_parent_joint:
                parent_link = link.tree_parent_joint.tree_parent_link
                parent_link_name = parent_link.get_full_name()
                #print('parent:', parent_link_name)
                parentinstance_link_name = parent_link_name.replace(model_name, modelinstance_name, 1)
            else: # direct child of world
                parentinstance_link_name = 'gazebo_world'
        else: # Not an SDF model
            parentinstance_link_name = 'gazebo_world'
        #print('parentinstance:', parentinstance_link_name)
        if is_ignored(parentinstance_link_name):
            rospy.loginfo("Ignoring TF %s -> %s" % (parentinstance_link_name, link_name))
            continue
        pose = poses[link_name]
        parent_pose = poses[parentinstance_link_name]
        rel_tf = concatenate_matrices(inverse_matrix(parent_pose), pose)
        translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_tf)
        #print('Publishing TF %s -> %s: t=%s q=%s' % (pysdf.sdf2tfname(parentinstance_link_name), pysdf.sdf2tfname(link_name), translation, quaternion))
        tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(link_name), pysdf.sdf2tfname(parentinstance_link_name))
        lastUpdateTime = rospy.get_rostime()

if __name__ == "__main__":
    
    #Spawn objects
    
    spawn_object("Table", object_locations["Table"])
    spawn_object("Coke", object_locations["Coke"],position=[0.89,0.37,0.80])#[0.66,0.21,0.80] this position is the one programmed for the scenario. This one is for trialing the counterfactual code.
    spawn_object("Chair", object_locations["Chair"], position=[1.2,0.8,0],model_type = "urdf")
    
    #Broadcast TF's for gazebo objects
    rospy.init_node("simul_risk")
    
    global lastUpdateTime
    lastUpdateTime = rospy.get_rostime()
    linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_states_msg)
    
    rospy.spin()
