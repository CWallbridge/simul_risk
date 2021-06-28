import rospy
import os
import sys

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties#spawn coke & table
from geometry_msgs.msg import Pose

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


if __name__ == "__main__":
    #Spawn objects
    spawn_object("Table", object_locations["Table"])
    spawn_object("Coke", object_locations["Coke"],position=[0.89,0.37,0.80])#[0.66,0.21,0.80] this position is the one programmed for the scenario. This one is for trialing the counterfactual code.
    spawn_object("Chair", object_locations["Chair"], position=[1.2,0.8,0],model_type = "urdf")
