#Most of the functionality has now been moved to risk_uwds!
#This script now only spawns coke,table and chair. Perhaps in the future, it should be deleted entirely in favour of configuring the world file directly.

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties#spawn coke & table
from geometry_msgs.msg import Pose

#Var definitions
#Dictionary containing all relevant paths
object_locations = {"Table":"/home/risk/.gazebo/models/cafe_table/model.sdf","Coke":"/home/risk/.gazebo/models/coke_can/model.sdf","Chair":"/home/risk/ws/dev/share/cob_gazebo_objects/objects/chair_ikea_borje.urdf"}

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
    
def spawn_object(name,position=[1,0,0],quaternion=[0,0,0,1],model_type = "sdf"):
    #Read xml file
    f = open(object_locations[name])
    
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
	#Actual script
	#Spawn objects
	spawn_object("Table")
	spawn_object("Coke",position=[0.89,0.37,0.80])#[0.66,0.21,0.80] this position is the one programmed for the scenario. This one is for trialing the counterfactual code.
	spawn_object("Chair", position=[1.2,0.8,0],model_type = "urdf")
