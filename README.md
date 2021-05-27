# simul_risk
Contents
1. Setting up the simulation
 1. Care-o-bot environment
 1. simul_risk scenario
1. Known issues
1. Implementing your own animations
## Installation instructions
### Preliminaries
1. Make sure you are running ROS Melodic.
 1. *This has only been tested on Ubuntu 18.04 on a clean installation*
1. Follow [this](https://docs.google.com/document/d/1eiPcR867ab5HGgm4HHsc-XqRiZFeQXtwseT_8zeoQHQ/edit?usp=sharing) guide to set up the Care-o-Bot simulation environment.
### Setup the Scenario
*If you have not already done so, set the environment variables needed for launching Gazebo with Care-o-Bot.*
'export ROBOT_ENV=ipa-apartment'
'export ROBOT=cob4-8'
1. Clone the Repository
 1. 'cd simul_risk/launch'
  1. Change the robot.launch "world\_name" from "worlds/empty.world" to the full path. For example "/home/risk/ws/src/simul_risk/worlds/empty.world"
 1. 'cd ../worlds'
  1. Change the relative paths inside the <filename> tags from "worlds/corrected.dae" to the full path. For example "/home/risk/ws/src/simul_risk/worlds/corrected.dae"
  1. 'cd ..'
1. Start Gazebo with pre-defined world with 'roslaunch launch/robot.launch'
1. Launch another terminal, navigate to simul_risk directory and 'rosparam load scripts/newconfig.yaml'
 1. This should make available pre-defined positions for the robot arms.
1. 'python scripts/main.py'
 1. *NB: You may need to pip install any missing packages*
 1. You also need to download (from Gazebo) and change the paths to point to the right location of the models in main.py script line 17 (definition of object_locations)
1. Make the Coke can static
 1. Navigate to "/home/your_username/.gazebo/models/coke_can/model.sdf" and open it in text editor.
 1. 
If all has gone well, then the once the script is launched, the robot will perform the scenario.
## Known Issues
1. Grasping from the main.py script is buggy. The coke can sticks to the hand too early.
 1. This can be adjusted via adjusting the radius parameter inside the updateObject() function.
  1. The issue is that due to imperfect odometry, the robot does not always navigate to the same position and thus the radius will vary!
  1. Currently the robot grasps a predifned position with respect to its torso.
 1. The coke can (when grabbed) is upside down.
  1. This should be fixed in the next version.
1. The robot does not reach the exact position specified in the main.py file. This is because of drift (perhaps bad odometry).
1. Animation imperfections.
 1. The coke can and the arms of the human are slightly above the table.

## Implementing and your own animations
Please refer to [here](https://drive.google.com/file/d/1Et-1vtnkE1YNcKgtow8Xngz5-m_-ZPGt/view) for a detailed guide on how the current animation was created.
[here](http://gazebosim.org/tutorials?tut=actor&amp;cat=build_robot) is an official gazebo tutorial which is a good starting point.
