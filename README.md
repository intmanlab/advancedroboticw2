# ros_dmp_AR_workshop


## Dynamic Motion Primitives for Learning from Demonstration

This package implements Dynamic Motion Primitives for Learning from Demonstration. This package was developed by https://github.com/abhishek098. Detailed report on analysis, implementation and use of this package can be found at https://github.com/abhishek098/r_n_d_report/blob/master/PadalkarAbhishek-%5BRnD%5DReport.pdf . 

Reference : Auke Jan Ijspeert, Jun Nakanishi, Heiko Hoffmann, Peter Pastor, and Stefan Schaal. Dynamical movement primitives: learning attractor models for motor behaviors. Neural computation, 25(2):328{373, 2013}. 

## Dependencies:

This package has been tested on Ubuntu 16.04, ROS kinetic and python 3.5. The codes depend on following libraries. We first setupa virtual environment within which we will install all the dependencies. 

* Step 1: Update your repositories

`sudo apt-get update`

* Step 2: Install pip for Python 3

`sudo apt-get install build-essential libssl-dev libffi-dev python-dev`
`sudo apt install python3-pip`

* Step 3: Use pip to install virtualenv

`sudo pip3 install virtualenv`

* Step 4: Create a virtual environment named venv

`virtualenv -p python3 venv`

* Step 5: Activate the environment 

`source venv/bin/activate`

Now you should see (venv) on your command line appearing before the username. 

* Step 6: Install the essential libraries: 

`pip install pyyaml`

`pip install pyaml`

`pip install catkin_pkg`

`pip install empy`

`pip install rospkg`

`pip install numpy`

`pip install matplotlib`

With this, we have successfully set up our virtual environment. Now we are ready to install the dependencies. 

* tinker

`sudo apt-get install python3-tk`

* pyQT5

` pip3 install PyQt5==5.9.2`

* pycairo

`sudo apt install libcairo2-dev`

`pip3 install pycairo`

* pyGObject

`sudo apt install libgirepository1.0-dev`

`pip3 install PyGObject`

Now we can deactivate the virtual environment. 

`deactivate`

## How to use the package

Open a new terminal and run the follwing commands. 

`sudo apt-get install python-catkin-tools `

`sudo apt-get install python-scipy`

Create a catkin workspace

`mkdir -p catkin_dmp/src`

Navigate to the source folder of this workspace from the terminal. 

`cd catkin_dmp/src`

Clone the repository:

`git clone https://github.com/intmanlab/advancedroboticw2.git`

Go to the root of the workspace and build the package:

`cd ..`

`catkin build`

Now please follow the instruction given in the workshop document. The information given below is only to introduce the package but it can be skipped as of now. 

#
This package provides two services:
1. Service for learning the DMP
2. Service for genearating motion from already learnt DMP

Geerated motion is published as navigation path on the topic:

`/generate_motion_service_node/cartesian_path`

And as cartesian trajectory on the topic:

`/generate_motion_service_node/cartesian_trajectory`

For more details about using these services, have a look at example clients in `example` folder. 

### Example clients
Running learn DMP and generate motion clients

Launch the DMP simulation launch file in one terminal using following command.

`roslaunch ros_dmp dmp_sim.launch`

Run rviz to visualize learned and demonstrated path published on following topics which can be visualized in rviz. 

`/learn_dmp_service_node/demonstrated_path`

`/learn_dmp_service_node/demonstrated_path`

In another terminal, run learn client script in `example` folder

`python learn_client.py`

This script calls the learn DMP service for imitating a artificial trajectory. 

Observe the imitated and demonstrated trajectories in rviz.

In another terminal, run generate motion client

`python motion_generation_client.py`

Genrated motion can be visualized in rviz on following topic:

`/generate_motion_service_node/cartesian_path`
