# Udacity Project 13: Capstone
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Team Members


Name              | E-Mail                            | Slack
------------      | -------------                     | ------
Stefan Cyliax     | stefan.cyliax@gmail.com           | @stefancyliax
Paul Michalik     | paul.michalik@outlook.com         | @paul-michalik
Frank Keidel      | f.keidel@gmx.de                   | @frankk
Patrick Schalast  | patrick.schalast@gmail.com        | @patrickschalast


The goal of this project was to implement a complete stack that can drive a vehicle down a street and recognize and respect traffic lights. For this the [Robot Operating System](http://www.ros.org/) was used. 

Real Parking Lot at Udacity: (Youtube link)

[![Real Parking Lot at Udacity](https://github.com/stefancyliax/CarND-Capstone/raw/master/imgs/track3.gif)](https://www.youtube.com/watch?v=LB_6-3flmas)


Highway video: (Youtube link)

[![Project track1](https://github.com/stefancyliax/CarND-Capstone/raw/master/imgs/track1.gif)](https://youtu.be/_33XWYFTFxg)

Test Lot Video: (Youtube link)

[![Project track2](https://github.com/stefancyliax/CarND-Capstone/raw/master/imgs/track2.gif)](https://youtu.be/AKB_CoLEhTs)


## Submission checklist and requirements

- [x] Smoothly follow waypoints in the simulator.
- [x] Respect the target top speed set for the waypoints' twist.twist.linear.x in waypoint_loader.py. Be sure to check that this is working by testing with different values for kph velocity parameter in /ros/src/waypoint_loader/launch/waypoint_loader.launch.
- [x] If your vehicle adheres to the kph target top speed set here, then you have satisfied this requirement.
- [x] Stop at traffic lights when needed.
- [x] Stop and restart PID controllers depending on the state of /vehicle/dbw_enabled.
- [x] Publish throttle, steering, and brake commands at 50hz.
- [x] Launch correctly using the launch files provided in the capstone repo.

## rqt_graph
![rqt_graph](https://github.com/stefancyliax/CarND-Capstone/raw/master/imgs/rosgraph.png)




---
---
# Udacity README


Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation
### More to read

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Run on Docker for Windows

Clone the project as described in section "Usage". 

```
git clone https://github.com/stefancyliax/CarND-Capstone.git
```  
    
On the first usage or when you modify `Dockerfile` build the Docker image
```
cd CarND-Capstone
docker_build.bat
```

Run the image in a new container with interactive shell terminal: `docker_run.bat`. Then proceed as described in section "Usage".

Run the image in a new container and log-in via RDP: `docker_run_rdp.bat`. Log in and proceed as described in section "Usage". To log-in into a running container, open a terminal (e.g. via [Kitematic](https://kitematic.com/) or `docker attach <container-id>` or `docker attach happy-capstone-win`. (Work in progress, not yet supported)
 
### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
