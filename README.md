# EMAP

EMAP is a planning framework developed in the scope of the master's thesis titled 'Using Artificial Evolution for Planning in Self-Organizing Robot Systems'. It intends to solve the multi-agent programming contest of 2018.

## Installation Guide

EMAP requires ROS melodic and Python 3. The following is a step-by-step guide to install all necessary components to run EMAP. 

### ROS setup

Under Ubuntu, install ROS melodic according to the official ROS [installation guide](http://wiki.ros.org/Installation/Ubuntu).

In arch-based systems, ROS can be installed from the AUR repository. You need an AUR package manager like yay:

```shell
yay -S ros-melodic-ros-base
```

After installing ROS, initialize it by

```shell
sudo rosdep init
rosdep update
```

You might want to add the global ROS workspace to your bashrc:

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Python packages

EMAP is implemented with python3 and the following python3 packages have to be installed, e.g. by using pip. In Ubuntu, the packages can be installed by

```shell
sudo pip3 install scipy, networkx
```
Arch-based systems use python3 as a default and the necessary packages can be installed by using

```shell
sudo pip install scipy, networkx
```

## Usage

The two zips containing graphhopper and the massim simulation server in the folder my_mapc/third_party have to be unpacked. After that, in the my_mapc folder, run

```shell
catkin_make
```

The massim server can be started within the my_mapc folder by running 

```shell
bash ./massim.sh
```
There are two options for starting the server. Option 0 starts the simulation for one team. The server then expects only a team A. The evolutionary and randomized versions of EMAP for this configuration can be started by running

```shell
bash ./emap_evolution.sh
```
or

```shell
bash ./emap_random_a.sh
```
Option 1 in the massim configuration starts a competition and the server expects two teams, i.e. team A and team B. If you want to run a competition between the randomized and the evolutionary version of EMAP, the following two scripts have to be run in separate terminals:

```shell
bash ./emap_evolution.sh
```
and
```shell
bash ./emap_random.sh
```

## Evaluation and visualization

The plots shown in the thesis' evaluation chapter can be reproduced by running

```shell
python3 ./evaluate.py
```
in the my_mapc folder. This will generate the plots in the my_mapc/plots folder and produce latex-formatted output for the statistics presented in the tables in the thesis.