# IF-ROS

**IF-ROS** is a ROS package that implements Insetting Formation(IF)-Based trajectory planning and cooperative control for the transportation system of cable-suspended payload with multi UAVs, which includes:

- `inset_ros_msgs`: messages corresponding to polyhedron and ellipsoid
- `inset_test_node`: test IF algorithm
- `InsetUtil`: C++ package that implements IF algorithm
- `iris-distro`: IRIS algorithm for iterative convex regional inflation by semidefinite programming
- `multi_main_controller`:  cooperative controller for the transportation system of cable-suspended payload with multi UAVs
- `waypoint_generator`: process waypoints

Video Links: [Youtube](https://www.youtube.com/watch?v=LjTXRFqyL6Q) or [Bilibili](https://www.bilibili.com/video/BV1Yg4y1p7zn/) (only forMainland China).

## 1. Installation

### 1.1 Prerequisite:

- `ROS(melodic)`
- [`catkin_simple`](https://github.com/catkin/catkin_simple)
- [`coppeliasim(V4.3.0 for Ubuntu 18.04) `](https://www.coppeliarobotics.com/previousVersions)

Please refer to the [ROS tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm) to enable CoppeliaSim ROS.

If the submodule is not initialized yet, run following commands at first:

```bash
cd /PATH/TO/IF_ROS
git submodule update --init --recursive
```

### 1.2 Install IRIS

In order to successfully compile the iris-distro package in this package, please follow the requirements provided in https://github.com/rdeits/iris-distro .

And make sure that the required Python modules are enabled, e.g. by inserting into your `.bashrc` file: `export PYTHONPATH=:${PYTHONPATH}:<python_prefix>` .

### 1.3 Install Mosek

You'll also need a license for the [Mosek](https://www.mosek.com/) optimization toolbox(iris-distro package includes a downloader for the Mosek code, but you have to get your own license). Mosek has free licenses available for academic use.

After obtaining a Mosek license, you need to enable it, e.g. by inserting into your `.bashrc` file: `export PATH=$PATH:/PATH/TO/mosek/mosek.lic`

### 1.4 Install CasADi

See [InstallationLinux](https://github.com/casadi/casadi/wiki/InstallationLinux) for the CasADi installation process。

If there are issues with locating CasADi library .so files, run `sudo ldconfig` in the terminal of the build folder where CasADi is installed.

## 2. Compilation

Using Catkin Tools (Recommended):

First, ensure that both [`catkin_simple`](https://github.com/catkin/catkin_simple) and `IF_ROS` are located in the `/PATH/TO/catkin_ws/src` directory. Then, execute the following command in the terminal:

```bash
sudo apt-get install python-catkin-tools
cd /PATH/TO/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Alternatively, if you want to debug, enter the following command in the terminal:

```bash
sudo apt-get install python-catkin-tools
cd /PATH/TO/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build
```

## 3. Usage

First, enter the following command in the terminal:

```bash
roscore
```

Then, start CoppeliaSim, open the corresponding scene file `cross.ttt`, and initiate the simulation.

Finally, run the following command in the terminal:

```bash
cd /PATH/TO/catkin_ws
source devel/setup.bash 
roslaunch inset_test_node cross.launch 
```

The experiment scene file path : `/PATH/TO/IF_ROS/multi_main_controller/Experimental_scene/cross.ttt`

## 5. Acknowledgement

We sincerely thank the following open-source projects for providing valuable, high-quality code:

[`mpl_ros`](https://github.com/sikang/mpl_ros)

[`iris-distro`](https://github.com/rdeits/iris-distro)

[`Vision-encoder-based-Payload-State-Estimator`](https://github.com/jianhengLiu/Vision-encoder-based-Payload-State-Estimator)

## 6.About

Author: Yu Zhang, Jie Xu, Cheng Zhao and Jiuxiang Dong from Northeastern University.

Related Paper:
IF-Based Trajectory Planning and Cooperative Control for Transportation System of Cable Suspended Payload With Multi UAVs, Yu Zhang, Jie Xu, Cheng Zhao and Jiuxiang Dong.

Our paper is to be published online soon in [IROS2023](https://ieee-iros.org/).
