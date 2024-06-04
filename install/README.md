## Additional Installation Configurations

dairlib can be built with many different external packages and solvers. Installation instructions are detailed here (some may be out of date)

### ROS
To integrate with ROS (tested on ROS Noetic with 20.04), the following steps are required.

Drake no longer supports Ubuntu 20.04. It is possible to compile against an older version of Drake, but there is not guarantee that all branches of dairlib will be compatible.
There is ongoing work on some experimental branches to support ROS2.
 
1. Install ROS http://wiki.ros.org/ROS/Installation
2. Do not forget to setup your environment. For instance, add these lines to `~/.bashrc`
```
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash 
```
3. Install additional dependencies
```
sudo apt install python3-rosinstall-generator python-catkin-tools python3-vcstool
```
4. Build the ROS workspace using catkin. From `dairlib/`,
```
sudo ./tools/workspace/ros/compile_ros_workspace.sh
```
5. Set the environment variable `DAIRLIB_WITH_ROS` to `ON`. For instance, add to `~/.bashrc`
```
export DAIRLIB_WITH_ROS=ON
```

### SNOPT
Download and setup SNOPT

dairlib, by default, assumes that users have access to SNOPT(https://web.stanford.edu/group/SOL/snopt.htm), though it is not required. **If you do not have SNOPT**, you will need to edit `.bazelrc` and change `build --define=WITH_SNOPT=ON` to `build --define=WITH_SNOPT=OFF`

For users at Penn, download SNOPT (https://www.seas.upenn.edu/~posa/snopt/snopt7.6.tar.gz) and add the following line to your `~/.bashrc`
```
export SNOPT_PATH=<the directory you downloaded to>/snopt7.6.tar.gz
```

There is no need to extract the tar.

### GUROBI
Download and setup GUROBI, currently tested with `gurobi1003`

```
export GUROBI_HOME=<gurobi_directory>
```

add the line `build --define=WITH_GUROBI=ON` to the `.bazelrc`

### Local Version of Drake
The library is meant to be built with Drake (see http://drake.mit.edu/ for more details). Often, we will use an older version of drake with the commit defined in the [WORKSPACE](WORKSPACE) file or using the environment variable `DAIRLIB_LOCAL_DRAKE_PATH` set to the root of the drake repository.

For example:
```
export DAIRLIB_LOCAL_DRAKE_PATH=/home/user/my-workspace/drake
```

### Invariant-EKF
State Estimation for Cassie is done using contact-aided invariant-EKF. `invariant-ekf` is an external repository forked from Ross Hartley's repository of the same name. By default, a pegged version of this forked repository is used i.e. the `bazel` branch of DAIR lab's fork of `invariant-ekf` is automatically downloaded and used. However, to make changes to the files, the [DAIR Lab's fork of invariant-ekf](https://github.com/DAIRLab/invariant-ekf/tree/bazel "DAIR Lab's fork of invariant-ekf") can be cloned as a local repository.

To use local version of `invariant-ekf`, set the environment variable `DAIRLIB_LOCAL_INEKF_PATH`, e.g.
```
export DAIRLIB_LOCAL_INEKF_PATH=/home/user/my-workspace/invariant-ekf
```




#### Docker (experimental, may be broken)
Docker support is currently experimental. See `install/bionic/Dockerfile` for an Ubuntu Dockerfile. Docker is being used in conjuction with Cirrus Continuous Integration, and should be better supported in the future.
