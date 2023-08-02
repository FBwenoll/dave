# DAVE

Due to the lack of virgil related files in the original project and a series of problems caused by the change of the name of the marine_msgs package, the author made some changes on the basis of the original project.   


All documentation of Project DAVE can be found in the [Dave Documentation (https://field-robotics-lab.github.io/dave.doc/)](https://field-robotics-lab.github.io/dave.doc/).

# Get Source Code

- Clone this repository and other relevant repositories provided under FBwenoll:

``` bash
 mkdir -p ~/uuv_ws/src
  cd ~/uuv_ws/src
  git clone https://github.com/FBwenoll/dave.git
```
- Use `vcs` to clone source dependencies   
If not already installed -[install vcstool.](http://wiki.ros.org/vcstool)

- Use `vcs` to read the input file and clone required dependencies

``` bash
cd ~/uuv_ws/src
vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
```

If you get error `Could not determine ref type of version: git@github.com: Permission denied (publickey).`, try following [Git Hub Error Permission Denied Publickey](https://docs.github.com/en/authentication/troubleshooting-ssh/error-permission-denied-publickey) and remove clone failed empty directory and try again

``` bash
cd ~/uuv_ws/src
rm -rf dockwater ds_msgs ds_sim eca_a9 rexrov2 uuv_manipulators uuv_simulator
vcs import --skip-existing --input dave/extras/repos/dave_sim.repos .
```

- GPU Multibeam sonar

__DO NOT INCLUDE THIS if you are not using multibeam sonar.__  

__It require CUDA Library and NVIDIA driver along with the NVIDIA graphics card that supports CUDA feature.__

```bash 
cd ~/uuv_ws/src
vcs import --skip-existing --input dave/extras/repos/multibeam_sim.repos .
```

# Build

 - Now that you’ve set up your development environment and obtained the source code, you can build the project by running:

 ``` bash
 cd ~/uuv_ws
catkin_make
 ```

 - When the build is finished, source your new `setup.bash/zsh`:
 ``` bash
 source ~/uuv_ws/devel/setup.bash
 or
 source ~/uuv_ws/devel/setup.zsh
 ```
 - Optionally, you may wish to add source `~/uuv_ws/devel/setup.bash/zsh` to your `.bashrc`/`.zshrc` file.

 - An alternative to `catkin_make` is to use the newer `catkin build`(Recommended).
 