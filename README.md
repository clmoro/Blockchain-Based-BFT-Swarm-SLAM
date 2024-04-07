# Blockchain-Based-BFT-Swarm-SLAM
Repository that contains the code associated to the paper: "Blockchain-Based Byzantine Fault Tolerant Swarm-SLAM" (April 2024). Additional material related to the paper and the results can be found at this [website](https://sites.google.com/view/bft-swarm-slam).


**Abstract**

Effective methods for Simultaneous Localisation And Mapping (SLAM) are key to enabling autonomous robots to navigate unknown environments. While single-robot SLAM has been extensively researched, attention has shifted to multi-robot collaborative SLAM (C-SLAM) which offers the opportunity for higher performance thanks to parallel execution of mapping and localisation by a distributed team of robots. However, C-SLAM also introduces challenges in system scalability and consistent data aggregation, and exposes the system to potential security risks. 
In particular Byzantine robots, which are robots that behave improperly or maliciously, can heavily disrupt the C-SLAM process. This paper discusses how different types of Byzantine robots can disrupt Swarm-SLAM, the state-of-the-art decentralised C-SLAM framework for swarm robotics.
We then propose a new approach based on blockchain technology to mitigate several of the identified security threats and improve the Byzantine fault tolerance of Swarm-SLAM. The proposed solution consists of a blockchain-based smart contract that manages robots' reputations to identify and neutralise Byzantine robots. Our multi-robot simulation results show the existence of a trade-off between fault tolerance and efficiency in terms of map generation speed. With this work, we also release a custom open-source blockchain software integrated into the ROS2 framework.

**Packages summary**

* [toychain-ROS2](https://github.com/clmoro/toychain-ROS2): package that contains our Toy-Chain ROS2 integration and our custom smart contract;
* [toychain-swarm-SLAM](https://github.com/clmoro/toychain-swarm-SLAM): package that contains the code for our simulation experiments and for the blockchain/Swarm-SLAM interface;
* [representative_experiment_data](link): .csv file that contains the APE error dataset for results replication. It is related to the simulation runs of the representative experiment with 10 metres Byzantine perturbation described in the paper.
  
**Additional requirements**

* [ROS2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html);
* [Swarm-SLAM and its dependencies](https://github.com/MISTLab/Swarm-SLAM): the open-source C-SLAM system used in the project;
* [Gazebo](https://classic.gazebosim.org/tutorials?tut=ros2_installing): to install if you want run the simulation environment used in the paper;
* [Toy-Chain](https://github.com/teksander/toychain): it is a custom blockchain already integrated in the packege [toychain-ROS2](https://github.com/clmoro/toychain-ROS2).

**How to use**

```
# Install Swarm-SLAM ([Swarm-SLAM start-up instructions](https://lajoiepy.github.io/cslam_documentation/html/md_startup_instructions.html))
sudo apt install python3-vcstool
git clone https://github.com/MISTLab/Swarm-SLAM.git
cd Swarm-SLAM
mkdir src
vcs import src < cslam.repos
# Download toychain-swarm-SLAM
git clone https://github.com/clmoro/toychain-swarm-SLAM
# Download toychain-ROS2
git clone https://github.com/clmoro/toychain-ROS2
# Run your Swarm-SLAM nodes and Swarm-SLAM visualisation nodes (optional)
# Launch the Gazebo simulation with 8 robots (in a new terminal)
cd ../toychain-swarm-SLAM
source /opt/ros/foxy/setup.bash
source instal/setup.bash
ros2 launch multiturtlebots_pkg multiturtlebots_arena1.py
# Launch the robots controllers (in a new terminal)
cd ../toychain-swarm-SLAM
source /opt/ros/foxy/setup.bash
source instal/setup.bash
ros2 launch multiturtlebots_controller_pkg controller_estimator.launch.py
# Launch the nodes of the blockchain Swarm-SLAM interface (in a new terminal)
cd ../toychain-swarm-SLAM
source /opt/ros/foxy/setup.bash
source instal/setup.bash
ros2 launchcustom_msg_pkg launch.xml
# Run the blockchain node (in a new terminal)
cd ../toychain-ROS2
source /opt/ros/foxy/setup.bash
source instal/setup.bash
ros2 run blockchain_controller_pkg BC_controller
```

**Citation**

If you find our project useful, please cite our paper.
