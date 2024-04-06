# Blockchain-Based-BFT-Swarm-SLAM
Blockchain-Based Byzantine Fault Tolerant Swarm-SLAM

**Abstract**
Effective methods for Simultaneous Localisation And Mapping (SLAM) are key to enabling autonomous robots to navigate unknown environments. While single-robot SLAM has been extensively researched, attention has shifted to multi-robot collaborative SLAM (C-SLAM) which offers the opportunity for higher performance thanks to parallel execution of mapping and localisation by a distributed team of robots. However, C-SLAM also introduces challenges in system scalability and consistent data aggregation, and exposes the system to potential security risks. 
In particular Byzantine robots, which are robots that behave improperly or maliciously, can heavily disrupt the C-SLAM process. This paper discusses how different types of Byzantine robots can disrupt Swarm-SLAM, the state-of-the-art decentralised C-SLAM framework for swarm robotics.
We then propose a new approach based on blockchain technology to mitigate several of the identified security threats and improve the Byzantine fault tolerance of Swarm-SLAM. The proposed solution consists of a blockchain-based smart contract that manages robots' reputations to identify and neutralise Byzantine robots. Our multi-robot simulation results show the existence of a trade-off between fault tolerance and efficiency in terms of map generation speed. With this work, we also release a custom open-source blockchain software integrated into the ROS2 framework.

**Package summary**
*Instructions for replicating your work (data access, code usage, dependencies)*

**Citation**
