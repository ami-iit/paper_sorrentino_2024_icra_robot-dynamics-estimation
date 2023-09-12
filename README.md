<h1 align="center">
  Stochastic Sensor Fusion for <br/> Joint-Torque Sensorless Humanoid Robots
</h1>

## Installation

This repository requires to install [bipedal-locomotion-framework]([https://github.com/ami-iit/ironcub_software](https://github.com/isorrentino/bipedal-locomotion-framework/tree/paper)) and its [dependencies](https://github.com/ami-iit/bipedal-locomotion-framework/#page_facing_up-mandatory-dependencies). Follow the [instructions](https://github.com/ami-iit/bipedal-locomotion-framework/#package-install-with-conda-recommended), then just clone/download this repo to use it. If you are interested in running the Python example here, please follow https://github.com/ami-iit/bipedal-locomotion-framework/#snake-python to enable the Python bindings.

## Repo usage
- The [dataset](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/dataset) folder contains the datasets related to the experiments presented in the paper.
- The python folder contains an example of usage of the [`robot-dynamics-estimation`](https://github.com/isorrentino/bipedal-locomotion-framework/blob/paper/src/Estimators/include/BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h) library. It contains a dataset logged from the real ergoCub robot which can be used to run the joint torque estimation for the right leg joints. Further, the folder contains the configuration files needed to run the example. The folder `urdf` contains the robot urdf model. 

## Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://user-images.githubusercontent.com/43743081/89022636-a17e9e00-d322-11ea-9abd-92cda85d3705.jpeg" width="40">](https://github.com/isorrentino) | [@inessorrentino](https://github.com/isorrentino) |
