<h1 align="center">
  UKF-Based Sensor Fusion <br/> for Joint-Torque Sensorless Humanoid Robots

</h1>

<div align="center">
<b>Ines Sorrentino</b>, Giulio Romualdi, Daniele Pucci <br> <br>
</div>

<div align="center">
    📅 Submitted to the 2024 International Conference on Robotics and Automation (ICRA) 🤖
</div>

## Installation

This repository requires to install [bipedal-locomotion-framework]([https://github.com/ami-iit/ironcub_software](https://github.com/isorrentino/bipedal-locomotion-framework/tree/paper)) and its [dependencies](https://github.com/ami-iit/bipedal-locomotion-framework/#page_facing_up-mandatory-dependencies). Follow the [instructions](https://github.com/ami-iit/bipedal-locomotion-framework/#package-install-with-conda-recommended), then just clone/download this repo to use it. If you are interested in running the Python example here, please follow https://github.com/ami-iit/bipedal-locomotion-framework/#snake-python to enable the Python bindings.

## Repo usage
- The [dataset](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/dataset) folder contains the datasets about experiments presented in the paper.
- The [python](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/python) folder contains two example applications of usage of the [`robot-dynamics-estimation`](https://github.com/isorrentino/bipedal-locomotion-framework/blob/paper/src/Estimators/include/BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h) library. You can run the estimation using the Python bindings on the right leg of ergoCub or on a simpler model generated manually. In particular:
  - the folder [`config`](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/python/config) contains the configuration parameters for the two models
  - the folder [`urdf`](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/python/urdf) contains both the urdfs
  - the folder [`dataset`](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/python/dataset) provides one dataset logged from ergoCub during one of the experiments and one dataset containing synthetic data generated for the model `four_joints_two_sensors`
  - the two applications you can run are [`rde_ergoCub_application.py`](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/blob/main/python/rde_ergocub_application.py) and [`rde_synthetic_application.py`](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/blob/main/python/rde_synthetic_application.py) both should be run from the [containing folder](https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/tree/main/python).

## Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://user-images.githubusercontent.com/43743081/89022636-a17e9e00-d322-11ea-9abd-92cda85d3705.jpeg" width="40">](https://github.com/isorrentino) | [@inessorrentino](https://github.com/isorrentino) |
