# README for Sumonity: Enhanced Traffic Simulation Interface

## Table of Contents
1. [Project Title](#project-title)
2. [Introduction](#introduction)
3. [Features](#features)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Architecture](#architecture)
7. [Agent Information](#agent-information)
8. [Vehicle Control Mechanism](#vehicle-control-mechanism)
9. [Dependencies](#dependencies)
10. [Configuration](#configuration)
11. [Documentation](#documentation)
12. [Examples](#examples)
13. [Troubleshooting](#troubleshooting)
14. [Contributors](#contributors)
15. [License](#license)

---

## Project Title
### Sumonity: Enhanced Traffic Simulation Interface

## Introduction
Sumonity is a cutting-edge interface that integrates the Simulation of Urban MObility (SUMO) with the Unity game engine. This combination brings a new level of realism and interactivity to traffic simulations. By harnessing SUMO's comprehensive traffic modeling and Unity's advanced graphical engine, Sumonity offers unprecedented simulation fidelity and an enhanced user experience.

![Overview of Sumonity Architecture](images/Sumonity_overview.png)

## Features
- **Microscopic Traffic Simulation**: Utilizes SUMO's detailed traffic modeling for highly realistic simulations.
- **Advanced Graphical Rendering**: Takes advantage of Unity's engine for superior visual quality.
- **Pure Pursuit Control**: Offers improved control over individual vehicle behaviors in Unity's simulation environment.
- **Flexible Vehicle Types**: Accommodates a variety of vehicle types including passenger cars, buses, taxis, and pedestrians.

## Installation
1. Get the sub repositories:
```
git submodule update --init --recursive
```

2. Go into the sub repository for Sumo and install the virtual environment as follows:

```
sudo apt install python3-pip python3-virtualenv python3-tk
pip3 install virtualenv
```
Now setup the virtual environment
```
virtualenv venv
source venv/bin/activate
pip install -r requirements.txt
```

3. Add the Sumo Bridge Prefab to your Unity Scene.



## Usage
Guidelines for using Sumonity are as follows:
- In order to achieve a synchronized sumo and untiy environment we are using mathwork roarunner to create the "digital twin" from there we export the 3d model to unity and the opendrive file to sumo. We convert the open drive file using netconvert:

```
netconvert --opendrive .\TUM_009.xodr -o tum_009.net.xml
```
Hint: Execute the comment using the active virtual environment.

After setting up the network file, we create demand etc. as usual.

## Architecture
Sumonity's architecture is built around a Python-based interface that connects SUMO and Unity. The following diagram provides an overview of this setup:

![Sumonity Architecture](images/Sumonity_overview_detailed.png)

## Agent Information
Sumonity supports multiple SUMO vehicle types, each characterized by properties such as position, orientation, speed, and control signals.

## Vehicle Control Mechanism
- **Longitudinal Control**: Features a Proportional-Integral-Derivative (PID) controller for precise management of velocity and position.
- **Lateral Control**: Employs a Pure Pursuit Control (PPC) strategy for accurate path tracking.

![Detailed Architecture of Sumonity](images/lateral_control_strategy.png)

## Dependencies
Setup is tested on Ubuntu 20.04.

## Configuration
(TODO: Provide details on how to configure Sumonity for different simulation environments.)

## Documentation
(TODO: Include a link or directions to the full documentation of Sumonity.)

## Examples
(TODO: Provide example simulations or use cases to demonstrate how Sumonity can be utilized.)

## Troubleshooting
- setup the correct file in the socketServer.py
```
traci.start(["sumo","-c", "tum_008.sumocfg","--num-clients", "1"])
```
We are going to introduce a scenario selection menu in Unity in the future, to avoid this issue.


- make the script executable
```
sudo chmod +x socketServer.py
```

- module not found error:
Make sure you have installed the virtual environment, by following the installation instructions.



## Contributors
(TODO: List the contributors to the Sumonity project.)

## License
(TODO: Specify the licensing information for Sumonity.)

---

**Note:** This README is a work in progress. For sections marked as TODO, further details are required to complete this document. Please provide the missing information or check back later for updates.
