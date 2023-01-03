# Example Error-Aware Robotic System

## Introduction
This repository accompanies the paper On Using Social Signals to Enable Flexible Error-Aware HRI. It contains the source code for an example Error-Aware Robotic System. The system is comprised of three components (1) error management, (2) perception component, and (3) robot controller. 

## Software and Hardware Requirements for Running 

Environment:
- Windows 10 x64
- Linux Machine


Prerequisites:
- Python3
- [Kinova Kortex API](https://github.com/Kinovarobotics/kortex/blob/master/api_python)
- Microsoft Platform for Situated Intelligence ([\psi](https://github.com/microsoft/psi))
- CUDA enabled GPU for the automatic error detection ml algorithm using social signals
- Visual Studio 2017 (to compile OpenFace)
	- Desktop development with C++ (Workloads)
	- .NET desktop development (Workloads)
	- .NET Framework 4.6.1 SDK (Individual Component)
	- .NET Framework 4.6.1 targeting pack (Individual Component)
	- VC++ 2017 version 15.9v14.16 Libs for Spectre
	- VC++ 2017 version 15.9v14.16 latest v141 tools
- [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace) for AU calculations 


**NOTE**: this system is currently configured for a dyadic HRI and tailored for a specific task, assembly task where the participant collaboratively build structures out of pipes with a robot (Kinova Gen3 robot arm). Files are indicated at the top whether it contains task and robot specific code.

- - - -

## Contents
The Human Response Robot Error Detector folder contains files to run the perception component as well as to facilitate communication across all 3 components. This only runs on Windows 10 x64. The files are: 
- DetectorMain.cs—is the main file (“startup object”)  for the entire robotic system. It handles running and storing video, audio data, calculated AUs per timestep (1/3s). In addition, it establishes socket connections with the other two components. It sends commands generated from speech recognition and error detection notifications to the robot controller and receives whether the robot is moving. Also it sends AUs per timestep to the social signal machine learning algorithm and receives possible new error detections.
- IntegratedOpenFace.cs—This file calculates the AUs from video of a participant. It receives images from DetectorMain and send back the AUs and facial detection confidence for the timestep. These metrics are calculated through importing functions from OpenFace. This file makes the assumption that there is only one face visible in the view of the camera.
- IntegratedOpenFace2.cs—Identical to the file above. It is used in the case where the system has two camera connected. Default is 2 Azure Kinect cameras.
- Commands.grxml—This file contains all of the possible verbal commands used in the system. This is ported into DetectorMain for the speech recognition system.
- packages.config—Contains all of the NuGet packages required to run the system.
We recommend loading the folder as a project into Visual Studio 2017 on a Window 10 machine. Reason for this is that OpenFace, which is imported into the project, only works with VS 2017.


The Robot Controller folder contains files that takes in commands from Human Response Robot Error Detector and controls the robot (Kinova Gen3) to run sequences according to the commands. It sends whether the robot is moving to the Human Response Robot Error Detector. This should be run on a  Linux machine. The files are:
- connectPsiRobot.py—This file bridges the Human Response Robot Error Detector and the robot. It receives commands from DetectorMain.cs and sends run sequence commands to the robot. The specific commands it could receive are action requests, error detection notification, and query responses. In response to the commands, it sends back whether the robot is moving or not. It runs pre-programmed sequences and handles recovery (pre-programmed sequences) from errors once an error detection has been indicated.
- runSequence.py—This file provides functions that, low-level, runs the robot given a sequence handle provided by connectPsiRobot.py.
- utilities.py—This file is from the Kinova Kortex API to supplement connecting and running the robot.


The Social Signal Model For Detection folder contains files that receive AUs per timestep and outputs whether a new robot error has occurred and estimated start of that error. The files are:
	- connectML.py—This file contains the machine learning algorithm trained on social signals (trained on a Programming by Demonstration scenario). It receives AUs and whether the robot is currently moving from DetectorMain.cs and replies with whether a new error has occurred and 
	- CaseFullDict—Example binary classifier trained on  a Programming by Demonstration scenario.
  
- - - -

## Usage
To run the system, the sequence of commands are below (order matters):
1. python connectML.py
2. Run Human Response Robot Error Detector using DetectorMain.cs as the  “startup object”
3. python connectPsiRobot.py

- - - -

## Modifying System
We welcome development on the error-aware robotic system software to extend its error management capabilities and its applicability to different robots and different social signals.

For questions that are not covered in this README, we welcome developers to open an issue or contact the author at [mstiber@jhu.edu](mailto:mstiber@jhu.edu). 

- - - -

## BibTeX
If you use this robotic system in a scientific publication, please cite our work:
```
@inproceedings{stiber2023using,
  title={On Using Social Signals to Enable Flexible Error-Aware HRI},
  author={Stiber, Maia and Taylor, Russell H and Huang, Chien-Ming},
  booktitle={Proceedings of the ACM/IEEE International Conference on Human-Robot Interaction},
  year={2023}
}
```
