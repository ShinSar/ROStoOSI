# ROStoOSI
## Description
This repository is used for the generation of OSI-traces from ROSBags

ROSBags are converted into OSI traces following the ASAM OSI standard ( https://opensimulationinterface.github.io/osi-documentation/#top-osi_trace_file_formats )
Part of this script has been copied and adapted from the OSITrace python package example (https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/interface/architecture/trace_file_example.html)
A sample mcap (ROSBag) file is provided for checking out the script. In this example, ARS548 Radar, Lidr and camera was used to record the surroundings in a cut-in scenario. This script is used to extract the groundtruth data from the sensor detections of the ego vehicle and the target vehicle.

An extra script is provided which can be used for reading the OSI traces. 

## Pre-requisites
1. Python 3.8
2. OS: Ubuntu 20.04 LTS
3. CUDA: 11.3
4. cuDNN: 8
5. Docker: 20.10
6. NVIDIA Container Toolkit
3. ROS Module installed (preferrably ros galactic) https://docs.ros.org/en/galactic/Installation.html
4. Autoware- Module installed (https://github.com/tier4/autoware_auto_msgs/tree/tier4/main)
5. Open-Simulation-Interface https://github.com/OpenSimulationInterface/open-simulation-interface
