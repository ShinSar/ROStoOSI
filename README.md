# ROStoOSI
## Description
This repository is used Generation of OSI-traces from ROSBags

ROSBags are converted into OSI traces following the ASAM OSI standard ( https://opensimulationinterface.github.io/osi-documentation/#top-osi_trace_file_formats )
Part of this script has been copied and adapted from the OSITrace python package example (https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/interface/architecture/trace_file_example.html)
A sample mcap (ROSBag) file is provided for checking out the script. In this example, ARS548 Radar, Lidr and camera was used to record the surroundings in a cut-in scenario. This script is used to extract the groundtruth data from the sensor detections of the ego vehicle and the target vehicle.

An extra script is provided which can be used for reading the OSI traces. 
## How to get started?
Dependencies needed to run the script:
1. ROS Module installed (preferrably ros galactic) https://docs.ros.org/en/galactic/Installation.html
2. Python 3.8
3. Open-Simulation-Interface https://github.com/OpenSimulationInterface/open-simulation-interface
