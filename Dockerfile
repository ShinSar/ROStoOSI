FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

RUN apt-get install python3.8 -y
RUN apt install python3-pip -y
RUN apt-get install python3.8-venv -y

# Add 3.8 to the available alternatives
# RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1

# Set python3.8 as the default python
# RUN update-alternatives --set python /usr/bin/python3.8

RUN python3.8 -m pip install --upgrade pip

# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# # Done with python3.8 and pip

# Installing OSI

WORKDIR /
RUN git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git
WORKDIR /open-simulation-interface

WORKDIR /

#Installing ROS2 Galactic

RUN docker pull osrf/ros:galactic-desktop

# Run the ros2osi script 

RUN python3 generate_osi_reference_sensor.py 