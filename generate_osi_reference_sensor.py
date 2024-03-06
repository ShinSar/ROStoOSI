# generate_osi_messages.py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mcap.reader import make_reader

import sys
sys.path.append('/home/shinjinisarkar/Desktop/ars548-0.6.2-pre/ositrace/open-simulation-interface/')
from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_hostvehicledata_pb2 import HostVehicleData
from osi3.osi_featuredata_pb2 import LidarDetection

sys.path.append('/home/shinjinisarkar/Desktop/ReadRosbags/open-simulation-interface/format/')

from OSITrace import OSITrace
import struct
import os

import rosbag2_py
import numpy as np

# Estimates regarding the ego vehicle
MOVING_OBJECT_LENGTH = 5.0
MOVING_OBJECT_WIDTH = 1.5
MOVING_OBJECT_HEIGHT = 1.5

# which topic and message should be extracted? Can be adapted also for other msgs
TOPICS = "/sensing/gnss/pose_with_covariance"
MSG_TYPE = "geometry_msgs/PoseWithCovarianceStamped"
NAV_SAT_FIX_TOPICS = '/sensing/gnss/ublox/nav_sat_fix'
NAV_SAT_FIX_TYPE = 'sensor_msgs/msg/NavSatFix'
LIDAR_TOPICS = "/perception/object_recognition/detection/apollo/objects"
LIDAR_MSG_TYPE = "autoware_auto_perception_msgs/msg/DetectedObjects"

def read_messages(input_bag: str,selected_topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in selected_topics:
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader

def extract_data(filepath, file_out="test.osi"):

    # copied and adapted from the OSITrace python package example
    # https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/interface/architecture/trace_file_example.html
    with open(filepath, "rb") as f:
        reader = make_reader(f)

        #"""Initialize SensorView"""
        f = open(file_out, "ab")
      
        sensorview = SensorView()

        sv_ground_truth = sensorview.global_ground_truth
        sv_ground_truth.version.version_major = 3
        sv_ground_truth.version.version_minor = 5
        sv_ground_truth.version.version_patch = 0


        moving_object = sv_ground_truth.moving_object.add()
        moving_object.id.value = 0

        lidardetection = SensorView()
        lidar_gt = lidardetection.global_ground_truth
        Lidar_moving_object = lidar_gt.moving_object.add()
        Lidar_moving_object.id.value = 1
        #lidarsensorview = sensorview.LidarSensorView 
        #lidarsensorview.LidarSensorView.object_id = 0

        for schema, channel, message in reader.iter_messages(topics=[TOPICS]):
        # get the data from the msg and topic
            msg_type = get_message(MSG_TYPE)
            msg = deserialize_message(message.data, msg_type)


            # Increment the time
            sv_ground_truth.timestamp.seconds = msg.header.stamp.sec
            sv_ground_truth.timestamp.nanos = msg.header.stamp.nanosec

            sv_ground_truth.proj_string = '+proj=utm +zone=32 +ellps=WGS84 +units=m'

            moving_object.vehicle_classification.type = 2

            moving_object.base.dimension.length = MOVING_OBJECT_LENGTH
            moving_object.base.dimension.width = MOVING_OBJECT_WIDTH
            moving_object.base.dimension.height = MOVING_OBJECT_HEIGHT

            moving_object.base.position.x = msg.pose.pose.position.x
            moving_object.base.position.y = msg.pose.pose.position.y
            moving_object.base.position.z = 0.6

            moving_object.base.orientation.roll = 0.0
            moving_object.base.orientation.pitch = 0.0
            moving_object.base.orientation.yaw = 0.0

            # """Serialize"""
            # bytes_buffer = sensorview.SerializeToString()
            # f.write(struct.pack("<L", len(bytes_buffer)))
            # f.write(bytes_buffer)

        for schema, channel, message in reader.iter_messages(topics=[NAV_SAT_FIX_TOPICS]):

            nav_sat_fix_type = get_message(NAV_SAT_FIX_TYPE)
            nav_sat_fix_msg = deserialize_message(message.data, nav_sat_fix_type)
        
            host_vehicle_data = sensorview.host_vehicle_data
            hvd_vehicle_localization = host_vehicle_data.vehicle_localization
            geodetic_position = hvd_vehicle_localization.geodetic_position 
            geodetic_position.longitude = nav_sat_fix_msg.longitude
            geodetic_position.latitude = nav_sat_fix_msg.latitude
            geodetic_position.altitude = nav_sat_fix_msg.altitude

           # """Serialize"""
            bytes_buffer = sensorview.SerializeToString()
            f.write(struct.pack("<L", len(bytes_buffer)))
            f.write(bytes_buffer)

        for schema, channel, message in reader.iter_messages(topics=[LIDAR_TOPICS]):

            lidar_msg_type = get_message(LIDAR_MSG_TYPE)
            lidar_msg = deserialize_message(message.data, lidar_msg_type)

                       
            lidardetection.timestamp.seconds = lidar_msg.header.stamp._sec
            lidardetection.timestamp.nanos = lidar_msg.header.stamp._nanosec
            lidardetection.proj_string = '+proj=utm +zone=32 +ellps=WGS84 +units=m'

            Lidar_moving_object.vehicle_classification.type = 2
            
            Lidar_moving_object.base.position.x = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.position.x
            Lidar_moving_object.base.position.y = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.position.y
            Lidar_moving_object.base.position.z = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.position.z   
            
            Lidar_moving_object.base.orientation.roll = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.orientation._x
            Lidar_moving_object.base.orientation.pitch = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.orientation._y
            Lidar_moving_object.base.orientation.yaw = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.orientation._z 
            #Lidar_moving_object.base.orientation.w = lidar_msg.objects[0].kinematics.pose_with_covariance.pose.orientation._w
            
            bytes_buffer = lidardetection.SerializeToString()
            f.write(struct.pack("<L", len(bytes_buffer)))
            f.write(bytes_buffer)
            
            
           
    f.close()


if __name__ == "__main__":

    directory = 'gaiax_2023_06_19-13_29_23'
    #mcap_file_path = 'gaiax_2023_06_19-13_29_23/gaiax_2023_06_19-13_29_23_0.mcap'
    #object_msgs = np.array([msg for topic, msg, timestamp in read_messages(mcap_file_path, ['/perception/object_recognition/detection/apollo/objects'])])
    files = os.listdir(directory)
    files = [file for file in files if '.mcap' in file]

    # loop over all mcap files in the directory and create osi and human readable text files
    for file in files:
        filepath = directory + "/" + file

        file_out = directory + file[:-4] + "osi"
        extract_data(filepath, file_out)
        print(file_out)
        
        trace = OSITrace()
        trace.from_file(path=file_out)
        trace.make_readable(file_out[:-3] + "txth")
        trace.scenario_file.close()