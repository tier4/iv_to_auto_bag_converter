# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
from typing import Text

import yaml
from ament_index_python.packages import get_package_share_directory
from autoware_auto_vehicle_msgs.msg import *
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import *
from rosidl_runtime_py.utilities import get_message


class AutoBagConverter:
    def __init__(
        self,
        input_bag_dir: Text,
        output_bag_dir: Text,
        qos_override_file_name: Text,
    ):
        self.__input_bag_dir = input_bag_dir
        self.__output_bag_dir = output_bag_dir
        self.__qos_override_file_name = qos_override_file_name
        self.__topic_list_file = os.path.join(
            get_package_share_directory("iv_to_auto_bag_converter"),
            "topic_list.yaml",
        )

    def __create_reader(self):
        storage_options = StorageOptions(uri=self.__input_bag_dir, storage_id="sqlite3")
        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        return reader, storage_options, converter_options

    def __create_writer(self):
        storage_options = StorageOptions(
            uri=self.__output_bag_dir, storage_id="sqlite3"
        )
        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        writer = SequentialWriter()
        writer.open(storage_options, converter_options)
        return writer, storage_options, converter_options

    def __convert_qos_file(self, autoware_auto_msgs_list):
        # load qos setting yaml and convert.
        with open(
            os.path.join(self.__input_bag_dir, self.__qos_override_file_name)
        ) as qos_yaml:
            qos_setting = yaml.load(qos_yaml, Loader=yaml.SafeLoader)
        for topic_name in autoware_auto_msgs_list:
            if topic_name in qos_setting:
                qos_setting[autoware_auto_msgs_list[topic_name][0]] = qos_setting.pop(
                    topic_name
                )
        with open(
            os.path.join(self.__output_bag_dir, self.__qos_override_file_name), "w"
        ) as override_qos_yaml:
            yaml.dump(qos_setting, override_qos_yaml)

    def convert(self):
        # open reader
        reader, _, _ = self.__create_reader()
        # open writer
        writer, storage_options, _ = self.__create_writer()

        # first, write topic before write rosbag
        topic_type_list = {}
        with open(self.__topic_list_file) as yaml_file:
            autoware_auto_msgs_list = yaml.load(yaml_file, Loader=yaml.SafeLoader)
        for topic_type in reader.get_all_topics_and_types():
            topic_type_list[topic_type.name] = topic_type.type
            if topic_type.name in list(autoware_auto_msgs_list.keys()):
                topic_type = TopicMetadata(
                    name=autoware_auto_msgs_list[topic_type.name][0],
                    type=autoware_auto_msgs_list[topic_type.name][1],
                    serialization_format="cdr",
                )
                writer.create_topic(topic_type)
            writer.create_topic(topic_type)

        if os.path.exists(
            os.path.join(self.__input_bag_dir, self.__qos_override_file_name)
        ):
            self.__convert_qos_file(autoware_auto_msgs_list)

        # convert topic and write to rosbag.
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            data = deserialize_message(msg, get_message(topic_type_list[topic_name]))

            if topic_name == "/vehicle/status/control_mode":
                control_mode = ControlModeReport()
                control_mode.mode = data.data
                writer.write(
                    autoware_auto_msgs_list[topic_name][0],
                    serialize_message(control_mode),
                    stamp,
                )
            elif topic_name == "/vehicle/status/shift":
                gear_report = GearReport()
                gear_report.report = data.shift.data
                writer.write(
                    autoware_auto_msgs_list[topic_name][0],
                    serialize_message(gear_report),
                    stamp,
                )
            elif topic_name == "/vehicle/status/steering":
                steering_report = SteeringReport()
                steering_report.steering_tire_angle = data.data
                writer.write(
                    autoware_auto_msgs_list[topic_name][0],
                    serialize_message(steering_report),
                    stamp,
                )
            elif topic_name == "/vehicle/status/turn_signal":
                turn_indicators_report = TurnIndicatorsReport()
                turn_indicators_report.report = data.data
                writer.write(
                    autoware_auto_msgs_list[topic_name][0],
                    serialize_message(turn_indicators_report),
                    stamp,
                )
            elif topic_name == "/vehicle/status/twist":
                velocity_report = VelocityReport()
                velocity_report.header.frame_id = "base_link"
                velocity_report.longitudinal_velocity = data.twist.linear.x
                velocity_report.lateral_velocity = data.twist.linear.y
                velocity_report.heading_rate = data.twist.angular.z
                writer.write(
                    autoware_auto_msgs_list[topic_name][0],
                    serialize_message(velocity_report),
                    stamp,
                )
            else:
                writer.write(topic_name, msg, stamp)
        del writer
        Reindexer().reindex(storage_options)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="input .iv bag path")
    parser.add_argument("output", help="output .auto bag path")
    parser.add_argument(
        "-q",
        "--qos_override_file_name",
        default="qos_override.yaml",
        help="file name of qos override",
    )
    args = parser.parse_args()

    converter = AutoBagConverter(
        os.path.expandvars(args.input),
        os.path.expandvars(args.output),
        args.qos_override_file_name,
    )
    converter.convert()


if __name__ == "__main__":
    main()
