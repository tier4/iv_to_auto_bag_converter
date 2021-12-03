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
from typing import Any, Text, Tuple

import autoware_auto_vehicle_msgs.msg as auto_vehicle_msgs
import autoware_vehicle_msgs.msg as iv_vehicle_msgs
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message, serialize_message
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
        convert_topic_list_file = os.path.join(
            get_package_share_directory("iv_to_auto_bag_converter"),
            "topic_list.yaml",
        )
        with open(convert_topic_list_file) as topic_yaml_file:
            self.__convert_dict = yaml.load(topic_yaml_file, Loader=yaml.SafeLoader)
        self.__input_qos_path = os.path.join(
            self.__input_bag_dir, qos_override_file_name
        )
        self.__output_qos_path = os.path.join(
            self.__output_bag_dir, qos_override_file_name
        )
        self.__qos_override_obj = None
        if os.path.exists(self.__input_qos_path):
            with open(self.__input_qos_path) as qos_yaml_file:
                self.__qos_override_obj = yaml.load(
                    qos_yaml_file, Loader=yaml.SafeLoader
                )

    def __create_reader(self):
        storage_options = rosbag2_py.StorageOptions(
            uri=self.__input_bag_dir, storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        return reader, storage_options, converter_options

    def __create_writer(self):
        storage_options = rosbag2_py.StorageOptions(
            uri=self.__output_bag_dir, storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)
        return writer, storage_options, converter_options

    def __convert_qos_file(self):
        if self.__qos_override_obj is not None:
            for topic_name in self.__convert_dict:
                if topic_name in self.__qos_override_obj:
                    self.__qos_override_obj[
                        self.__convert_dict[topic_name][0]
                    ] = self.__qos_override_obj.pop(topic_name)
            with open(os.path.join(self.__output_qos_path), "w") as out_qos_yaml:
                yaml.dump(self.__qos_override_obj, out_qos_yaml)

    def __convert_iv_topic(self, iv_topic_name: Text, iv_type: Any) -> Tuple[Text, Any]:
        auto_topic_name = iv_topic_name
        auto_type = iv_type

        if iv_topic_name in self.__convert_dict:
            auto_topic_name = self.__convert_dict[iv_topic_name][0]
            if iv_topic_name == "/vehicle/status/control_mode":
                # convert logic
                auto_data = auto_vehicle_msgs.ControlModeReport()
                auto_data.mode = iv_type.data
            elif iv_topic_name == "/vehicle/status/shift":
                # convert logic
                auto_data = auto_vehicle_msgs.GearReport()
                auto_data = iv_type.shift.data
            elif iv_topic_name == "/vehicle/status/steering":
                # convert logic
                auto_data = auto_vehicle_msgs.SteeringReport()
                auto_data.steering_tire_angle = iv_type.data
            elif iv_topic_name == "/vehicle/status/turn_signal":
                # convert logic
                auto_data = auto_vehicle_msgs.TurnIndicatorsReport()
                auto_data.report = iv_type.data
            elif iv_topic_name == "/vehicle/status/twist":
                auto_data = auto_vehicle_msgs.VelocityReport()
                auto_data.header.frame_id = "base_link"
                auto_data.longitudinal_velocity = iv_type.twist.linear.x
                auto_data.lateral_velocity = iv_type.twist.linear.y
                auto_data.heading_rate = iv_type.twist.angular.z
        return auto_topic_name, auto_type

    def convert(self):
        # open reader
        reader, _, _ = self.__create_reader()
        # open writer
        writer, storage_options, _ = self.__create_writer()

        # first, write topic before write rosbag
        topic_type_list = {}
        for topic_type in reader.get_all_topics_and_types():
            topic_type_list[topic_type.name] = topic_type.type
            if topic_type.name in list(self.__convert_dict.keys()):
                topic_type = rosbag2_py.TopicMetadata(
                    name=self.__convert_dict[topic_type.name][0],
                    type=self.__convert_dict[topic_type.name][1],
                    serialization_format="cdr",
                )
            writer.create_topic(topic_type)

        self.__convert_qos_file()

        # convert topic and write to output bag
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            data = deserialize_message(msg, get_message(topic_type_list[topic_name]))
            topic_name, data = self.__convert_iv_topic(topic_name, data)
            writer.write(topic_name, msg, stamp)
        # reindex to update metadata.yaml
        del writer
        rosbag2_py.Reindexer().reindex(storage_options)


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
