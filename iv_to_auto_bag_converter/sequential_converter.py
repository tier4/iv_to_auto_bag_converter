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
import glob
import os
import shutil
from pathlib import Path

from iv_to_auto_bag_converter.converter import AutoBagConverter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("iv_bag_directory", help="input .iv bag path")
    parser.add_argument(
        "-q",
        "--qos_override_file_name",
        default="qos_override.yaml",
        help="file name of qos override",
    )
    parser.add_argument(
        "-d",
        "--delete",
        action="store_true",
        help="delete input bag and set the name of the converted bag to be the same as before the conversion",
    )
    args = parser.parse_args()

    regex = os.path.join(os.path.expandvars(args.iv_bag_directory), "**", "*.db3")
    bag_paths = glob.glob(regex, recursive=True)
    for db3_path_str in bag_paths:
        input_bag_dir = Path(db3_path_str).parent
        output_bag_dir = input_bag_dir.parent.joinpath(
            "converted_" + input_bag_dir.name
        )
        print("convert " + input_bag_dir.as_posix())
        converter = AutoBagConverter(
            input_bag_dir.as_posix(),
            output_bag_dir.as_posix(),
            args.qos_override_file_name,
        )
        converter.convert()
        if args.delete:
            shutil.rmtree(input_bag_dir.as_posix())
            output_bag_dir.rename(input_bag_dir)


if __name__ == "__main__":
    main()
