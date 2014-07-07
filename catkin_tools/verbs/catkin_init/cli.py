# Copyright 2014 Open Source Robotics Foundation, Inc.
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

from __future__ import print_function

import os

from catkin_pkg.packages import find_packages
from catkin_tools import metadata

def prepare_arguments(parser):
    add = parser.add_argument
    # What packages to build
    add('-f','--force', action='store_true', default=False,
            help='Overwite an existing catkin_tools metadata file if it exists.')
    add('workspace', nargs='?', default=os.getcwd(),
            help='The path to initialize as a catkin workspace. Default: current directory')

    return parser

def main(opts):
    metadata.init_metadata_file(
            opts.workspace,
            opts.force)
