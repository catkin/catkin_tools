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

    add('--workspace', '-w', default=None,
        help='The path to initialize as a catkin workspace. Default: current working directory')
    add('--reset', action='store_true', default=False,
        help='Reset (delete) the metadata for the given workspace.')

    return parser


def main(opts):
    try:
        # Check if the workspace is initialized
        if not opts.workspace:
            opts.workspace = os.getcwd()
        marked_workspace = metadata.find_enclosing_workspace(opts.workspace)

        # Initialize the workspace if necessary
        if marked_workspace == opts.workspace and not opts.reset:
            print('Catkin workspace %s is already initialized.' % (opts.workspace))
        else:
            # initialize the workspace
            metadata.init_metadata_dir(
                opts.workspace,
                opts.reset)
    except IOError as exc:
        # Usually happens if workspace is already underneath another catkin_tools workspace
        print('error: could not initialize catkin workspace: %s' % exc.message)
        return 1

    return 0
