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

from catkin_tools.argument_parsing import add_workspace_arg

from catkin_tools.context import Context

from catkin_tools.metadata import init_metadata_root


def prepare_arguments(parser):
    # Workspace / profile args
    add_workspace_arg(parser)

    add = parser.add_argument

    add('--reset', action='store_true', default=False,
        help='Reset (delete) all of the metadata for the given workspace.')

    return parser


def main(opts):
    try:
        # Load a context with initialization
        ctx = Context.load(opts.workspace, strict=True)

        # Initialize the workspace if necessary
        if ctx:
            print('Catkin workspace `%s` is already initialized. No action taken.' % (ctx.workspace))
        else:
            print('Initializing catkin workspace in `%s`.' % (opts.workspace or os.getcwd()))
            # initialize the workspace
            init_metadata_root(
                opts.workspace or os.getcwd(),
                opts.reset)

        ctx = Context.load(opts.workspace)
        print(ctx.summary())

    except IOError as exc:
        # Usually happens if workspace is already underneath another catkin_tools workspace
        print('error: could not initialize catkin workspace: %s' % exc.message)
        return 1

    return 0
