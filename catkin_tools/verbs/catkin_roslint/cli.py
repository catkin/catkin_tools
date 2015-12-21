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

from catkin_tools.verbs.catkin_build import main as build_main
from catkin_tools.verbs.catkin_build \
    import prepare_arguments as build_prepare_arguments


# use same prepare_arguments as catkin_build
prepare_arguments = build_prepare_arguments


def main(opts):
    if opts.catkin_make_args is None:
        opts.catkin_make_args = ['roslint']
    else:
        opts.catkin_make_args.append('roslint')

    build_main(opts)
