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

"""This modules provides a portable, failsafe notification function"""

from catkin_tools.verbs.catkin_build.common import which


def _notify_osx(title, msg):
    import os
    import subprocess

    this_dir = os.path.dirname(__file__)
    app_path = os.path.join(this_dir, 'resources', 'osx', 'catkin build.app')
    open_exec = which('open')
    subprocess.Popen([open_exec, app_path, '--args', title, msg],
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE)


def notify(title, msg):
    return _notify_osx(title, msg)
