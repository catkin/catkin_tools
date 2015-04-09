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

from catkin_tools.runner import run_command

from catkin_tools.utils import which

from .system_command import SystemCommand

CMAKE_EXEC = which('cmake')


class CMakeCommand(SystemCommand):
    stage_name = 'cmake'

    def __init__(self, env_loader, cmd, location):
        super(CMakeCommand, self).__init__(env_loader, cmd, location)

        if CMAKE_EXEC is None:
            raise RuntimeError("Executable 'cmake' could not be found in PATH.")
