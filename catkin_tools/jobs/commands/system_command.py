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

import os

from catkin_tools.runner import run_command

from .command import Command


class SystemCommand(Command):

    """Stage of a job which executes a system command"""

    def __init__(self, env_loader, cmd, location):
        super(SystemCommand, self).__init__(location)
        self.cmd = [env_loader] + cmd if env_loader else cmd
        self.cmd_str = ' '.join(self.cmd)
        self.executable = os.path.basename(cmd[0])
        self.pretty = ' '.join([self.executable] + cmd[1:])
        self.plain_cmd = cmd
        self.plain_cmd_str = ' '.join(self.plain_cmd)
        self.env_loader = env_loader

    def run(self):
        return run_command(self.cmd, cwd=self.location)
