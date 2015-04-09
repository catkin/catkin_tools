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

from .command import Command


class PythonCommand(Command):

    """Stage of a job implemented in python"""

    def __init__(self, function, kwargs, location):
        super(PythonCommand, self).__init__(location)
        self.function = function
        self.kwargs = kwargs
        self.cmd = 'py'
        self.cmd_str = str(function) + ": " + str(kwargs)

    def run(self):
        return [self.function(**self.kwargs)]
