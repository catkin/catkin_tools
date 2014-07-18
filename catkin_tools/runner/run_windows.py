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
import select

from subprocess import PIPE
from subprocess import Popen
from subprocess import STDOUT


def run_command(cmd, cwd=None):
    p = Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT, cwd=cwd)

    left_over = b''

    while p.poll() is None:
        incomming = left_over
        rlist, wlist, xlist = select.select([p.stdout], [], [])
        if rlist:
            incomming += os.read(p.stdout, 1024)
            lines = incomming.splitlines(True)  # keepends=True
            if not lines:
                continue
            if lines[-1].endswith('\n'):
                data = b''.join(lines)
                left_over = b''
            else:
                data = b''.join(lines[-1])
                left_over = lines[-1]
            yield data
    # Done
    yield p.returncode
