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
import pty
import select
import sys
import time

from subprocess import Popen
from subprocess import STDOUT, PIPE

from run_common import SOURCE_STDOUT, SOURCE_STDERR

def process_incomming_lines(lines, left_over):
    if not lines:
        return None, left_over
    if lines[-1].endswith('\n'):
        data = b''.join(lines)
        left_over = b''
    else:
        data = b''.join(lines[:-1])
        left_over = lines[-1]
    return data, left_over


def run_command(cmd, cwd=None):
    master, slave = pty.openpty()

    p = None
    while p is None:
        try:
            p = Popen(cmd, stdin=slave, stdout=slave, stderr=PIPE, cwd=cwd)
        except OSError as exc:
            if 'Text file busy' in str(exc):
                # This is a transient error, try again shortly
                time.sleep(0.01)
                continue
            raise
    if sys.platform.startswith('darwin'):
        os.close(slave)  # This causes the below select to exit when the subprocess closes

    left_over = b''
    left_over_stderr = b''

    # Read data until the process is finished
    while p.poll() is None:
        incomming = left_over
        incomming_stderr = left_over_stderr
        rlist, wlist, xlist = select.select([master, p.stderr], [], [], 0.1)
        if p.stderr in rlist:
            incomming_stderr += os.read(p.stderr.fileno(), 1024)
            lines = incomming_stderr.splitlines(True)  # keepends=True
            data, left_over_stderr = process_incomming_lines(lines, left_over_stderr)
            if not data is None:
                yield SOURCE_STDERR, data
        if master in rlist:
            incomming += os.read(master, 1024)
            lines = incomming.splitlines(True)  # keepends=True
            data, left_over = process_incomming_lines(lines, left_over)
            if not data is None:
                yield SOURCE_STDOUT, data


    # Done
    os.close(master)
    yield p.returncode
