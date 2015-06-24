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

from catkin_tools.execution.io import IOBufferProtocol

from catkin_tools.terminal_color import ansi
from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import sanitize

from catkin_tools.utils import which

CMAKE_EXEC = which('cmake')


class CMakeIOBufferProtocol(IOBufferProtocol):

    """An asyncio protocol that collects stdout and stderr.

    This class also generates `stdout` and `stderr` events.

    Since the underlying asyncio API constructs the actual protocols, this
    class provides a factory method to inject the job and stage information
    into the created protocol.
    """

    def __init__(self, label, job_id, stage_label, event_queue, log_path, source_path, *args, **kwargs):
        super(CMakeIOBufferProtocol, self).__init__(label, job_id, stage_label, event_queue, log_path, *args, **kwargs)
        self.source_path = source_path

    def on_stdout_received(self, data):
        colored = self.color_lines(data)
        super(CMakeIOBufferProtocol, self).on_stdout_received(colored)

    def on_stderr_received(self, data):
        colored = self.color_lines(data)
        super(CMakeIOBufferProtocol, self).on_stderr_received(colored)

    def color_lines(self, data):
        """Apply colorization rules to each line in data"""
        lines = data.decode('utf-8').splitlines()
        return (''.join([self.colorize_cmake(l) for l in lines if len(l) > 0])).encode('utf-8')

    @classmethod
    def factory_factory(cls, source_path):
        """Factory factory for constructing protocols that know the source path for this CMake package."""
        def factory(label, job_id, stage_label, event_queue, log_path):
            # factory is called by caktin_tools executor
            def init_proxy(*args, **kwargs):
                # init_proxy is called by asyncio
                return cls(label, job_id, stage_label, event_queue, log_path, source_path, *args, **kwargs)
            return init_proxy
        return factory

    def colorize_cmake(self, line):
        """Colorizes output from CMake

        This also prepends the source path to the locations of warnings and errors.

        :param line: one, new line terminated, line from `cmake` which needs coloring.
        :type line: str
        """
        cline = sanitize(line)
        if line.startswith('-- '):
            cline = '@{cf}-- @|' + cline[len('-- '):]
            if ':' in cline:
                split_cline = cline.split(':', 1)
                cline = '%s:@{yf}%s@|' % (split_cline[0], split_cline[1])
        elif line.lower().startswith('warning'):
            # WARNING
            cline = fmt('@{yf}') + cline
        elif line.startswith('CMake Warning at '):
            # CMake Error...
            cline = cline.replace('CMake Warning at ', '@{yf}@!CMake Warning@| at ' + self.source_path + os.path.sep)
        elif line.startswith('CMake Warning'):
            # CMake Warning...
            cline = cline.replace('CMake Warning', '@{yf}@!CMake Warning@|')
        elif line.startswith('ERROR:'):
            # ERROR:
            cline = cline.replace('ERROR:', '@!@{rf}ERROR:@|')
        elif line.startswith('CMake Error at '):
            # CMake Error...
            cline = cline.replace('CMake Error at ', '@{rf}@!CMake Error@| at ' + self.source_path + os.path.sep)
        elif line.startswith('CMake Error'):
            # CMake Error...
            cline = cline.replace('CMake Error', '@{rf}@!CMake Error@|')
        elif line.startswith('Call Stack (most recent call first):'):
            # CMake Call Stack
            cline = cline.replace('Call Stack (most recent call first):',
                                  '@{cf}@_Call Stack (most recent call first):@|')
        return fmt(cline) + '\n'
