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
import re

from catkin_tools.execution.io import IOBufferProtocol
from catkin_tools.execution.events import ExecutionEvent

from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import sanitize

from catkin_tools.utils import which

CMAKE_EXEC = which('cmake')
CMAKE_INSTALL_MANIFEST_FILENAME = 'install_manifest.txt'


def split_to_last_line_break(data):
    """This splits a byte buffer into (head, tail) where head contains the
    beginning of the buffer to the last line break (inclusive) and the tail
    contains all bytes after that."""
    last_break_index = 1 + data.rfind(b'\n')
    return data[:last_break_index], data[last_break_index:]


class CMakeIOBufferProtocol(IOBufferProtocol):

    """An asyncio protocol that collects stdout and stderr.

    This class also generates `stdout` and `stderr` events.

    Since the underlying asyncio API constructs the actual protocols, this
    class provides a factory method to inject the job and stage information
    into the created protocol.
    """

    def abspath(self, groups):
        """Group filter that turns source-relative paths into absolute paths."""
        return (groups[0] if groups[0].startswith(os.sep) else os.path.join(self.source_path, groups[0]),) + groups[1:]

    def __init__(self, label, job_id, stage_label, event_queue, log_path, source_path, *args, **kwargs):
        super(CMakeIOBufferProtocol, self).__init__(label, job_id, stage_label, event_queue, log_path, *args, **kwargs)
        self.source_path = source_path

        # These are buffers for incomplete lines that we want to wait to parse
        # until we have received them completely
        self.stdout_tail = b''
        self.stderr_tail = b''

        # Line formatting filters
        # Each is a 3-tuple:
        #  - regular expression (with captured groups)
        #  - output formatting line (subs captured groups)
        #  - functor which filters captured groups
        filters = [
            ('^-- :(.+)', '@{cf}--@| :@{yf}{}@|', None),
            ('^-- (.+)', '@{cf}--@| {}', None),
            ('CMake Error at (.+):(.+)', '@{rf}@!CMake Error@| at {}:{}', self.abspath),
            ('CMake Warning at (.+):(.+)', '@{yf}@!CMake Warning@| at {}:{}', self.abspath),
            ('CMake Warning (dev) at (.+):(.+)', '@{yf}@!CMake Warning (dev)@| at {}:{}', self.abspath),
            ('(?i)(warning.*)', '@(yf){}@|', None),
            ('(?i)ERROR:(.*)', '@!@(rf)ERROR:@|{}@|', None),
            ('Call Stack \(most recent call first\):(.*)', '@{cf}Call Stack (most recent call first):@|{}', None),
        ]

        self.filters = [(re.compile(p), r, f) for (p, r, f) in filters]

    def on_stdout_received(self, data):
        data_head, self.stdout_tail = split_to_last_line_break(self.stdout_tail + data)
        colored = self.color_lines(data_head)
        super(CMakeIOBufferProtocol, self).on_stdout_received(colored)

    def on_stderr_received(self, data):
        data_head, self.stderr_tail = split_to_last_line_break(self.stderr_tail + data)
        colored = self.color_lines(data_head)
        super(CMakeIOBufferProtocol, self).on_stderr_received(colored)

    def close(self):
        # Make sure tail buffers are flushed
        self.flush_tails()
        super(CMakeIOBufferProtocol, self).close()

    def flush_tails(self):
        """Write out any unprocessed tail buffers."""

        colored = self.color_lines(self.stdout_tail)
        super(CMakeIOBufferProtocol, self).on_stdout_received(colored)
        self.stdout_tail = b''

        colored = self.color_lines(self.stderr_tail)
        super(CMakeIOBufferProtocol, self).on_stderr_received(colored)
        self.stderr_tail = b''

    def color_lines(self, data):
        """Apply colorization rules to each line in data"""
        decoded_data = self._decode(data)
        # TODO: This will only work if all lines are received at once. Instead
        # of direclty splitting lines, we should buffer the data lines until
        # the last character is a line break
        lines = decoded_data.splitlines(True)  # Keep line breaks
        colored_lines = [self.colorize_cmake(l) for l in lines]
        colored_data = ''.join(colored_lines)
        encoded_data = self._encode(colored_data)
        return encoded_data

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
        # return line
        cline = sanitize(line).rstrip()

        if len(cline.strip()) > 0:
            for p, r, f in self.filters:
                match = p.match(cline)
                if match is not None:
                    cline = fmt(r, reset=False)
                    if f is not None:
                        cline = cline.format(*f(match.groups()))
                    else:
                        cline = cline.format(*match.groups())
                    break

        return cline + '\r\n'


class CMakeMakeIOBufferProtocol(IOBufferProtocol):

    """An IOBufferProtocol which parses CMake's progree prefixes and emits corresponding STAGE_PROGRESS events."""

    def __init__(self, label, job_id, stage_label, event_queue, log_path, *args, **kwargs):
        super(CMakeMakeIOBufferProtocol, self).__init__(
            label, job_id, stage_label, event_queue, log_path, *args, **kwargs)

    def on_stdout_received(self, data):
        super(CMakeMakeIOBufferProtocol, self).on_stdout_received(data)

        # Parse CMake Make completion progress
        progress_matches = re.match('\[\s*([0-9]+)%\]', self._decode(data))
        if progress_matches is not None:
            self.event_queue.put(ExecutionEvent(
                'STAGE_PROGRESS',
                job_id=self.job_id,
                stage_label=self.stage_label,
                percent=str(progress_matches.groups()[0])))


def get_installed_files(path):
    """Get a set of files installed by a CMake package as specified by an
    install_manifest.txt in a given directory."""

    install_manifest_path = os.path.join(
        path,
        CMAKE_INSTALL_MANIFEST_FILENAME)
    installed_files = set()
    if os.path.exists(install_manifest_path):
        with open(install_manifest_path) as f:
            installed_files = set([line.strip() for line in f.readlines()])
    return installed_files
