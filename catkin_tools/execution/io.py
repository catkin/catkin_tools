# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import re
import shutil
from glob import glob

from osrf_pycommon.process_utils import AsyncSubprocessProtocol

from catkin_tools.common import mkdir_p

from catkin_tools.terminal_color import fmt

from .events import ExecutionEvent

MAX_LOGFILE_HISTORY = 10

if type(u'') == str:
    def _encode(string):
        """Encode a Python 3 str into bytes.
        :type data: str
        """
        return string.encode('utf-8')
else:
    def _encode(string):
        """Encode a Python 2 str into bytes.
        :type data: str
        """
        return string.decode('utf-8').encode('utf-8')


class IOBufferContainer(object):

    """A simple buffer container for use in logging.

    This class will open a logfile for a given job stage and write to it
    continuously while receiving stdout and stderr.
    """

    def __init__(self, label, job_id, stage_label, event_queue, log_path):
        self.label = label
        self.job_id = job_id
        self.stage_label = stage_label
        self.event_queue = event_queue
        self.log_path = log_path

        self.is_open = False
        self.stdout_buffer = b""
        self.stderr_buffer = b""
        self.interleaved_buffer = b""

        # Construct the logfile path for this job and stage
        logfile_dir_path = os.path.join(log_path, self.job_id)
        self.logfile_basename = os.path.join(logfile_dir_path, '.'.join([self.label, self.stage_label]))
        self.logfile_name = '{}.log'.format(self.logfile_basename)

        # Create the logfile dir if it doesn't exist
        if not os.path.exists(logfile_dir_path):
            mkdir_p(logfile_dir_path)

        # Get the existing number of logfiles
        # TODO: Make this number global across all build stages
        existing_logfile_indices = sorted([int(lf.split('.')[-2])
                                           for lf in glob('{}.*.log'.format(self.logfile_basename))])
        if len(existing_logfile_indices) == 0:
            self.logfile_index = 0
        else:
            self.logfile_index = 1 + existing_logfile_indices[-1]

        # Generate the logfile name
        self.unique_logfile_name = '{}.{:0>{}}.log'.format(self.logfile_basename, self.logfile_index, 3)

        # Remove colliding file if necessary
        if os.path.exists(self.logfile_name):
            os.unlink(self.logfile_name)

        # Open logfile
        self.log_file = open(self.logfile_name, 'wb')
        self.is_open = True

    def close(self):
        # Close logfile
        self.log_file.close()
        self.is_open = False

        # Copy logfile to unique name
        shutil.copy(self.logfile_name, self.unique_logfile_name)

        # Remove older logfiles
        for logfile_name in glob('{}.*.log'.format(self.logfile_basename)):
            if (self.logfile_index - int(logfile_name.split('.')[-2])) >= MAX_LOGFILE_HISTORY:
                os.unlink(logfile_name)

        # Save output from stderr (these don't get deleted until cleaning the logfile directory)
        if len(self.stderr_buffer) > 0:
            with open(self.unique_logfile_name + '.stderr', 'wb') as logfile:
                logfile.write(self.stderr_buffer)

    def get_interleaved_log(self):
        """get decoded interleaved log."""
        try:
            return self._decode(self.interleaved_buffer)
        except UnicodeDecodeError:
            return "interleaved_log: some output cannot be displayed.\n"

    def get_stdout_log(self):
        """get decoded stdout log."""
        try:
            return self._decode(self.stdout_buffer)
        except UnicodeDecodeError:
            return "stdout_log: some output cannot be displayed.\n"

    def get_stderr_log(self):
        """get decoded stderr log."""
        try:
            return self._decode(self.stderr_buffer)
        except UnicodeDecodeError:
            return "stderr_log: some output cannot be displayed.\n"

    def _encode(self, data):
        """Encode a Python str into bytes.
        :type data: str
        """
        return _encode(data)

    def _decode(self, data):
        """Decode bytes into Python str.
        :type data: bytes
        """
        return data.decode('utf-8', 'replace')

    def __del__(self):
        if self.is_open:
            self.close()

    @classmethod
    def factory(cls, label, job_id, stage_label, event_queue, log_path):
        """Factory method for constructing with job metadata."""

        def init_proxy(*args, **kwargs):
            return cls(label, job_id, stage_label, event_queue, log_path, *args, **kwargs)

        return init_proxy


class IOBufferLogger(IOBufferContainer):

    """This is a logging class to be used instead of sys.stdout and sys.stderr
    in FunctionStage operations.

    This class also generates `stdout` and `stderr` events.
    """

    def __init__(self, label, job_id, stage_label, event_queue, log_path, *args, **kwargs):
        IOBufferContainer.__init__(self, label, job_id, stage_label, event_queue, log_path)

    def out(self, data, end='\n'):
        """
        :type data: str
        """
        # Buffer the encoded data
        data += end
        encoded_data = self._encode(data)
        self.stdout_buffer += encoded_data
        self.interleaved_buffer += encoded_data

        # Save the encoded data
        self.log_file.write(encoded_data)

        # Emit event with decoded Python str
        self.event_queue.put(ExecutionEvent(
            'STDOUT',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))

    def err(self, data, end='\n'):
        """
        :type data: str
        """
        # Buffer the encoded data
        data += end
        encoded_data = self._encode(data)
        self.stderr_buffer += encoded_data
        self.interleaved_buffer += encoded_data

        # Save the encoded data
        self.log_file.write(encoded_data)

        # Emit event with decoded Python str
        self.event_queue.put(ExecutionEvent(
            'STDERR',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))


class IOBufferProtocol(IOBufferContainer, AsyncSubprocessProtocol):

    """An asyncio protocol that collects stdout and stderr.

    This class also generates `stdout` and `stderr` events.

    Since the underlying asyncio API constructs the actual protocols, this
    class provides a factory method to inject the job and stage information
    into the created protocol.
    """

    def __init__(self, label, job_id, stage_label, event_queue, log_path, *args, **kwargs):
        IOBufferContainer.__init__(self, label, job_id, stage_label, event_queue, log_path)
        AsyncSubprocessProtocol.__init__(self, *args, **kwargs)

        self.intermediate_stdout_buffer = b''
        self.intermediate_stderr_buffer = b''

    def _split(self, data):
        try:
            last_break = data.rindex(b'\n') + 1
            return data[0:last_break], data[last_break:]
        except ValueError:
            return b'', data

    def on_stdout_received(self, data):
        """
        :type data: encoded bytes
        """

        data, self.intermediate_stdout_buffer = self._split(self.intermediate_stdout_buffer + data)

        self.stdout_buffer += data
        self.interleaved_buffer += data
        self.log_file.write(data)

        # Get the decoded Python str
        decoded_data = self._decode(data)

        # Emit event with decoded Python str
        self.event_queue.put(ExecutionEvent(
            'STDOUT',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=decoded_data))

    def on_stderr_received(self, data):
        """
        :type data: encoded bytes
        """

        data, self.intermediate_stderr_buffer = self._split(self.intermediate_stderr_buffer + data)

        self.stderr_buffer += data
        self.interleaved_buffer += data
        self.log_file.write(data)

        # Get the decoded Python str
        decoded_data = self._decode(data)

        # Emit event with decoded Python str
        self.event_queue.put(ExecutionEvent(
            'STDERR',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=decoded_data))

    def on_process_exited2(self, returncode):
        """
        Dump anything remaining in the intermediate buffers.
        """

        if len(self.intermediate_stdout_buffer) > 0:
            self.on_stdout_received(self.intermediate_stdout_buffer + b'\n')
        if len(self.intermediate_stderr_buffer) > 0:
            self.on_stderr_received(self.intermediate_stderr_buffer + b'\n')


class CatkinTestResultsIOBufferProtocol(IOBufferProtocol):
    """An IOBufferProtocol which parses the output of catkin_test_results"""
    def on_stdout_received(self, data):
        lines = data.decode().splitlines()
        clines = []
        for line in lines:
            match = re.match(r'(.*): (\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped', line)
            if match:
                line = fmt('@!{}@|: {} tests, @{rf}{} errors@|, @{rf}{} failures@|, @{kf}{} skipped@|')
                line = line.format(*match.groups())
            clines.append(line)

        cdata = '\n'.join(clines) + '\n'

        super(CatkinTestResultsIOBufferProtocol, self).on_stdout_received(cdata.encode())
