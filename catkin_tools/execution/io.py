
import os
import re
import shutil

from glob import glob

from osrf_pycommon.process_utils import AsyncSubprocessProtocol

from catkin_tools.common import mkdir_p

from .events import ExecutionEvent


MAX_LOGFILE_HISTORY = 10


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

    def out(self, data):
        """
        :type data: str
        """
        encoded_data = data.encode('utf-8')
        self.stdout_buffer += encoded_data
        self.interleaved_buffer += encoded_data

        self.event_queue.put(ExecutionEvent(
            'STDOUT',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))

        self.log_file.write(encoded_data)

    def err(self, data):
        """
        :type data: str
        """
        encoded_data = data.encode('utf-8')
        self.stderr_buffer += encoded_data
        self.interleaved_buffer += encoded_data

        self.event_queue.put(ExecutionEvent(
            'STDERR',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))

        self.log_file.write(encoded_data)


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

    def on_stdout_received(self, data):
        """
        :type data: utf-8 encoded bytes
        """
        decoded_data = data.decode('utf-8')

        self.stdout_buffer += data
        self.interleaved_buffer += data

        progress_matches = re.match('\[\s*([0-9]+)%\]', decoded_data)
        if progress_matches is not None:
            self.event_queue.put(ExecutionEvent(
                'STAGE_PROGRESS',
                job_id=self.job_id,
                stage_label=self.stage_label,
                percent=str(progress_matches.groups()[0])))

        self.event_queue.put(ExecutionEvent(
            'STDOUT',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=decoded_data))

        self.log_file.write(data)

    def on_stderr_received(self, data):
        """
        :type data: utf-8 encoded bytes
        """
        decoded_data = data.decode('utf-8')

        self.stderr_buffer += data
        self.interleaved_buffer += data

        self.event_queue.put(ExecutionEvent(
            'STDERR',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=decoded_data))

        self.log_file.write(data)
