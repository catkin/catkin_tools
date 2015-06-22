
import os
import re

from glob import glob

from osrf_pycommon.process_utils import AsyncSubprocessProtocol

from catkin_tools.common import mkdir_p

from .events import ExecutionEvent


class IOBufferContainer(object):

    """A simple buffer container for use in logging."""

    def __init__(self, label, job_id, stage_label, event_queue):
        self.label = label
        self.job_id = job_id
        self.stage_label = stage_label
        self.event_queue = event_queue

        self.stdout_buffer = b""
        self.stderr_buffer = b""
        self.interleaved_buffer = b""

    def save(self, path):
        # Get the existing number of logfiles
        logfile_dir_path = os.path.join(path, self.job_id)
        logfile_basename = os.path.join(logfile_dir_path, '.'.join([self.label, self.stage_label]))
        logfile_link_name = '{}.out'.format(logfile_basename)
        logfile_err_link_name = '{}.err'.format(logfile_basename)

        n_files = len(glob('{}.*.out'.format(logfile_basename)))
        new_logfile_name = '{}.{}.out'.format(logfile_basename, n_files)
        new_logfile_err_name = '{}.{}.err'.format(logfile_basename, n_files)

        # Create the logfile dir
        if not os.path.exists(logfile_dir_path):
            mkdir_p(logfile_dir_path)

        # Remove colliding file
        if os.path.exists(new_logfile_name):
            os.unlink(new_logfile_name)

        # Write out interleaved log data
        with open(new_logfile_name, 'wb') as logfile:
            logfile.write(self.interleaved_buffer)

        # Create symlink to the latest logfile
        if os.path.exists(logfile_link_name):
            os.unlink(logfile_link_name)
        os.symlink(new_logfile_name, logfile_link_name)

        # Create stderr logfile and symlinkt if stderr is empty
        if len(self.stderr_buffer) > 0:
            with open(new_logfile_err_name, 'wb') as logfile:
                logfile.write(self.stderr_buffer)
            if os.path.exists(logfile_err_link_name):
                os.unlink(logfile_err_link_name)
            os.symlink(new_logfile_err_name, logfile_err_link_name)

        return new_logfile_name

    @classmethod
    def factory(cls, label, job_id, stage_label, event_queue):
        """Factory method for constructing with job metadata."""

        def init_proxy(*args, **kwargs):
            return cls(label, job_id, stage_label, event_queue, *args, **kwargs)

        return init_proxy


class IOBufferLogger(IOBufferContainer):

    """This is a logging class to be used instead of sys.stdout and sys.stderr
    in FunctionStage operations.

    This class also generates `stdout` and `stderr` events.
    """

    def __init__(self, label, job_id, stage_label, event_queue, *args, **kwargs):
        IOBufferContainer.__init__(self, label, job_id, stage_label, event_queue,)

    def out(self, data):
        self.stdout_buffer += (data.rstrip() + '\n').encode('utf-8')
        self.interleaved_buffer += (data.rstrip() + '\n').encode('utf-8')

        self.event_queue.put(ExecutionEvent(
            'STDOUT',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))

    def err(self, data):
        self.stderr_buffer += (data.rstrip() + '\n').encode('utf-8')
        self.interleaved_buffer += (data.rstrip() + '\n').encode('utf-8')

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

    def __init__(self, label, job_id, stage_label, event_queue, *args, **kwargs):
        IOBufferContainer.__init__(self, label, job_id, stage_label, event_queue)
        AsyncSubprocessProtocol.__init__(self, *args, **kwargs)

    def on_stdout_received(self, data):
        self.stdout_buffer += data
        self.interleaved_buffer += data

        progress_matches = re.match('\[\s*([0-9]+)%\]', data.decode('utf-8'))
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
            data=data))

    def on_stderr_received(self, data):
        self.stderr_buffer += data
        self.interleaved_buffer += data

        self.event_queue.put(ExecutionEvent(
            'STDERR',
            job_id=self.job_id,
            stage_label=self.stage_label,
            data=data))
