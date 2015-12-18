
import os

try:
    from shlex import quote as cmd_quote
except ImportError:
    from pipes import quote as cmd_quote

from .io import IOBufferLogger
from .io import IOBufferProtocol


class Stage(object):

    """A description of one of the serially-executed stages of a Job.

    Like Jobs, Stages are stateless, and simply describe what needs to be done
    and how to do it.
    """

    def __init__(self, label, logger_factory=IOBufferProtocol.factory, occupy_job=True, repro=None):
        self.label = str(label)
        self.logger_factory = logger_factory
        self.occupy_job = occupy_job
        self.repro = repro

    def get_repro(self, verb, jid):
        """Get a string which should reproduce this stage outside of the catkin command."""
        return self.repro


class CommandStage(Stage):

    """Job stage that describes a system command.

    :param label: The label for the stage
    :param command: A list of strings composing a system command
    :param protocol: A protocol class to use for this stage

    Additional kwargs are passed to `async_execute_process`
    """

    def __init__(
            self,
            label,
            cmd,
            cwd=os.getcwd(),
            env={},
            shell=False,
            emulate_tty=True,
            stderr_to_stdout=False,
            occupy_job=True,
            logger_factory=IOBufferProtocol.factory):
        """ """

        if not type(cmd) in [list, tuple] or not all([type(s) is str for s in cmd]):
            raise ValueError('Command stage must be a list of strings: {}'.format(cmd))
        super(CommandStage, self).__init__(label, logger_factory, occupy_job)

        self.async_execute_process_kwargs = {
            'cmd': cmd,
            'cwd': cwd,
            'env': env,
            'shell': shell,
            # Emulate tty for cli colors
            'emulate_tty': emulate_tty,
            # Capture stderr and stdout separately
            'stderr_to_stdout': stderr_to_stdout,
        }

    def get_repro(self, verb, jid):
        return 'cd {}; env "$(catkin {} --env {})" {}; cd -'.format(
            self.async_execute_process_kwargs['cwd'],
            verb,
            jid,
            ' '.join([cmd_quote(t) for t in self.async_execute_process_kwargs['cmd']]))


class FunctionStage(Stage):

    """Job stage that describes a python function.

    :param label: The label for the stage
    :param function: A python function which returns 0 on success

    Functions must take the arguments:
        - logger
        - event_queue
    """

    def __init__(self, label, function, logger_factory=IOBufferLogger.factory, occupy_job=True, *args, **kwargs):
        if not callable(function):
            raise ValueError('Function stage must be callable.')
        super(FunctionStage, self).__init__(label, logger_factory, occupy_job)

        def function_proxy(logger, event_queue):
            return function(logger, event_queue, *args, **kwargs)
        self.function = function_proxy
