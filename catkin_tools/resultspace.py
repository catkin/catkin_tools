
import os
import re

from .runner import run_command
from .common import string_type
from .utils import which

CMAKE_EXEC = which('cmake')
SORT_EXEC = which('sort')

# Cache for result-space environments
_resultspace_env_cache = {}


def get_resultspace_environment(result_space_path, quiet=False, cached=True):
    """Get the environemt variables which result from sourcing another catkin
    workspace's setup files as the string output of `cmake -E environment`.
    This command is used to be as portable as possible.

    :param result_space_path: path to a Catkin result-space whose environment should be loaded, ``str``
    :type result_space_path: str
    :param quiet: don't throw exceptions, ``bool``
    :type quiet: bool
    :param cached: use the cached environment
    :type cached: bool

    :returns: a dictionary of environment variables and their values
    """

    # Check the cache first
    if cached and result_space_path in _resultspace_env_cache:
        return _resultspace_env_cache[result_space_path]

    # Check to make sure result_space_path is a valid directory
    if not os.path.isdir(result_space_path):
        if quiet:
            return dict()
        raise IOError(
            "Cannot load environment from resultspace \"%s\" because it does not "
            "exist." % result_space_path
        )

    # Check to make sure result_space_path contains a `.catkin` file
    # TODO: `.catkin` should be defined somewhere as an atom in catkin_pkg
    if not os.path.exists(os.path.join(result_space_path, '.catkin')):
        if quiet:
            return dict()
        raise IOError(
            "Cannot load environment from resultspace \"%s\" because it does not "
            "appear to be a catkin-generated resultspace (missing .catkin marker "
            "file)." % result_space_path
        )

    # Determine the shell to use to source the setup file
    shell_path = os.environ['SHELL']
    (_, shell_name) = os.path.split(shell_path)

    # Use fallback shell if using a non-standard shell
    if shell_name not in ['bash', 'zsh']:
        shell_name = 'bash'

    # Check to make sure result_space_path contains the appropriate setup file
    setup_file_path = os.path.join(result_space_path, 'env.sh')
    if not os.path.exists(setup_file_path):
        if quiet:
            return dict()
        raise IOError(
            "Cannot load environment from resultspace \"%s\" because the "
            "required setup file \"%s\" does not exist." % (result_space_path, setup_file_path)
        )

    # Make sure we've found CMAKE and SORT executables
    if CMAKE_EXEC is None:
        print("WARNING: Failed to find 'cmake' executable.")
        return {}
    if SORT_EXEC is None:
        print("WARNING: Failed to find 'sort' executable.")
        return {}

    # Construct a command list which sources the setup file and prints the env to stdout
    norc_flags = {'bash': '--norc', 'zsh': '-f'}
    subcommand = '%s %s -E environment | %s' % (setup_file_path, CMAKE_EXEC, SORT_EXEC)

    command = [
        shell_path,
        norc_flags[shell_name],
        '-c', subcommand]

    # Run the command to source the other environment and output all environment variables
    blacklisted_keys = ('_', 'PWD')
    env_regex = re.compile('(.+?)=(.*)$', re.M)
    env_dict = {}

    try:
        for line in run_command(command, cwd=os.getcwd()):
            if isinstance(line, string_type):
                matches = env_regex.findall(line)
                for (key, value) in matches:
                    value = value.rstrip()
                    if key not in blacklisted_keys and key not in env_dict:
                        env_dict[key] = value
    except IOError as err:
        print("WARNING: Failed to extract environment from resultspace: %s: %s" % (result_space_path, str(err)))
        return {}

    _resultspace_env_cache[result_space_path] = env_dict

    return env_dict


def load_resultspace_environment(result_space_path, cached=True):
    """Load the environemt variables which result from sourcing another
    workspace path into this process's environment.

    :param result_space_path: path to a Catkin result-space whose environment should be loaded, ``str``
    :type result_space_path: str
    :param cached: use the cached environment
    :type cached: bool
    """
    env_dict = get_resultspace_environment(result_space_path, cached=cached)
    os.environ.update(env_dict)
