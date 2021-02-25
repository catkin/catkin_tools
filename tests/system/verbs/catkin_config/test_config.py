import os

from ...workspace_factory import workspace_factory
from ....utils import in_temporary_directory
from ....utils import assert_cmd_success

from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_warning_message
from ....workspace_assertions import assert_no_warnings
from ....workspace_assertions import assert_in_config


@in_temporary_directory
def test_config_no_ws():
    out = assert_cmd_success(['catkin', 'config'])
    assert_warning_message(out, 'Workspace .+ is not yet initialized')


@in_temporary_directory
def test_init_local_empty_src():
    cwd = os.getcwd()
    os.mkdir(os.path.join(cwd, 'src'))
    out = assert_cmd_success(['catkin', 'config', '--init'])
    assert_no_warnings(out)
    assert_workspace_initialized('.')


@in_temporary_directory
def test_config_non_bare():
    out = assert_cmd_success(['catkin', 'config', '--install'])
    assert_workspace_initialized('.')
    assert_warning_message(out, 'Source space .+ does not yet exist')


@in_temporary_directory
def test_config_unchanged():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--make-args', '-j6'])
        assert_cmd_success(['catkin', 'config'])
        assert_in_config('.', 'default', 'jobs_args', ['-j6'])


def test_config_no_args_flags():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--make-args', '-j6', 'test'])
        assert_cmd_success(['catkin', 'config', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release'])
        assert_cmd_success(['catkin', 'config', '--catkin-make-args', 'run_tests'])
        assert_cmd_success(['catkin', 'config', '--no-make-args', '--no-cmake-args', '--no-catkin-make-args'])
        assert_in_config('.', 'default', 'jobs_args', [])
        assert_in_config('.', 'default', 'make_args', [])
        assert_in_config('.', 'default', 'cmake_args', [])
