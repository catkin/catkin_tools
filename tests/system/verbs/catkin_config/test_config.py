import os

from ....utils import catkin_success
from ....utils import in_temporary_directory
from ....utils import redirected_stdio
from ....workspace_assertions import assert_in_config
from ....workspace_assertions import assert_no_warnings
from ....workspace_assertions import assert_warning_message
from ....workspace_assertions import assert_workspace_initialized
from ...workspace_factory import workspace_factory


@in_temporary_directory
def test_config_no_ws():
    with redirected_stdio() as (out, err):
        assert catkin_success(['config'])
        assert_warning_message(out, 'Workspace .+ is not yet initialized')


@in_temporary_directory
def test_init_local_empty_src():
    cwd = os.getcwd()
    os.mkdir(os.path.join(cwd, 'src'))
    with redirected_stdio() as (out, err):
        assert catkin_success(['config', '--init'])
        assert_no_warnings(out)
    assert_workspace_initialized('.')


@in_temporary_directory
def test_config_non_bare():
    with redirected_stdio() as (out, err):
        assert catkin_success(['config', '--install'])
        assert_warning_message(out, 'Source space .+ does not yet exist')
    assert_workspace_initialized('.')


@in_temporary_directory
def test_config_unchanged():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', '-j6'])
            assert catkin_success(['config'])
        assert_in_config('.', 'default', 'jobs_args', ['-j6'])


def test_config_no_args_flags():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', '-j6', 'test'])
            assert catkin_success(['config', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release'])
            assert catkin_success(['config', '--catkin-make-args', 'run_tests'])
            assert catkin_success(['config', '--no-make-args', '--no-cmake-args', '--no-catkin-make-args'])
        assert_in_config('.', 'default', 'jobs_args', [])
        assert_in_config('.', 'default', 'make_args', [])
        assert_in_config('.', 'default', 'cmake_args', [])


@in_temporary_directory
def test_config_no_buildlist():
    with redirected_stdio():
        assert catkin_success(['config', '--buildlist', 'mypackage'])
        assert_in_config('.', 'default', 'whitelist', ['mypackage'])
        assert catkin_success(['config', '--no-buildlist'])
        assert_in_config('.', 'default', 'whitelist', [])


@in_temporary_directory
def test_config_no_skiplist():
    with redirected_stdio():
        assert catkin_success(['config', '--skiplist', 'mypackage'])
        assert_in_config('.', 'default', 'blacklist', ['mypackage'])
        assert catkin_success(['config', '--no-skiplist'])
        assert_in_config('.', 'default', 'blacklist', [])
