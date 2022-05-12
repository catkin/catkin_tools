from ....utils import assert_cmd_success
from ....utils import catkin_success
from ....utils import in_temporary_directory
from ....utils import redirected_stdio
from ....workspace_assertions import assert_in_config
from ...workspace_factory import workspace_factory

BUILD = ['build', '--no-notify', '--no-status']


@in_temporary_directory
def test_profile_list():
    assert_cmd_success(['mkdir', 'src'])
    with redirected_stdio():
        assert catkin_success(['init'])
        assert catkin_success(BUILD)
        assert catkin_success(['profile', 'list'])


@in_temporary_directory
def test_profile_set():
    assert_cmd_success(['mkdir', 'src'])
    with redirected_stdio():
        assert catkin_success(['init'])
        assert catkin_success(BUILD)
        assert catkin_success(['profile', 'set', 'default'])


def test_profile_copy():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', 'test'])
            assert catkin_success(['profile', 'add', '--copy', 'default', 'mycopy'])
        assert_in_config('.', 'mycopy', 'make_args', ['test'])


def test_profile_extend():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', 'test'])
            assert catkin_success(['profile', 'add', '--extend', 'default', 'myextend'])
            assert catkin_success(['config', '--profile', 'myextend', '--skiplist', 'mypackage'])
        assert_in_config('.', 'default', 'make_args', ['test'])
        assert_in_config('.', 'myextend', 'blacklist', ['mypackage'])
