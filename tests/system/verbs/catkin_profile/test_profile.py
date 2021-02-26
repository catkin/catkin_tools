import os

from ...workspace_factory import workspace_factory
from ....utils import in_temporary_directory
from ....utils import assert_cmd_success

from ....workspace_assertions import assert_in_config
from ....workspace_assertions import assert_active_profile


@in_temporary_directory
def test_profile_list():
    assert_cmd_success(['mkdir', 'src'])
    assert_cmd_success(['catkin', 'init'])
    assert_cmd_success(['catkin', 'build'])
    assert_cmd_success(['catkin', 'profile', 'list'])
    assert_active_profile('.', 'default')


@in_temporary_directory
def test_profile_set():
    assert_cmd_success(['mkdir', 'src'])
    assert_cmd_success(['catkin', 'init'])
    assert_cmd_success(['catkin', 'build'])
    assert_cmd_success(['catkin', 'profile', 'set', 'default'])
    assert_active_profile('.', 'default')


def test_profile_copy():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--make-args', 'test'])
        assert_cmd_success(['catkin', 'profile', 'add', '--copy', 'default', 'mycopy'])
        assert_in_config('.', 'mycopy', 'make_args', ['test'])


def test_profile_extend():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--make-args', 'test'])
        assert_cmd_success(['catkin', 'profile', 'add', '--extend', 'default', 'myextend'])
        assert_cmd_success(['catkin', 'config', '--profile', 'myextend', '--blacklist', 'mypackage'])
        assert_in_config('.', 'default', 'make_args', ['test'])
        assert_in_config('.', 'myextend', 'blacklist', ['mypackage'])


def test_profile_no_default():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--profile', 'myprofile', '--make-args', 'test'])
        assert_in_config('.', 'myprofile', 'make_args', ['test'])
        assert_active_profile('.', 'myprofile')
