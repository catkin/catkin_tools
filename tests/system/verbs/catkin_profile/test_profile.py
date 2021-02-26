import os

from ...workspace_factory import workspace_factory
from ....utils import in_temporary_directory
from ....utils import assert_cmd_success

from ....workspace_assertions import assert_in_config


@in_temporary_directory
def test_profile_list():
    assert_cmd_success(['mkdir', 'src'])
    assert_cmd_success(['catkin', 'init'])
    assert_cmd_success(['catkin', 'build'])
    assert_cmd_success(['catkin', 'profile', 'list'])


@in_temporary_directory
def test_profile_set():
    assert_cmd_success(['mkdir', 'src'])
    assert_cmd_success(['catkin', 'init'])
    assert_cmd_success(['catkin', 'build'])
    assert_cmd_success(['catkin', 'profile', 'set', 'default'])


def test_profile_copy():
    with workspace_factory() as wf:
        wf.build()
        assert_cmd_success(['catkin', 'config', '--make-args', 'test'])
        assert_cmd_success(['catkin', 'profile', 'add', '--copy', 'default', 'mycopy'])
        assert_in_config('.', 'mycopy', 'make_args', ['test'])
