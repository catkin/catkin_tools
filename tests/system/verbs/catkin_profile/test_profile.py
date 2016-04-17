import os

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success

from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_warning_message
from ....workspace_assertions import assert_no_warnings


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
