import os

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success

from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_warning_message
from ....workspace_assertions import assert_no_warnings


@in_temporary_directory
def test_init_local_no_src():
    out = assert_cmd_success(['catkin', 'init'])
    assert_warning_message(out, 'Source space .+ does not yet exist')
    assert_workspace_initialized('.')


@in_temporary_directory
def test_init_local_empty_src():
    cwd = os.getcwd()
    os.mkdir(os.path.join(cwd, 'src'))
    out = assert_cmd_success(['catkin', 'init'])
    assert_no_warnings(out)
    assert_workspace_initialized('.')
