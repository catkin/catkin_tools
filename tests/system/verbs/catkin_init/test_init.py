import os

from ....utils import catkin_success
from ....utils import in_temporary_directory
from ....utils import redirected_stdio
from ....workspace_assertions import assert_no_warnings
from ....workspace_assertions import assert_warning_message
from ....workspace_assertions import assert_workspace_initialized


@in_temporary_directory
def test_init_local_no_src():
    with redirected_stdio() as (out, err):
        assert catkin_success(['init'])
        assert_warning_message(out, 'Source space .+ does not yet exist')
    assert_workspace_initialized('.')


@in_temporary_directory
def test_init_local_empty_src():
    cwd = os.getcwd()
    os.mkdir(os.path.join(cwd, 'src'))
    with redirected_stdio() as (out, err):
        assert catkin_success(['init'])
        assert_no_warnings(out)
    assert_workspace_initialized('.')
