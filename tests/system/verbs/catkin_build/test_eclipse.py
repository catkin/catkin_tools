import os

from ....utils import in_temporary_directory
from ....utils import assert_files_exist
from ....utils import catkin_success
from ....utils import redirected_stdio


from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_no_warnings

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')


@in_temporary_directory
def test_build_eclipse():
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    print("Creating source directory: %s" % source_space)
    os.mkdir(source_space)
    with redirected_stdio() as (out, err):
        assert catkin_success(
            ['create', 'pkg', '--rosdistro', 'hydro', '-p', source_space, 'pkg_a']), 'create pkg'
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose',
             '--cmake-args', '-GEclipse CDT4 - Unix Makefiles'])
    assert_no_warnings(out)
    assert_workspace_initialized('.')
    assert_files_exist(os.path.join(cwd, 'build', 'pkg_a'),
                       ['.project', '.cproject'])
