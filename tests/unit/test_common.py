import mock

from catkin_tools import common


def test_get_recursive_build_depends_in_workspace_with_test_depend():
    pkg1 = mock.Mock()
    pkg1.name = 'pkg1'
    pkg1.build_depends = []
    pkg1.buildtool_depends = []
    pkg1.test_depends = []
    pkg1.run_depends = []

    pkg2 = mock.Mock()
    pkg2.name = 'pkg2'
    pkg2.build_depends = []
    pkg2.buildtool_depends = []
    pkg2.test_depends = []
    pkg2.run_depends = []

    pkg1.test_depends.append(pkg2)

    ordered_packages = [
        ('/path/to/pkg1', pkg1),
        ('/path/to/pkg2', pkg2),
    ]

    r = common.get_recursive_build_depends_in_workspace(pkg1, ordered_packages)
    assert r == ordered_packages[1:], r
