import mock

from catkin_tools import common


def test_get_recursive_build_depends_in_workspace_with_test_depend():
    pkg1 = mock.Mock()
    pkg1.name = 'pkg1'
    pkg1.build_depends = []
    pkg1.buildtool_depends = []
    pkg1.test_depends = []
    pkg1.run_depends = []
    pkg1.exec_depends = []
    pkg1.build_export_depends = []

    pkg2 = mock.Mock()
    pkg2.name = 'pkg2'
    pkg2.build_depends = []
    pkg2.buildtool_depends = []
    pkg2.test_depends = []
    pkg2.run_depends = []
    pkg2.exec_depends = []
    pkg2.build_export_depends = []

    pkg1.test_depends.append(pkg2)

    ordered_packages = [
        ('/path/to/pkg1', pkg1),
        ('/path/to/pkg2', pkg2),
    ]

    r = common.get_recursive_build_depends_in_workspace(pkg1, ordered_packages)
    assert r == ordered_packages[1:], r


def test_format_time_delta_short():
    inputs = {
        1.45: "1.4",
        61.45: "01:01.4",
        121.45: "02:01.4",
        # this one tests: https://github.com/catkin/catkin_tools/issues/356
        3601.45: "1:00:01.4",
        3721.45: "1:02:01.4",
        7321.45: "2:02:01.4",
        93821.45: "1 day 2:03:41.5",
    }
    for k, v in inputs.items():
        f = common.format_time_delta_short(k)
        assert f == v, "format_time_delta_short({0}) -> '{1}' != '{2}'".format(k, f, v)
