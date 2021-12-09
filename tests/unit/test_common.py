import mock

from catkin_tools import common


class MockPackage(mock.Mock):
    def __init__(self, name):
        super(MockPackage, self).__init__()
        self.name = name
        self.build_depends = []
        self.buildtool_depends = []
        self.test_depends = []
        self.run_depends = []
        self.exec_depends = []
        self.build_export_depends = []
        self.evaluated_condition = True


def test_get_recursive_build_depends_in_workspace_with_test_depend():
    pkg1 = MockPackage('pkg1')
    pkg2 = MockPackage('pkg2')
    pkg1.test_depends.append(pkg2)

    ordered_packages = [
        ('/path/to/pkg1', pkg1),
        ('/path/to/pkg2', pkg2),
    ]

    r = common.get_recursive_build_depends_in_workspace(pkg1, ordered_packages)
    assert r == ordered_packages[1:], r


def test_get_recursive_build_depends_in_workspace_with_condition():
    pkg = MockPackage('pkg')
    cond_false_pkg = MockPackage('cond_false_pkg')
    cond_false_pkg.evaluated_condition = False
    cond_true_pkg = MockPackage('cond_true_pkg')
    pkg.build_depends = [cond_true_pkg, cond_false_pkg]

    ordered_packages = [
        ('/path/to/pkg', pkg),
        ('/path/to/cond_false_pkg', cond_false_pkg),
        ('/path/to/cond_true_pkg', cond_true_pkg),
    ]

    r = common.get_recursive_build_depends_in_workspace(pkg, ordered_packages)
    assert r == ordered_packages[2:], r


def test_get_recursive_build_depends_in_workspace_circular_run_depend():
    pkg1 = MockPackage('pkg1')
    pkg2 = MockPackage('pkg2')
    pkg1.run_depends.append(pkg2)
    pkg2.run_depends.append(pkg1)

    ordered_packages = [
        ('/path/to/pkg1', pkg1),
        ('/path/to/pkg2', pkg2),
    ]

    r = common.get_recursive_build_depends_in_workspace(pkg1, ordered_packages)
    assert r == ordered_packages[:], r

    r = common.get_recursive_build_depends_in_workspace(pkg2, ordered_packages)
    assert r == ordered_packages[:], r


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
