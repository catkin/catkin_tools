import os
import shutil

from ..utils import temporary_directory


class workspace_factory(temporary_directory):
    def __init__(self, source_space='src', prefix=''):
        super(workspace_factory, self).__init__(prefix=prefix)
        self.source_space = source_space

    def __enter__(self):
        self.temporary_directory = super(workspace_factory, self).__enter__()
        self.workspace_factory = WorkspaceFactory(self.temporary_directory, self.source_space)
        return self.workspace_factory

    def __exit__(self, exc_type, exc_value, traceback):
        super(workspace_factory, self).__exit__(exc_type, exc_value, traceback)


class WorkspaceFactory(object):
    def __init__(self, workspace, source_space):
        self.workspace = workspace
        self.source_space = os.path.join(self.workspace, source_space)
        self.packages = {}

    class Package(object):
        def __init__(self, name, depends, build_depends, run_depends, test_depends):
            self.name = name
            self.build_depends = (build_depends or []) + (depends or [])
            self.run_depends = (run_depends or []) + (depends or [])
            self.test_depends = (test_depends or [])

    def add_package(self, pkg_name, depends=None, build_depends=None, run_depends=None, test_depends=None):
        self.packages[pkg_name] = self.Package(pkg_name, depends, build_depends, run_depends, test_depends)

    def build(self):
        cwd = os.getcwd()
        if not os.path.isdir(self.workspace):
            if os.path.exists(self.workspace):
                raise RuntimeError("Cannot build workspace in '{0}' because it is a file".format(self.workspace))
            os.makedirs(self.workspace)
        if os.path.exists(self.source_space):
            print("WARNING: source space given to WorkspaceFactory exists, clearing before build()'ing")
            self.clear()
        os.makedirs(self.source_space)
        try:
            os.chdir(self.source_space)
            for name, pkg in self.packages.items():
                pkg_dir = os.path.join(self.source_space, name)
                os.makedirs(pkg_dir)
                pkg_xml_path = os.path.join(pkg_dir, 'package.xml')
                pkg_xml = """\
<?xml version="1.0"?>
<package>
  <name>{name}</name>
  <version>0.0.0</version>
  <description>
    Description for {name}
  </description>

  <maintainer email="person@email.com">Firstname Lastname</maintainer>
  <license>MIT</license>

"""
                pkg_xml += '\n'.join(
                    ['  <build_depend>{0}</build_depend>'.format(x) for x in pkg.build_depends] +
                    ['  <run_depend>{0}</run_depend>'.format(x) for x in pkg.run_depends] +
                    ['  <test_depend>{0}</test_depend>'.format(x) for x in pkg.test_depends]
                )
                pkg_xml += """
  <export>
    <build_type>cmake</build_type>
  </export>
</package>
"""
                with open(pkg_xml_path, 'w') as f:
                    f.write(pkg_xml.format(name=name))
                cmakelists_txt_path = os.path.join(pkg_dir, 'CMakeLists.txt')
                cmakelists_txt = """\
cmake_minimum_required(VERSION 2.8.3)
project({name})

add_custom_target(install)

"""
                with open(cmakelists_txt_path, 'w') as f:
                    f.write(cmakelists_txt.format(name=name, find_package=' '.join(pkg.build_depends)))
        finally:
            os.chdir(cwd)

    def clear(self):
        if os.path.exists(self.workspace):
            shutil.rmtree(self.workspace)
