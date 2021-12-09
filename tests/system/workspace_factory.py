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

    def __init__(self, workspace, source_space='src'):
        self.workspace = workspace
        self.source_space = os.path.join(self.workspace, source_space)
        self.packages = {}

    class Package(object):

        PACKAGE_XML_TEMPLATE = """\
<?xml version="1.0"?>
<package>
  <name>{name}</name>
  <version>0.0.0</version>
  <description>
    Description for {name}
  </description>

  <maintainer email="person@email.com">Firstname Lastname</maintainer>
  <license>MIT</license>

{depends_xml}

{export_xml}

</package>
"""
        PACKAGE_XML_EXPORT_TEMPLATE = """
  <export>
    <build_type>{build_type}</build_type>
  </export>"""

        CATKIN_CMAKELISTS_TEMPLATE = """
cmake_minimum_required(VERSION 2.8.12)
project({name})
find_package(catkin REQUIRED COMPONENTS {catkin_components})
catkin_package()"""

        CMAKE_CMAKELISTS_TEMPLATE = """
cmake_minimum_required(VERSION 2.8.12)
project({name})
{find_packages}
add_custom_target(install)"""

        CMAKE_CMAKELISTS_FIND_PACKAGE_TEMPLATE = """
find_package({name})"""

        def __init__(self, name, build_type, depends, build_depends, run_depends, test_depends):
            self.name = name
            self.build_type = build_type
            self.build_depends = (build_depends or []) + (depends or [])
            self.run_depends = (run_depends or []) + (depends or [])
            self.test_depends = (test_depends or [])

        def get_package_xml(self):
            # Get dependencies
            depends_xml = '\n'.join(
                ['  <buildtool_depend>{0}</buildtool_depend>'.format(self.build_type)] +
                ['  <build_depend>{0}</build_depend>'.format(x) for x in self.build_depends] +
                ['  <run_depend>{0}</run_depend>'.format(x) for x in self.run_depends] +
                ['  <test_depend>{0}</test_depend>'.format(x) for x in self.test_depends]
            )

            # Get exports section
            if self.build_type == 'catkin':
                export_xml = ''
            else:
                export_xml = self.PACKAGE_XML_EXPORT_TEMPLATE.format(build_type=self.build_type)

            # Format the package.xml template
            return self.PACKAGE_XML_TEMPLATE.format(
                name=self.name,
                depends_xml=depends_xml,
                export_xml=export_xml)

        def get_cmakelists_txt(self):
            if self.build_type == 'catkin':
                return self.CATKIN_CMAKELISTS_TEMPLATE.format(
                    name=self.name,
                    catkin_components=' '.join(self.build_depends))
            if self.build_type == 'cmake':
                find_packages = '\n'.join([
                    self.CMAKE_CMAKELISTS_FIND_PACKAGE_TEMPLATE.format(name=name)
                    for name in self.build_depends])
                return self.CMAKE_CMAKELISTS_TEMPLATE.format(
                    name=self.name,
                    find_packages=find_packages)

    def add_package(self, pkg_name, package_path):
        """Copy a static package into the workspace"""
        shutil.copytree(package_path, self.source_space)

    def create_package(
        self,
        pkg_name,
        build_type='cmake',
        depends=None,
        build_depends=None,
        run_depends=None,
        test_depends=None
    ):
        """Add a package to be generated in this workspace."""
        self.packages[pkg_name] = self.Package(pkg_name, build_type, depends, build_depends, run_depends, test_depends)

    def build(self):
        """Generate workspace paths and packages."""
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
                with open(pkg_xml_path, 'w') as f:
                    f.write(pkg.get_package_xml())
                cmakelists_txt_path = os.path.join(pkg_dir, 'CMakeLists.txt')
                with open(cmakelists_txt_path, 'w') as f:
                    f.write(pkg.get_cmakelists_txt())
        finally:
            os.chdir(cwd)

    def clear(self):
        if os.path.exists(self.workspace):
            shutil.rmtree(self.workspace)
