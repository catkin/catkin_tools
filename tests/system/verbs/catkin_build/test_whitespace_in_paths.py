import os

from ....utils import catkin_success

from ...workspace_factory import workspace_factory


def test_catkin_build_with_whitespace_in_paths():
    with workspace_factory(source_space='source packages') as wf:
        wf.create_package('foo', depends=['bar'])
        wf.create_package('bar')
        wf.build()

        print('Workspace: {0}'.format(wf.workspace))

        assert os.path.isdir(wf.workspace)

        cmd = ['config', '--source', wf.source_space,
               '--devel', 'devel space',
               '--build', 'build space',
               '--install-space', 'install space']
        assert catkin_success(cmd), cmd

        cmd = ['build', '--no-status', '--no-notify', '--verbose']
        assert catkin_success(cmd), cmd
