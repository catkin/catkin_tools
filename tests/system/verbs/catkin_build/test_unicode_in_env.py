import os

from ....utils import catkin_success

from ...workspace_factory import workspace_factory


def test_catkin_build_with_unicode_in_env():
    with workspace_factory() as wf:
        wf.create_package('foo', depends=['bar'])
        wf.create_package('bar')
        wf.build()

        print('Workspace: {0}'.format(wf.workspace))

        assert os.path.isdir(wf.workspace)

        env = {'NON_ASCII': '\xc3\xb6'}
        cmd = ['build', '--no-status', '--no-notify', '--verbose']
        assert catkin_success(cmd, env)
