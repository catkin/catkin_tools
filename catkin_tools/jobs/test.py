# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from catkin_tools.execution.io import IOBufferProtocol
from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import FunctionStage, CommandStage
from catkin_tools.jobs.commands.cmake import CMakeMakeIOBufferProtocol
from catkin_tools.jobs.commands.make import MAKE_EXEC
from catkin_tools.jobs.utils import loadenv


def create_test_job(
    context,
    package,
    package_path,
):
    """Generate a job that tests a package"""

    # Package build space path
    build_space = context.package_build_space(package)
    # Environment dictionary for the job, which will be built
    # up by the executions in the loadenv stage.
    job_env = dict(os.environ)

    # Create job stages
    stages = []

    # Load environment for job
    stages.append(FunctionStage(
        'loadenv',
        loadenv,
        locked_resource=None,
        job_env=job_env,
        package=package,
        context=context,
        verbose=False,
    ))

    # Make command
    stages.append(CommandStage(
        'make',
        [MAKE_EXEC, 'run_tests'],
        cwd=build_space,
        logger_factory=IOBufferProtocol.factory
    ))

    return Job(
        jid=package.name,
        deps=[],
        env=job_env,
        stages=stages,
    )