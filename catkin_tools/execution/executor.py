from __future__ import print_function

import time
import traceback

from itertools import tee

import trollius as asyncio

from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import FIRST_COMPLETED

from osrf_pycommon.process_utils import async_execute_process
from osrf_pycommon.process_utils import get_loop


from .events import ExecutionEvent

from .io import IOBufferLogger

from .jobs import JobServer

from .stages import CommandStage
from .stages import FunctionStage


def split(values, cond):
    """Split an iterable based on a condition."""
    head, tail = tee((cond(v), v) for v in values)
    return [v for c, v in head if c], [v for c, v in tail if not c]


@asyncio.coroutine
def async_job(label, job, threadpool, event_queue, log_path):
    """Run a sequence of Stages from a Job and collect their output.

    :param job: A Job instance
    :threadpool: A thread pool executor for blocking stages
    :event_queue: A queue for asynchronous events
    """

    # Initialize success flag
    all_stages_succeeded = True

    # Jobs start occuping a jobserver job
    occupying_job = True

    # Execute each stage of this job
    for stage in job.stages:
        # Logger reference in this scope for error reporting
        logger = None

        # Abort the job if one of the stages has failed
        if job.continue_on_failure and not all_stages_succeeded:
            break

        # If the stage doesn't require a job token, release it temporarily
        if stage.occupy_job:
            if not occupying_job:
                while JobServer.try_acquire() is None:
                    yield asyncio.From(asyncio.sleep(0.2))
                occupying_job = True
        else:
            if occupying_job:
                JobServer.release()
                occupying_job = False

        # Notify stage started
        event_queue.put(ExecutionEvent(
            'STARTED_STAGE',
            job_id=job.jid,
            stage_label=stage.label))

        if type(stage) is CommandStage:
            try:
                # Initiate the command
                while True:
                    try:
                        protocol_type = stage.logger_factory(label, job.jid, stage.label, event_queue, log_path)
                        transport, logger = yield asyncio.From(
                            async_execute_process(
                                protocol_type,
                                **stage.async_execute_process_kwargs))
                        break
                    except OSError as exc:
                        if 'Text file busy' in str(exc):
                            # This is a transient error, try again shortly
                            # TODO: report the file causing the problem (exc.filename)
                            time.sleep(0.01)
                            continue
                        raise

                # Notify that a subprocess has been created
                event_queue.put(ExecutionEvent(
                    'SUBPROCESS',
                    job_id=job.jid,
                    stage_label=stage.label,
                    **stage.async_execute_process_kwargs))

                # Asynchronously yield until this command is  completed
                retcode = yield asyncio.From(logger.complete)
            except:
                if logger is None:
                    logger = IOBufferLogger(label, job.jid, stage.label, event_queue, log_path)
                logger.err(str(traceback.format_exc()))
                retcode = 3

        elif type(stage) is FunctionStage:
            logger = IOBufferLogger(label, job.jid, stage.label, event_queue, log_path)
            try:
                # Asynchronously yield until this function is completed
                retcode = yield asyncio.From(get_loop().run_in_executor(
                    threadpool,
                    stage.function,
                    logger,
                    event_queue))
            except:
                logger.err(str(traceback.format_exc()))
                retcode = 3
        else:
            raise TypeError("Bad Job Stage: {}".format(stage))

        # Set whether this stage succeeded
        stage_succeeded = (retcode == 0)

        # Update success tracker from this stage
        all_stages_succeeded = all_stages_succeeded and stage_succeeded

        # Close the logger
        logger.close()

        # Store the results from this stage
        event_queue.put(ExecutionEvent(
            'FINISHED_STAGE',
            job_id=job.jid,
            stage_label=stage.label,
            succeeded=stage_succeeded,
            stdout=logger.stdout_buffer,
            stderr=logger.stderr_buffer,
            interleaved=logger.interleaved_buffer,
            logfile_filename=logger.unique_logfile_name,
            retcode=retcode))

    # Finally, return whether all stages of the job completed
    raise asyncio.Return(job.jid, all_stages_succeeded)


@asyncio.coroutine
def execute_jobs(
        label,
        jobs,
        event_queue,
        log_path,
        max_toplevel_jobs=None,
        continue_on_failure=False,
        continue_without_deps=False):
    """Process a number of jobs asynchronously.

    :param jobs: A list of topologically-sorted Jobs with no circular dependencies.
    :param event_queue: A python queue for reporting events.
    :param log_path: The path in which logfiles can be written
    :param max_toplevel_jobs: Max number of top-level jobs
    :param continue_on_failure: Keep running jobs even if one fails.
    :param continue_without_deps: Run jobs even if their dependencies fail.
    """

    # Map of jid -> job
    job_map = dict([(j.jid, j) for j in jobs])
    # Jobs which are not ready to be executed
    pending_jobs = []
    # Jobs which are ready to be executed once workers are available
    queued_jobs = []
    # List of active jobs
    active_jobs = []
    # Set of active job futures
    active_job_fs = set()
    # Dict of completd jobs job_id -> succeeded
    completed_jobs = {}
    # List of jobs whose deps failed
    abandoned_jobs = []

    # Create a thread pool executor for blocking python stages in the asynchronous jobs
    threadpool = ThreadPoolExecutor(max_workers=JobServer.max_jobs())

    # Immediately abandon jobs with bad dependencies
    pending_jobs, new_abandoned_jobs = split(jobs, lambda j: all([d in job_map for d in j.deps]))

    for abandoned_job in new_abandoned_jobs:
        abandoned_jobs.append(abandoned_job)
        event_queue.put(ExecutionEvent(
            'ABANDONED_JOB',
            job_id=abandoned_job.jid,
            reason='MISSING_DEPS',
            dep_ids=[d for d in abandoned_job.deps if d not in job_map]))

    # Initialize list of ready and pending jobs (jobs not ready to be executed)
    queued_jobs, pending_jobs = split(pending_jobs, lambda j: len(j.deps) == 0)

    # Process all jobs asynchronously until there are none left
    while len(active_job_fs) + len(queued_jobs) + len(pending_jobs) > 0:

        # Activate jobs while the jobserver dispenses tokens
        while ((len(queued_jobs) > 0) and
               ((max_toplevel_jobs is None) or (len(active_jobs) < max_toplevel_jobs)) and
               (JobServer.try_acquire() is not None)):

            # Pop a job off of the job queue
            job = queued_jobs.pop(0)

            # Label it (for debugging)
            JobServer.add_label(job.jid)

            # Notify that the job is being started
            event_queue.put(ExecutionEvent(
                'STARTED_JOB',
                job_id=job.jid))

            # Start the job coroutine
            active_jobs.append(job)
            active_job_fs.add(async_job(label, job, threadpool, event_queue, log_path))

        # Report running jobs
        event_queue.put(ExecutionEvent(
            'JOB_STATUS',
            pending=[j.jid for j in pending_jobs],
            queued=[j.jid for j in queued_jobs],
            active=[j.jid for j in active_jobs],
            abandoned=[j.jid for j in abandoned_jobs],
            completed=completed_jobs
        ))

        # Process jobs as they complete asynchronously
        done_job_fs, active_job_fs = yield asyncio.From(asyncio.wait(
            active_job_fs,
            timeout=0.10,
            return_when=FIRST_COMPLETED))

        for done_job_f in done_job_fs:
            # Capture a result once the job has finished
            job_id, succeeded = yield asyncio.From(done_job_f)

            # Release a jobserver token now that this job has succeeded
            JobServer.release(job_id)
            active_jobs = [j for j in active_jobs if j.jid != job_id]

            # Generate event with the results of this job
            event_queue.put(ExecutionEvent(
                'FINISHED_JOB',
                job_id=job_id,
                succeeded=succeeded))

            # Add the job to the completed list
            completed_jobs[job_id] = succeeded

            # Handle failure modes
            if not succeeded:
                # Handle different abandoning policies
                if not continue_on_failure:
                    # Abort all pending jobs if any job fails
                    new_abandoned_jobs = queued_jobs + pending_jobs
                    queued_jobs = []
                    pending_jobs = []

                    # Notify that jobs have been abandoned
                    for abandoned_job in new_abandoned_jobs:
                        abandoned_jobs.append(abandoned_job)
                        event_queue.put(ExecutionEvent(
                            'ABANDONED_JOB',
                            job_id=abandoned_job.jid,
                            reason='PEER_FAILED',
                            peer_job_id=job_id))

                elif not continue_without_deps:
                    unhandled_abandoned_job_ids = [job_id]

                    # Abandon jobs which depend on abandoned jobs
                    while len(unhandled_abandoned_job_ids) > 0:
                        # Get the abandoned job
                        abandoned_job_id = unhandled_abandoned_job_ids.pop(0)

                        # Abandon all pending jobs which depend on this job_id
                        unhandled_abandoned_jobs, pending_jobs = split(
                            pending_jobs,
                            lambda j: abandoned_job_id in j.deps)

                        # Handle each new abandoned job
                        for abandoned_job in unhandled_abandoned_jobs:
                            abandoned_jobs.append(abandoned_job)
                            # Notify if any jobs have been abandoned
                            event_queue.put(ExecutionEvent(
                                'ABANDONED_JOB',
                                job_id=abandoned_job.jid,
                                reason='DEP_FAILED',
                                direct_dep_job_id=abandoned_job_id,
                                dep_job_id=job_id))

                        # Add additional job ids to check
                        unhandled_abandoned_job_ids.extend(
                            [j.jid for j in unhandled_abandoned_jobs])

            # Update the list of ready jobs (based on completed job dependencies)
            new_queued_jobs, pending_jobs = split(
                pending_jobs,
                lambda j: j.all_deps_completed(completed_jobs))
            queued_jobs.extend(new_queued_jobs)

            # Notify of newly queued jobs
            for queued_job in new_queued_jobs:
                event_queue.put(ExecutionEvent(
                    'QUEUED_JOB',
                    job_id=queued_job.jid))

    # Report running jobs
    event_queue.put(ExecutionEvent(
        'JOB_STATUS',
        pending=[j.jid for j in pending_jobs],
        queued=[j.jid for j in queued_jobs],
        active=[j.jid for j in active_jobs],
        abandoned=[j.jid for j in abandoned_jobs],
        completed=completed_jobs
    ))

    raise asyncio.Return(all(completed_jobs.values()))


def run_until_complete(coroutine):
    # Get event loop
    loop = get_loop()

    # Run jobs
    return loop.run_until_complete(coroutine)
