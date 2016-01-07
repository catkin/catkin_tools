# Copyright 2016 Open Source Robotics Foundation, Inc.
#
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

try:
    # Python3
    from queue import Empty
except ImportError:
    # Python2
    from Queue import Empty

import os
import operator
import sys
import threading
import time

from catkin_tools.common import disable_wide_log
from catkin_tools.common import format_time_delta
from catkin_tools.common import format_time_delta_short
from catkin_tools.common import remove_ansi_escape
from catkin_tools.common import terminal_width
from catkin_tools.common import wide_log

from catkin_tools.notifications import notify

from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import sanitize
from catkin_tools.terminal_color import ColorMapper

from catkin_tools.execution import job_server

# This map translates more human reable format strings into colorized versions
_color_translation_map = {
    # 'output': 'colorized_output'

    '': fmt('@!' + sanitize('') + '@|'),

    # Job starting
    "Starting >>> {:<{}}":
    fmt("Starting  @!@{gf}>>>@| @!@{cf}{:<{}}@|"),

    # Job finishing
    "Finished <<< {:<{}} [ {} ]":
    fmt("@!@{kf}Finished@|  @{gf}<<<@| @{cf}{:<{}}@| [ @{yf}{}@| ]"),

    "Failed <<< {:<{}} [ {} ]":
    fmt("@!@{rf}Failed@|    @{rf}<<<@| @{cf}{:<{}}@| [ @{yf}{}@| ]"),

    # Job abandoning
    "Abandoned <<< {:<{}} [ {} ]":
    fmt("@!@{rf}Abandoned@| @{rf}<<<@| @{cf}{:<{}}@| [ @{yf}{}@| ]"),

    "Depends on failed job {}":
    fmt("@{yf}Depends on failed job @!{}@|"),

    "Depends on failed job {} via {}":
    fmt("@{yf}Depends on failed job @!{}@| @{yf}via @!{}@|"),

    # Stage finishing
    "Starting >> {}:{}":
    fmt("Starting  @{gf} >>@| @{cf}{}@|:@{bf}{}@|"),

    "Subprocess > {}:{} `{}`":
    fmt("Subprocess  @!@{gf}>@| @{cf}{}@|:@{bf}{}@| @!@{kf}`{}`@|"),

    "Finished << {}:{}":
    fmt("@!@{kf}Finished@|  @{gf} <<@| @{cf}{}@|:@{bf}{}@|"),

    "Failed << {}:{:<{}} [ Exited with code {} ]":
    fmt("@!@{rf}Failed@|    @{rf} <<@| @{cf}{}@|:@{bf}{:<{}}@|[ @{yf}Exited with code @!@{yf}{}@| ]"),

    "Output << {}:{} {}":
    fmt("@!@{kf}Output@|    @!@{kf} <<@| @{cf}{}@|:@{bf}{}@| @!@{kf}{}@|"),

    "Warnings << {}:{} {}":
    fmt("@!@{yf}Warnings@|  @{yf} <<@| @{cf}{}@|:@{bf}{}@| @!@{yf}{}@|"),

    "Errors << {}:{} {}":
    fmt("@!@{rf}Errors@|    @{rf} <<@| @{cf}{}@|:@{bf}{}@| @!@{rf}{}@|"),

    # Interleaved
    "[{}:{}] ":
    fmt("[@{cf}{}@|:@{bf}{}@|] "),

    # Status line
    "[{} {} s] [{}/{} complete] [{}/{} jobs] [{} queued]":
    fmt("[@{pf}{}@| - @{yf}{}@|] [@!@{gf}{}@|/@{gf}{}@| complete] [@!@{gf}{}@|/@{gf}{}@| jobs] [@!@{kf}{}@| queued]"),

    "[{}:{} - {}]":
    fmt("[@{cf}{}@|:@{bf}{}@| - @{yf}{}@|]"),

    "[{}:{} ({}%) - {}]":
    fmt("[@{cf}{}@|:@{bf}{}@| @{bf}({}%)@| - @{yf}{}@|]"),

    # Summary
    "[{}] Summary: All {} jobs completed successfully!":
    fmt("[{}] @/@!Summary:@| @/All @!{}@| @/jobs completed successfully!@|"),

    "[{}] Summary: {} of {} jobs completed.":
    fmt("[{}] @/@!@{yf}Summary:@| @/@!@{yf}{}@| @/@{yf}of @!@{yf}{}@| @/@{yf}jobs completed.@|"),

    "[{}] Warnings: No completed jobs produced warnings.":
    fmt("[{}]   @/@!@{kf}Warnings:  None.@|"),

    "[{}] Warnings: {} completed jobs produced warnings.":
    fmt("[{}]   @/@!@{yf}Warnings:@|  @/@!{}@| @/completed jobs produced warnings.@|"),

    "[{}] Skipped: None.":
    fmt("[{}]   @/@!@{kf}Skipped:   None.@|"),

    "[{}] Skipped: {} jobs skipped.":
    fmt("[{}]   @/@!@{yf}Skipped:@|   @/@!{}@| @/jobs skipped.@|"),

    "[{}] Failed: No jobs failed.":
    fmt("[{}]   @/@!@{kf}Failed:    None.@|"),

    "[{}] Failed: {} jobs failed.":
    fmt("[{}]   @/@!@{rf}Failed:@|    @/@!{}@| @/jobs failed.@|"),

    "[{}] Abandoned: No jobs were abandoned.":
    fmt("[{}]   @/@!@{kf}Abandoned: None.@|"),

    "[{}] Abandoned: {} jobs were abandoned.":
    fmt("[{}]   @/@!@{rf}Abandoned:@| @/@!{}@| @/jobs were abandoned.@|"),

    "[{}]  - {}":
    fmt("[{}]     @{cf}{}@|"),

    "[{}] Runtime: {} total.":
    fmt("[{}] @/@!Runtime:@| @/{} total.@|")
}

color_mapper = ColorMapper(_color_translation_map)

clr = color_mapper.clr


class ConsoleStatusController(threading.Thread):

    """Status thread for displaying events to the console.


    TODO: switch to interleaved output if only one job is running
    """

    def __init__(
            self,
            label,
            job_labels,
            jobs,
            event_queue,
            show_notifications=False,
            show_stage_events=False,
            show_buffered_stdout=False,
            show_buffered_stderr=True,
            show_live_stdout=False,
            show_live_stderr=False,
            show_compact_io=False,
            show_active_status=True,
            show_summary=True,
            show_full_summary=False,
            show_repro_cmd=True,
            active_status_rate=10.0,
            pre_start_time=None):
        """
        :param label: The label for this task (build, clean, etc)
        :param job_labels: The labels to be used for the jobs (packages, tests, etc)
        :param event_queue: The event queue used by an Executor
        :param show_notifications: Show a libnotify notification when the jobs are finished
        :param show_stage_events: Show events relating to stages in each job
        :param show_buffered_stdout: Show stdout from jobs as they finish
        :param show_buffered_stderr: Show stderr from jobs as they finish
        :param show_live_stdout: Show stdout lines from jobs as they're generated
        :param show_live_stderr: Show stdout lines from jobs as they're generated
        :param show_compact_io: Don't print blank lines from redirected io
        :param show_active_status: Periodically show a status line displaying the active jobs
        :param show_summary: Show numbers of jobs that completed with errors and warnings
        :param show_full_summary: Show lists of jobs in each termination category
        :param show_repro_cmd: Show the commands to reproduce failed stages
        :param active_status_rate: The rate in Hz at which the status line should be printed
        :param pre_start_time: The actual start time to report, if preprocessing was done
        """
        super(ConsoleStatusController, self).__init__()

        self.label = label
        self.job_label = job_labels[0]
        self.jobs_label = job_labels[1]
        self.event_queue = event_queue

        self.show_notifications = show_notifications
        self.show_stage_events = show_stage_events
        self.show_buffered_stdout = show_buffered_stdout
        self.show_buffered_stderr = show_buffered_stderr
        self.show_live_stdout = show_live_stdout
        self.show_live_stderr = show_live_stderr
        self.show_compact_io = show_compact_io
        self.show_active_status = show_active_status
        self.show_full_summary = show_full_summary
        self.show_summary = show_summary
        self.show_repro_cmd = show_repro_cmd
        self.active_status_rate = active_status_rate
        self.pre_start_time = pre_start_time

        self.keep_running = True

        # Map from jid -> job
        self.jobs = dict([(j.jid, j) for j in jobs])

        # Compute the max job id length when combined with stage labels
        self.max_jid_length = 1
        if len(self.jobs) > 0:
            self.max_jid_length += max(
                [len(jid) + max([len(s.label) for s in job.stages] or [0])
                 for jid, job
                 in self.jobs.items()]
            )

    def run(self):
        queued_jobs = []
        active_jobs = []
        completed_jobs = {}
        failed_jobs = []
        warned_jobs = []

        cumulative_times = dict()
        start_times = dict()
        active_stages = dict()

        start_time = self.pre_start_time or time.time()

        # Disable the wide log padding if the status is disabled
        if not self.show_active_status:
            disable_wide_log()

        while True:
            # Check if we should stop
            if not self.keep_running:
                wide_log(clr('[{}] An internal error occurred!').format(self.label))
                return
            # Write a continuously-updated status line
            if self.show_active_status:
                # Try to get an event from the queue (non-blocking)
                try:
                    event = self.event_queue.get(False)
                except Empty:
                    # Print live status (overwrites last line)
                    status_line = clr('[{} {} s] [{}/{} complete] [{}/{} jobs] [{} queued]').format(
                        self.label,
                        format_time_delta_short(time.time() - start_time),
                        len(completed_jobs),
                        len(self.jobs),
                        job_server.running_jobs(),
                        job_server.max_jobs(),
                        len(queued_jobs) + len(active_jobs) - len(active_stages)
                    )

                    # Show failed jobs
                    if len(failed_jobs) > 0:
                        status_line += clr(' [@!@{rf}{}@| @{rf}failed@|]').format(len(failed_jobs))

                    # Check load / mem
                    if not job_server.load_ok():
                        status_line += clr(' [@!@{rf}High Load@|]')
                    if not job_server.mem_ok():
                        status_line += clr(' [@!@{rf}Low Memory@|]')

                    # Add active jobs
                    if len(active_jobs) == 0:
                        status_line += clr(' @/@!@{kf}Waiting for jobs...@|')
                    else:
                        active_labels = []

                        for j, (s, t, p) in active_stages.items():
                            d = format_time_delta_short(cumulative_times[j] + time.time() - t)
                            if p == '':
                                active_labels.append(clr('[{}:{} - {}]').format(j, s, d))
                            else:
                                active_labels.append(clr('[{}:{} ({}%) - {}]').format(j, s, p, d))

                        status_line += ' ' + ' '.join(active_labels)

                    # Print the status line
                    # wide_log(status_line)
                    wide_log(status_line, rhs='', end='\r')
                    sys.stdout.flush()
                    if self.active_status_rate > 1E-5:
                        time.sleep(1.0 / self.active_status_rate)
                        continue
            else:
                # Try to get an event from the queue (blocking)
                try:
                    event = self.event_queue.get(True)
                except Empty:
                    break

            # A `None` event is a signal to terminate
            if event is None:
                break

            # Handle the received events
            eid = event.event_id

            if 'JOB_STATUS' == eid:
                queued_jobs = event.data['queued']
                active_jobs = event.data['active']
                completed_jobs = event.data['completed']

                # Check if all jobs have finished in some way
                if all([len(event.data[t]) == 0 for t in ['pending', 'queued', 'active']]):
                    break

            elif 'STARTED_JOB' == eid:
                cumulative_times[event.data['job_id']] = 0.0
                wide_log(clr('Starting >>> {:<{}}').format(
                    event.data['job_id'],
                    self.max_jid_length))

            elif 'FINISHED_JOB' == eid:
                duration = format_time_delta(cumulative_times[event.data['job_id']])

                if event.data['succeeded']:
                    wide_log(clr('Finished <<< {:<{}} [ {} ]').format(
                        event.data['job_id'],
                        self.max_jid_length,
                        duration))
                else:
                    failed_jobs.append(event.data['job_id'])
                    wide_log(clr('Failed <<< {:<{}} [ {} ]').format(
                        event.data['job_id'],
                        self.max_jid_length,
                        duration))

            elif 'ABANDONED_JOB' == eid:
                # Create a human-readable reason string
                if 'DEP_FAILED' == event.data['reason']:
                    direct = event.data['dep_job_id'] == event.data['direct_dep_job_id']
                    if direct:
                        reason = clr('Depends on failed job {}').format(event.data['dep_job_id'])
                    else:
                        reason = clr('Depends on failed job {} via {}').format(
                            event.data['dep_job_id'],
                            event.data['direct_dep_job_id'])
                elif 'PEER_FAILED' == event.data['reason']:
                    reason = clr('Unrelated job failed')
                elif 'MISSING_DEPS' == event.data['reason']:
                    reason = clr('Depends on unknown jobs: {}').format(
                        ', '.join([clr('@!{}@|').format(jid) for jid in event.data['dep_ids']]))

                wide_log(clr('Abandoned <<< {:<{}} [ {} ]').format(
                    event.data['job_id'],
                    self.max_jid_length,
                    reason))

            elif 'STARTED_STAGE' == eid:
                active_stages[event.data['job_id']] = [event.data['stage_label'], event.time, '']
                start_times[event.data['job_id']] = event.time

                if self.show_stage_events:
                    wide_log(clr('Starting >> {}:{}').format(
                        event.data['job_id'],
                        event.data['stage_label']))

            elif 'STAGE_PROGRESS' == eid:
                active_stages[event.data['job_id']][2] = event.data['percent']

            elif 'SUBPROCESS' == eid:
                if self.show_stage_events:
                    wide_log(clr('Subprocess > {}:{} `{}`').format(
                        event.data['job_id'],
                        event.data['stage_label'],
                        event.data['stage_repro']))

            elif 'FINISHED_STAGE' == eid:
                # Get the stage duration
                duration = event.time - start_times[event.data['job_id']]
                cumulative_times[event.data['job_id']] += duration

                # This is no longer the active stage for this job
                del active_stages[event.data['job_id']]

                header_border = None
                header_title = None
                lines = []
                footer_title = None
                footer_border = None

                # Generate headers / borders for output
                if event.data['succeeded']:
                    footer_title = clr(
                        'Finished << {}:{}').format(
                            event.data['job_id'],
                            event.data['stage_label'])

                    if len(event.data['stderr']) > 0:
                        # Mark that this job warned about something
                        if event.data['job_id'] not in warned_jobs:
                            warned_jobs.append(event.data['job_id'])

                        # Output contains warnings
                        header_border = clr('@!@{yf}' + '_' * (terminal_width() - 1) + '@|')
                        header_title = clr(
                            'Warnings << {}:{} {}').format(
                                event.data['job_id'],
                                event.data['stage_label'],
                                event.data['logfile_filename'])
                        footer_border = clr('@{yf}' + '.' * (terminal_width() - 1) + '@|')
                    else:
                        # Normal output, no warnings
                        header_title = clr(
                            'Output << {}:{} {}').format(
                                event.data['job_id'],
                                event.data['stage_label'],
                                event.data['logfile_filename'])

                    # Don't print footer title
                    if not self.show_stage_events:
                        footer_title = None
                else:
                    # Output contains errors
                    header_border = clr('@!@{rf}' + '_' * (terminal_width() - 1) + '@|')
                    header_title = clr(
                        'Errors << {}:{} {}').format(
                            event.data['job_id'],
                            event.data['stage_label'],
                            event.data['logfile_filename'])
                    footer_border = clr('@{rf}' + '.' * (terminal_width() - 1) + '@|')

                    footer_title = clr(
                        'Failed << {}:{:<{}} [ Exited with code {} ]').format(
                            event.data['job_id'],
                            event.data['stage_label'],
                            max(0, self.max_jid_length - len(event.data['job_id'])),
                            event.data['retcode'])

                if self.show_buffered_stdout:
                    if len(event.data['interleaved']) > 0:
                        lines = [
                            l
                            for l in event.data['interleaved'].splitlines()
                            if (self.show_compact_io is False or len(l.strip()) > 0)
                        ]
                    else:
                        header_border = None
                        header_title = None
                        footer_border = None
                elif self.show_buffered_stderr:
                    if len(event.data['stderr']) > 0:
                        lines = [
                            l
                            for l in event.data['stderr'].splitlines()
                            if (self.show_compact_io is False or len(l.strip()) > 0)
                        ]
                    else:
                        header_border = None
                        header_title = None
                        footer_border = None

                if self.show_repro_cmd and len(lines) > 0:
                    if event.data['repro'] is not None:
                        lines.insert(0, clr('@!@{kf}{}@|').format(event.data['repro']))

                # Print the output
                if header_border:
                    wide_log(header_border)
                if header_title:
                    wide_log(header_title)
                if len(lines) > 0:
                    wide_log('\n'.join(lines))
                if footer_border:
                    wide_log(footer_border)
                if footer_title:
                    wide_log(footer_title)

            elif 'STDERR' == eid:
                if self.show_live_stderr:
                    prefix = clr('[{}:{}] ').format(
                        event.data['job_id'],
                        event.data['stage_label'])
                    wide_log(''.join(prefix + l for l in event.data['data'].splitlines(True)))

            elif 'STDOUT' == eid:
                if self.show_live_stdout:
                    prefix = clr('[{}:{}] ').format(
                        event.data['job_id'],
                        event.data['stage_label'])
                    wide_log(''.join(prefix + l for l in event.data['data'].splitlines(True)))

            elif 'MESSAGE' == eid:
                wide_log(event.data['msg'])

        if not self.show_summary:
            return

        # Print final runtime
        wide_log(clr('[{}] Runtime: {} total.').format(
            self.label,
            format_time_delta(time.time() - start_time)))

        # Print error summary
        if len(completed_jobs) == len(self.jobs) and all(completed_jobs.items()) and len(failed_jobs) == 0:
            if self.show_notifications:
                notify("{} Succeded".format(self.label.capitalize()),
                       "{} {} completed with no warnings.".format(
                    len(completed_jobs), self.jobs_label))

            wide_log(clr('[{}] Summary: All {} jobs completed successfully!').format(self.label, len(self.jobs)))
        else:
            wide_log(clr('[{}] Summary: {} of {} jobs completed.').format(
                self.label,
                len([succeeded for jid, succeeded in completed_jobs.items() if succeeded]),
                len(self.jobs)))

        if len(warned_jobs) == 0:
            wide_log(clr('[{}] Warnings: No completed jobs produced warnings.').format(
                self.label))
        else:
            if self.show_notifications:
                notify("{} Produced Warnings".format(self.label.capitalize()),
                       "{} {} completed with warnings.".format(
                    len(warned_jobs), self.jobs_label))

            wide_log(clr('[{}] Warnings: {} completed jobs produced warnings.').format(
                self.label,
                len(warned_jobs)))
            if self.show_full_summary:
                for jid in warned_jobs:
                    wide_log(clr('[{}]  - {}').format(
                        self.label,
                        jid))

        all_abandoned_jobs = [j for j in self.jobs if j not in completed_jobs]
        if len(all_abandoned_jobs) == 0:
            wide_log(clr('[{}] Abandoned: No jobs were abandoned.').format(
                self.label))
        else:
            if self.show_notifications:
                notify("{} Incomplete".format(self.label.capitalize()),
                       "{} {} were abandoned.".format(
                    len(all_abandoned_jobs), self.jobs_label))

            wide_log(clr('[{}] Abandoned: {} jobs were abandoned.').format(
                self.label,
                len(all_abandoned_jobs)))
            if self.show_full_summary:
                for jid in all_abandoned_jobs:
                    wide_log(clr('[{}]  - {}').format(
                        self.label,
                        jid))

        if len(failed_jobs) == 0:
            wide_log(clr('[{}] Failed: No jobs failed.').format(
                self.label))
        else:
            if self.show_notifications:
                notify("{} Failed".format(self.label.capitalize()),
                       "{} {} failed.".format(
                    len(failed_jobs), self.jobs_label))

            wide_log(clr('[{}] Failed: {} jobs failed.').format(
                self.label,
                len(failed_jobs)))
            if self.show_full_summary:
                for jid in failed_jobs:
                    wide_log(clr('[{}]  - {}').format(
                        self.label,
                        jid))


def print_error_summary(verb, errors, no_notify, log_dir):
    wide_log(clr("[" + verb + "] There were '" + str(len(errors)) + "' @!@{rf}errors@|:"))
    if not no_notify:
        notify("Build Failed", "there were {0} errors".format(len(errors)))
    for error in errors:
        if error.event_type == 'exit':
            wide_log("""\
Executor '{exec_id}' had an unhandled exception while processing package '{package}':

{data[exc]}""".format(exec_id=error.executor_id + 1, **error.__dict__))
        else:
            wide_log(clr("""
@{rf}Failed@| to build package '@{cf}{package}@|' because the following command:

@!@{kf}# Command to reproduce:@|
cd {location} && {cmd.cmd_str}; cd -

@!@{kf}# Path to log:@|
cat {log_dir}

@{rf}Exited@| with return code: @!{retcode}@|""").format(
                package=error.package,
                log_dir=os.path.join(log_dir, error.package + '.log'),
                **error.data)
            )


def print_items_in_columns(items_in, number_of_columns):
    number_of_items_in_line = 0
    line_template = "{}" * number_of_columns
    line_items = []
    items = list(items_in)
    while items:
        line_items.append(items.pop(0))
        number_of_items_in_line += 1
        if number_of_items_in_line == number_of_columns:
            wide_log(line_template.format(*line_items))
            line_items = []
            number_of_items_in_line = 0
    if line_items:
        wide_log(("{}" * len(line_items)).format(*line_items))


def print_build_summary(context, packages_to_be_built, completed_packages, failed_packages):
    # Calculate the longest package name
    max_name_len = max([len(pkg.name) for _, pkg in context.packages])

    def get_template(template_name, column_width):
        templates = {
            'successful': " @!@{gf}Successful@| @{cf}{package:<" + str(column_width) + "}@|",
            'failed': " @!@{rf}Failed@|     @{cf}{package:<" + str(column_width) + "}@|",
            'not_built': " @!@{kf}Not built@|  @{cf}{package:<" + str(column_width) + "}@|",
        }
        return templates[template_name]

    # Setup templates for comparison
    successful_template = get_template('successful', max_name_len)
    failed_template = get_template('failed', max_name_len)
    not_built_template = get_template('not_built', max_name_len)
    # Calculate the maximum _printed_ length for each template
    faux_package_name = ("x" * max_name_len)
    templates = [
        remove_ansi_escape(clr(successful_template).format(package=faux_package_name)),
        remove_ansi_escape(clr(failed_template).format(package=faux_package_name)),
        remove_ansi_escape(clr(not_built_template).format(package=faux_package_name)),
    ]
    # Calculate the longest column using the longest template
    max_column_len = max([len(template) for template in templates])
    # Calculate the number of columns
    number_of_columns = (terminal_width() / max_column_len) or 1

    successfuls = {}
    faileds = {}
    not_builts = {}
    non_whitelisted = {}
    blacklisted = {}

    for (_, pkg) in context.packages:
        if pkg.name in context.blacklist:
            blacklisted[pkg.name] = clr(not_built_template).format(package=pkg.name)
        elif len(context.whitelist) > 0 and pkg.name not in context.whitelist:
            non_whitelisted[pkg.name] = clr(not_built_template).format(package=pkg.name)
        elif pkg.name in completed_packages:
            successfuls[pkg.name] = clr(successful_template).format(package=pkg.name)
        else:
            if pkg.name in failed_packages:
                faileds[pkg.name] = clr(failed_template).format(package=pkg.name)
            else:
                not_builts[pkg.name] = clr(not_built_template).format(package=pkg.name)

    # Combine successfuls and not_builts, sort by key, only take values
    wide_log("")
    wide_log("Build summary:")
    combined = dict(successfuls)
    combined.update(not_builts)
    non_failed = [v for k, v in sorted(combined.items(), key=operator.itemgetter(0))]
    print_items_in_columns(non_failed, number_of_columns)

    # Print out whitelisted packages
    if len(non_whitelisted) > 0:
        wide_log("")
        wide_log("Non-Whitelisted Packages:")
        non_whitelisted_list = [v for k, v in sorted(non_whitelisted.items(), key=operator.itemgetter(0))]
        print_items_in_columns(non_whitelisted_list, number_of_columns)

    # Print out blacklisted packages
    if len(blacklisted) > 0:
        wide_log("")
        wide_log("Blacklisted Packages:")
        blacklisted_list = [v for k, v in sorted(blacklisted.items(), key=operator.itemgetter(0))]
        print_items_in_columns(blacklisted_list, number_of_columns)

    # Faileds only, sort by key, only take values
    failed = [v for k, v in sorted(faileds.items(), key=operator.itemgetter(0))]
    if len(failed) > 0:
        wide_log("")
        wide_log("Failed packages:")
        print_items_in_columns(failed, number_of_columns)
    else:
        wide_log("")
        wide_log("All packages built successfully.")

    wide_log("")
    wide_log(clr("[{0}] @!@{gf}Successfully@| built '@!@{cf}{1}@|' packages, "
                 "@!@{rf}failed@| to build '@!@{cf}{2}@|' packages, "
                 "and @!@{kf}did not try to build@| '@!@{cf}{3}@|' packages.").format(
        len(successfuls), len(faileds), len(not_builts)
    ))
