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

import math
import itertools
import sys
import threading
import time
from queue import Empty

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
    "[{}] Summary: All {} {} succeeded!":
    fmt("[{}] @/@!@{gf}Summary:@| @/All @!{}@| @/{} succeeded!@|"),

    "[{}] Summary: {} of {} {} succeeded.":
    fmt("[{}] @/@!@{yf}Summary:@| @/@!{}@| @/of @!{}@| @/{} succeeded.@|"),

    "[{}] Warnings: None.":
    fmt("[{}]   @/@!@{kf}Warnings:  None.@|"),

    "[{}] Warnings: {} {} succeeded with warnings.":
    fmt("[{}]   @/@!@{yf}Warnings:@|  @/@!{}@| @/{} succeeded with warnings.@|"),

    "[{}] Skipped: None.":
    fmt("[{}]   @/@!@{kf}Skipped:   None.@|"),

    "[{}] Skipped: {} {} skipped.":
    fmt("[{}]   @/@!@{yf}Skipped:@|   @/@!{}@| @/{} skipped.@|"),

    "[{}] Ignored: None.":
    fmt("[{}]   @/@!@{kf}Ignored:   None.@|"),

    "[{}] Ignored: {} {} were skipped or are blacklisted.":
    fmt("[{}]   @/@!@{pf}Ignored:@|   @/@!{}@| @/{} were skipped or are blacklisted.@|"),

    "[{}] Failed: No {} failed.":
    fmt("[{}]   @/@!@{kf}Failed:    None.@|"),

    "[{}] Failed: {} {} failed.":
    fmt("[{}]   @/@!@{rf}Failed:@|    @/@!{}@| @/{} failed.@|"),

    "[{}] Abandoned: No {} were abandoned.":
    fmt("[{}]   @/@!@{kf}Abandoned: None.@|"),

    "[{}] Abandoned: {} {} were abandoned.":
    fmt("[{}]   @/@!@{rf}Abandoned:@| @/@!{}@| @/{} were abandoned.@|"),

    "[{}]  - {}":
    fmt("[{}]     @{cf}{}@|"),

    "[{}] Runtime: {} total.":
    fmt("[{}] @/@!Runtime:@| @/{} total.@|")
}

color_mapper = ColorMapper(_color_translation_map)

clr = color_mapper.clr


def print_items_in_columns(items, n_cols):
    """Print items in columns

    :param items: list of tuples (identifier, template) where the template takes `jid` as a parameter
    :param n_cols: number of columns
    """

    # Format all the items
    formatted_items = [t.format(jid=j) for j, t in items]

    # Compute the number of rows
    n_items = len(items)
    if n_items <= n_cols:
        n_cols = 1
    n_rows = int(math.ceil(n_items / float(n_cols)))

    # Print each row
    for r in range(n_rows):
        wide_log(''.join(formatted_items[(r * n_cols):((r + 1) * n_cols)]))


class ConsoleStatusController(threading.Thread):

    """Status thread for displaying events to the console.


    TODO: switch to interleaved output if only one job is running
    """

    def __init__(
            self,
            label,
            job_labels,
            jobs,
            max_toplevel_jobs,
            available_jobs,
            whitelisted_jobs,
            blacklisted_jobs,
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
        self.max_toplevel_jobs = max_toplevel_jobs

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

        self.available_jobs = available_jobs
        self.blacklisted_jobs = blacklisted_jobs
        self.whitelisted_jobs = whitelisted_jobs

        # Compute the max job id length when combined with stage labels
        self.max_jid_length = 1
        if len(self.jobs) > 0:
            self.max_jid_length += max(
                [len(jid) + max([len(s.label) for s in job.stages] or [0])
                 for jid, job
                 in self.jobs.items()]
            )

    def print_exec_summary(self, completed_jobs, warned_jobs, failed_jobs):
        """
        Print verbose execution summary.
        """

        # Calculate the longest jid
        max_jid_len = max([len(jid) for jid in self.available_jobs])

        templates = {
            'successful': clr(" [@!@{gf}Successful@|] @{cf}{jid:<%d}@|" % max_jid_len),
            'warned': clr(" [    @!@{yf}Warned@|] @{cf}{jid:<%d}@|" % max_jid_len),
            'failed': clr(" [    @!@{rf}Failed@|] @{cf}{jid:<%d}@|" % max_jid_len),
            'ignored': clr(" [   @!@{kf}Ignored@|] @{cf}{jid:<%d}@|" % max_jid_len),
            'abandoned': clr(" [ @!@{rf}Abandoned@|] @{cf}{jid:<%d}@|" % max_jid_len),
        }

        # Calculate the maximum _printed_ length for each template
        max_column_len = max([
            len(remove_ansi_escape(t.format(jid=("?" * max_jid_len))))
            for t in templates.values()
        ])

        # Calculate the number of columns
        number_of_columns = int((terminal_width() / max_column_len) or 1)

        # Construct different categories of jobs (jid -> output template)
        successfuls = {}
        warneds = {}
        faileds = {}
        ignoreds = {}
        abandoneds = {}
        non_whitelisted = {}
        blacklisted = {}

        # Give each package an output template to use
        for jid in self.available_jobs:
            if jid in self.blacklisted_jobs:
                blacklisted[jid] = templates['ignored']
            elif jid not in self.jobs:
                ignoreds[jid] = templates['ignored']
            elif len(self.whitelisted_jobs) > 0 and jid not in self.whitelisted_jobs:
                non_whitelisted[jid] = templates['ignored']
            elif jid in completed_jobs:
                if jid in failed_jobs:
                    faileds[jid] = templates['failed']
                elif jid in warned_jobs:
                    warneds[jid] = templates['warned']
                else:
                    successfuls[jid] = templates['successful']
            else:
                abandoneds[jid] = templates['abandoned']

        # Combine successfuls and ignoreds, sort by key
        if len(successfuls) + len(ignoreds) > 0:
            wide_log("")
            wide_log(clr("[{}] Successful {}:").format(self.label, self.jobs_label))
            wide_log("")
            print_items_in_columns(
                sorted(itertools.chain(successfuls.items(), ignoreds.items())),
                number_of_columns)
        else:
            wide_log("")
            wide_log(clr("[{}] No {} succeeded.").format(self.label, self.jobs_label))
            wide_log("")

        # Print out whitelisted jobs
        if len(non_whitelisted) > 0:
            wide_log("")
            wide_log(clr("[{}] Non-whitelisted {}:").format(self.label, self.jobs_label))
            wide_log("")
            print_items_in_columns(sorted(non_whitelisted.items()), number_of_columns)

        # Print out blacklisted jobs
        if len(blacklisted) > 0:
            wide_log("")
            wide_log(clr("[{}] Blacklisted {}:").format(self.label, self.jobs_label))
            wide_log("")
            print_items_in_columns(sorted(blacklisted.items()), number_of_columns)

        # Print out jobs that failed
        if len(faileds) > 0:
            wide_log("")
            wide_log(clr("[{}] Failed {}:").format(self.label, self.jobs_label))
            wide_log("")
            print_items_in_columns(sorted(faileds.items()), number_of_columns)

        # Print out jobs that were abandoned
        if len(abandoneds) > 0:
            wide_log("")
            wide_log(clr("[{}] Abandoned {}:").format(self.label, self.jobs_label))
            wide_log("")
            print_items_in_columns(sorted(abandoneds.items()), number_of_columns)

        wide_log("")

    def print_compact_summary(self, completed_jobs, warned_jobs, failed_jobs):
        """Print a compact build summary."""

        notification_title = ""
        notification_msg = []
        # Print error summary
        if len(completed_jobs) == len(self.jobs) and all(completed_jobs.items()) and len(failed_jobs) == 0:
            notification_title = "{} Succeeded".format(self.label.capitalize())
            notification_msg.append("All {} {} succeeded!".format(len(self.jobs), self.jobs_label))

            wide_log(clr('[{}] Summary: All {} {} succeeded!').format(
                self.label,
                len(self.jobs),
                self.jobs_label))
        else:
            notification_msg.append("{} of {} {} succeeded.".format(
                len([succeeded for jid, succeeded in completed_jobs.items() if succeeded]),
                len(self.jobs), self.jobs_label))
            wide_log(clr('[{}] Summary: {} of {} {} succeeded.').format(
                self.label,
                len([succeeded for jid, succeeded in completed_jobs.items() if succeeded]),
                len(self.jobs),
                self.jobs_label))

        # Display number of ignored jobs (jobs which shouldn't have been built)
        all_ignored_jobs = [j for j in self.available_jobs if j not in self.jobs]
        if len(all_ignored_jobs) == 0:
            wide_log(clr('[{}] Ignored: None.').format(
                self.label))
        else:
            notification_msg.append("{} {} were skipped.".format(len(all_ignored_jobs), self.jobs_label))
            wide_log(clr('[{}] Ignored: {} {} were skipped or are blacklisted.').format(
                self.label,
                len(all_ignored_jobs),
                self.jobs_label))

        # Display number of jobs which produced warnings
        if len(warned_jobs) == 0:
            wide_log(clr('[{}] Warnings: None.').format(
                self.label))
        else:
            notification_title = "{} Succeeded with Warnings".format(self.label.capitalize())
            notification_msg.append("{} {} succeeded with warnings.".format(len(warned_jobs), self.jobs_label))

            wide_log(clr('[{}] Warnings: {} {} succeeded with warnings.').format(
                self.label,
                len(warned_jobs),
                self.jobs_label))

        # Display number of abandoned jobs
        all_abandoned_jobs = [j for j in self.jobs if j not in completed_jobs]
        if len(all_abandoned_jobs) == 0:
            wide_log(clr('[{}] Abandoned: No {} were abandoned.').format(
                self.label,
                self.jobs_label))
        else:
            notification_title = "{} Incomplete".format(self.label.capitalize())
            notification_msg.append("{} {} were abandoned.".format(len(all_abandoned_jobs), self.jobs_label))

            wide_log(clr('[{}] Abandoned: {} {} were abandoned.').format(
                self.label,
                len(all_abandoned_jobs),
                self.jobs_label))

        # Display number of failed jobs
        if len(failed_jobs) == 0:
            wide_log(clr('[{}] Failed: No {} failed.').format(
                self.label,
                self.jobs_label))
        else:
            notification_title = "{} Failed".format(self.label.capitalize())
            notification_msg.append("{} {} failed.".format(len(failed_jobs), self.jobs_label))

            wide_log(clr('[{}] Failed: {} {} failed.').format(
                self.label,
                len(failed_jobs),
                self.jobs_label))

        if self.show_notifications:
            if len(failed_jobs) != 0:
                notify(notification_title, "\n".join(notification_msg), icon_image='catkin_icon_red.png')
            elif len(warned_jobs) != 0:
                notify(notification_title, "\n".join(notification_msg), icon_image='catkin_icon_yellow.png')
            else:
                notify(notification_title, "\n".join(notification_msg))

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
        last_update_time = time.time()

        # If the status rate is too low, just disable it
        if self.active_status_rate < 1E-3:
            self.show_active_status = False
        else:
            update_duration = 1.0 / self.active_status_rate

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
                    # Determine if the status should be shown based on the desired
                    # status rate
                    elapsed_time = time.time() - last_update_time
                    show_status_now = elapsed_time > update_duration

                    if show_status_now:
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

                        # Store this update time
                        last_update_time = time.time()
                    else:
                        time.sleep(max(0.0, min(update_duration - elapsed_time, 0.01)))

                    # Only continue when no event was received
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
                header_border_file = sys.stdout
                header_title = None
                header_title_file = sys.stdout
                lines = []
                footer_title = None
                footer_title_file = sys.stdout
                footer_border = None
                footer_border_file = sys.stdout

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
                        header_border_file = sys.stderr
                        header_title = clr(
                            'Warnings << {}:{} {}').format(
                                event.data['job_id'],
                                event.data['stage_label'],
                                event.data['logfile_filename'])
                        header_title_file = sys.stderr
                        footer_border = clr('@{yf}' + '.' * (terminal_width() - 1) + '@|')
                        footer_border_file = sys.stderr
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
                    header_border_file = sys.stderr
                    header_title = clr(
                        'Errors << {}:{} {}').format(
                            event.data['job_id'],
                            event.data['stage_label'],
                            event.data['logfile_filename'])
                    header_title_file = sys.stderr
                    footer_border = clr('@{rf}' + '.' * (terminal_width() - 1) + '@|')
                    footer_border_file = sys.stderr

                    footer_title = clr(
                        'Failed << {}:{:<{}} [ Exited with code {} ]').format(
                            event.data['job_id'],
                            event.data['stage_label'],
                            max(0, self.max_jid_length - len(event.data['job_id'])),
                            event.data['retcode'])
                    footer_title_file = sys.stderr

                lines_target = sys.stdout
                if self.show_buffered_stdout:
                    if len(event.data['interleaved']) > 0:
                        lines = [
                            line + '\n'
                            for line in event.data['interleaved'].splitlines()
                            if (self.show_compact_io is False or len(line.strip()) > 0)
                        ]
                    else:
                        header_border = None
                        header_title = None
                        footer_border = None
                elif self.show_buffered_stderr:
                    if len(event.data['stderr']) > 0:
                        lines = [
                            line + '\n'
                            for line in event.data['stderr'].splitlines()
                            if (self.show_compact_io is False or len(line.strip()) > 0)
                        ]
                        lines_target = sys.stderr
                    else:
                        header_border = None
                        header_title = None
                        footer_border = None

                if len(lines) > 0:
                    if self.show_repro_cmd:
                        if event.data['repro'] is not None:
                            lines.append(clr('@!@{kf}{}@|\n').format(event.data['repro']))

                    # Print the output
                    if header_border:
                        wide_log(header_border, file=header_border_file)
                    if header_title:
                        wide_log(header_title, file=header_title_file)
                    if len(lines) > 0:
                        wide_log(''.join(lines), end='\n', file=lines_target)
                    if footer_border:
                        wide_log(footer_border, file=footer_border_file)
                    if footer_title:
                        wide_log(footer_title, file=footer_title_file)

            elif 'STDERR' == eid:
                if self.show_live_stderr and len(event.data['data']) > 0:
                    wide_log(self.format_interleaved_lines(event.data), file=sys.stderr)

            elif 'STDOUT' == eid:
                if self.show_live_stdout and len(event.data['data']) > 0:
                    wide_log(self.format_interleaved_lines(event.data))

            elif 'MESSAGE' == eid:
                wide_log(event.data['msg'])

        # Print the full summary
        if self.show_full_summary:
            self.print_exec_summary(completed_jobs, warned_jobs, failed_jobs)

        # Print a compact summary
        if self.show_summary or self.show_full_summary:
            self.print_compact_summary(completed_jobs, warned_jobs, failed_jobs)

        # Print final runtime
        wide_log(clr('[{}] Runtime: {} total.').format(
            self.label,
            format_time_delta(time.time() - start_time)))

    def format_interleaved_lines(self, data):
        if self.max_toplevel_jobs != 1:
            prefix = clr('[{}:{}] ').format(
                data['job_id'],
                data['stage_label'])
        else:
            prefix = ''

        # This is used to clear the status bar that is printed in the current line
        clear_line = '\r{}\r'.format(' ' * terminal_width())
        suffix = clr('@|')
        lines = data['data'].splitlines()
        return clear_line + '\n'.join(prefix + line + suffix for line in lines)
