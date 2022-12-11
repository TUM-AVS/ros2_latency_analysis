#!/usr/bin/python3.10

import argparse
import logging
import os.path
import shutil
import subprocess
import sys
import time
import traceback
from datetime import datetime, timedelta
from enum import Enum
from pathlib import Path

import ruamel.yaml

logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class TaskState(Enum):
    INITIALIZED = 1
    STARTING = 2
    RUNNING = 3
    STOPPING = 4
    STOPPED = 5
    CRASHED = 6


class RunnerState(Enum):
    SETUP = 0
    STARTUP = 1
    RUNNING = 2
    TEARDOWN = 3
    STOPPED = 4


class TaskManager:
    def __init__(self, cfg_path, tasks, startup_task: 'GenericTask', cleanup_task: 'GenericTask', start_deps, runtime_s):
        self.tasks = tasks
        self.startup_task = startup_task
        self.cleanup_task = cleanup_task
        self.cfg_path = cfg_path
        self.cfg_name = Path(cfg_path).stem
        self.logger = logging.getLogger(self.cfg_name)
        self.runtime_s = runtime_s
        self._state = RunnerState.SETUP
        self.t_started = None

        self.start_order = []
        tasks_to_be_ordered = list(self.tasks.keys())
        while len(self.start_order) < len(tasks):
            can_start = [t for t in tasks_to_be_ordered if
                         t not in self.start_order and all(dep in self.start_order for dep in start_deps[t])]
            self.start_order += can_start

    def run(self):
        self.logger.info("[RUNNER] SETUP")
        if self.startup_task:
            self.startup_task.start()
            while self._state == RunnerState.SETUP:
                self.startup_task.poll()
                if self.startup_task.state().value >= TaskState.STOPPED.value:
                    self._state = RunnerState.STARTUP
        else:
            self._state = RunnerState.STARTUP

        currently_starting = None
        self.logger.info("[RUNNER] STARTUP")
        while self._state == RunnerState.STARTUP:
            for task_name, task in self.tasks.items():
                task.poll()

            if not currently_starting or currently_starting.state().value >= TaskState.RUNNING.value:
                if self.start_order:
                    currently_starting = self.tasks[self.start_order.pop(0)]
                    currently_starting.start()
                else:
                    self._state = RunnerState.RUNNING

        self.t_started = datetime.now()
        if self.runtime_s is None:
            runtime_str = "until all tasks terminate (no runtime given in config)"
        else:
            runtime_str = f"for {self.runtime_s:.1f}s"
        self.logger.info(f"[RUNNER] RUNNING {runtime_str}")
        all_terminated = False
        while self._state == RunnerState.RUNNING:
            for task in self.tasks.values():
                task.poll()

            all_terminated = all(t.state().value >= TaskState.STOPPED.value for t in self.tasks.values())
            runtime_over = self.runtime_s is not None and datetime.now() - self.t_started > timedelta(
                seconds=self.runtime_s)
            if all_terminated or runtime_over:
                self._state = RunnerState.TEARDOWN

        self.logger.info(f"[RUNNER] TEARDOWN: {'all tasks finished' if all_terminated else 'runtime is over'}")
        if self.cleanup_task:
            self.logger.info("[RUNNER] Commencing CLEANUP")
            self.cleanup_task.start()
            while self.cleanup_task.state().value < TaskState.STOPPED.value:
                self.cleanup_task.poll()
            self.logger.info("[RUNNER] Finished CLEANUP")

        while self._state == RunnerState.TEARDOWN:
            for task in self.tasks.values():
                try:
                    task.poll()
                    task.stop()
                except Exception:
                    traceback.print_exc()

            time.sleep(.1)
            if all(t.state().value >= TaskState.STOPPED.value for t in self.tasks.values()):
                self._state = RunnerState.STOPPED

        self.logger.info("[RUNNER] STOPPED")

        run_dir = f"artifacts/{self.cfg_name}"
        self.logger.info(f"[RUNNER] Copying artifacts to {os.path.abspath(run_dir)}")
        shutil.rmtree(run_dir, ignore_errors=True)
        os.makedirs(run_dir)
        for task in self.tasks.values():
            task.copy_artifacts(run_dir)
        shutil.copy(self.cfg_path, run_dir)
        self.logger.info("[RUNNER] DONE")


def parse_config(config_path, cfg_dict, env_vars):

    tasks = {}
    start_deps = {}

    for task_name, task_dict in cfg_dict["tasks"].items():
        if "start_deps" in task_dict:
            task_deps = task_dict["start_deps"]
            del task_dict["start_deps"]
        else:
            task_deps = []

        task_class = TASK_MAP.get(task_name) or GenericTask
        task = task_class(task_name, task_dict, env_vars)
        start_deps[task_name] = task_deps
        tasks[task_name] = task

    if "cleanup" in cfg_dict:
        cleanup_task = GenericTask("cleanup", cfg_dict["cleanup"], env_vars)
    else:
        cleanup_task = None

    if "startup" in cfg_dict:
        startup_task = GenericTask("startup", cfg_dict["startup"], env_vars)
    else:
        startup_task = None

    runtime_s = cfg_dict.get("runtime_s") or None
    if runtime_s is not None:
        runtime_s = float(runtime_s)

    return TaskManager(config_path, tasks, startup_task, cleanup_task, start_deps, runtime_s)


class Task:
    def __init__(self, name, cfg_dict, env_vars):
        self.name = name
        self.logger = logging.getLogger(self.name)
        self.commands = cfg_dict["commands"]
        self.artifacts_loc = cfg_dict["artifact_location"] if "artifact_location" in cfg_dict else None
        if self.artifacts_loc is not None:
            self.artifacts_loc = os.path.expanduser(os.path.expandvars(self.artifacts_loc))

        self.shell = None
        self.env = env_vars
        self._state = TaskState.INITIALIZED
        self._line_actions = []

    def copy_artifacts(self, target_dir):
        self.logger.debug(f"[RUNNER] Copy artifacts called for {target_dir}")
        if not self.artifacts_loc:
            return 0

        task_dir = os.path.join(target_dir, self.name)
        os.makedirs(task_dir)

        self.logger.info(f"[RUNNER] Copying {self.artifacts_loc} to {task_dir}")

        try:
            return subprocess.run(["cp", "-r", self.artifacts_loc, task_dir], timeout=60).returncode
        except subprocess.TimeoutExpired:
            self.logger.error(f"[RUNNER] Copying timed out, killed copy process.")
            return 1

    def poll(self):
        if not self.shell:
            return

        try:
            new_lines = self.shell.stdout.readlines()
        except Exception:
            self.logger.error(f"[RUNNER] pipe closed")
            return
        for line in new_lines:
            self.logger.info(f"[OUT] {line.rstrip()}")
        self._process_line_actions(new_lines)

    def _process_line_actions(self, new_lines):
        leftover_lines = new_lines

        if self._line_actions:
            action = self._line_actions[0]
            leftover_lines, done = action(leftover_lines)
            if done:
                self._line_actions = self._line_actions[1:]

    def state(self) -> TaskState:
        if not self.shell:
            return self._state

        shell_status = self.shell.poll()

        if shell_status is None:  # running
            pass
        elif shell_status == 0:  # exited (code 0)
            if self._state.value < TaskState.STOPPED.value:
                self.logger.info("[RUNNER] stopped")
            self._state = TaskState.STOPPED
        else:
            if self._state.value < TaskState.STOPPED.value:
                self.logger.info("[RUNNER] crashed")
            self._state = TaskState.CRASHED

        return self._state

    def _wait_for_line(self, line_content):
        def action(lines: list):
            while lines and (line := lines.pop(0)):
                if line_content in line:
                    self.logger.info(f"[RUNNER] found line containing '{line_content}'")
                    return lines, True
            return [], False

        self._line_actions.append(action)

    def _wait_seconds(self, seconds):
        t_start = datetime.now()
        dt = timedelta(seconds=seconds)

        def action(lines: list):
            if datetime.now() - t_start < dt:
                return [], False
            return [], True

        self._line_actions.append(action)

    def _set_started(self):
        self.logger.info("[RUNNER] ready")
        self._state = TaskState.RUNNING

    def _do_action(self, callback):
        def action(lines: list):
            callback()
            return [], True

        self._line_actions.append(action)

    def start(self):
        if self.state() != TaskState.INITIALIZED:
            return True
        self._state = TaskState.STARTING
        self.logger.info("[RUNNER] starting...")

        env_commands = [f"export {k}={v}" for k, v in self.env.items()]
        command_str = '; '.join(env_commands + self.commands)
        logger.debug(command_str)

        # We give the commands via stdin instead of -c, because otherwise
        # we kill ourselves with the pkill commands in the startup phase
        self.shell = subprocess.Popen(["/bin/bash"],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True,
                                      bufsize=1,
                                      universal_newlines=True)
        os.set_blocking(self.shell.stdout.fileno(), False)
        self.shell.stdin.write(command_str + '; exit 0\n')

        self.logger.debug("[RUNNER] shell started")

    def stop(self):
        if self.state().value >= TaskState.STOPPING.value:
            return
        self._state = TaskState.STOPPING
        self.logger.info("[RUNNER] stopping")

    def __str__(self):
        return f"Task(name={self.name})"


class GenericTask(Task):
    def start(self):
        if super().start():
            return True
        self._do_action(self._set_started)


class AutowareTask(Task):
    def start(self):
        if super().start():
            return True
        self._do_action(self._set_started)


class TracingTask(Task):
    def start(self):
        if super().start():
            return True
        self._wait_for_line("press enter to start...")
        self._do_action(lambda: self.shell.stdin.write("\n"))
        self._wait_for_line("press enter to stop...")
        self._do_action(self._set_started)

    def stop(self):
        if self.state() != TaskState.RUNNING:
            return
        self.shell.stdin.write("\n")
        self._wait_for_line("stopping & destroying tracing session")
        self._do_action(super().stop)


class RosbagTask(GenericTask):
    pass


class TraceCpuTask(GenericTask):
    pass


class TraceMemoryTask(GenericTask):
    pass


TASK_MAP = {
    "autoware": AutowareTask,
    "tracing": TracingTask,
    "rosbag": RosbagTask,
    "trace_cpu_usage": TraceCpuTask,
    "trace_memory_usage": TraceMemoryTask,
}


def run(config_path, env):
    yaml = ruamel.yaml.YAML()

    for env_var, value in env.items():
        os.environ[env_var] = value
        logger.debug(f"[RUNNER][ENV] {env_var}={value}")

    try:
        with open(config_path) as f:
            runner_cfg = yaml.load(f)

        mgr = parse_config(config_path, runner_cfg, env)
        mgr.run()
        return 0
    except Exception as e:
        logger.exception(e)
        return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Autoware in a predefined scenario, with tracing.')
    parser.add_argument('--config', '-c', metavar="CONFIG_PATH", type=str, default="config/aw_replay.yml",
                        help='Configuration file (.yml). Default: ./config/aw_replay.yml')

    parser.add_argument('env_vars', type=str, nargs='*', default=[],
                        help='Environment variables to pass to runners. In the format ENV_VAR_1:="value 1" ENV_VAR_2:=...')

    args = parser.parse_args()


    def parse_env_var(string):
        try:
            k, v = string.split(":=", maxsplit=1)
        except ValueError:
            raise ValueError(f"Environment variables have to be of format NAME:=value, got {string} instead!")
        if v.startswith('"'):
            v = v.strip('"')
        elif v.startswith("'"):
            v = v.strip("'")

        return k, v


    env = dict(map(parse_env_var, args.env_vars))
    if "ROS_DOMAIN_ID" not in env:
        raise ValueError("ROS domain ID is not set explicitly. "
                         "The domain ID MUST be set explicitly to not screw up other people's runs.")
    ret_status = run(args.config, env)
    sys.exit(ret_status)
