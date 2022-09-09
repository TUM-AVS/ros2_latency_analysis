#!/usr/bin/python3.10

import argparse
import functools
import os.path
import random
import subprocess
import sys
import time
import signal
from datetime import datetime, timedelta
from enum import Enum

import ruamel.yaml
import logging

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
    STARTUP = 1
    RUNNING = 2
    TEARDOWN = 3
    STOPPED = 4


class Scenario:
    def __init__(self, cfg_dict):
        self.name = cfg_dict["name"]
        self.desc = cfg_dict["desc"]
        self.end_condition = cfg_dict["end_condition"]
        self.rosbag = cfg_dict.get("rosbag")
        self.poses = cfg_dict.get("poses")

        logger.info(f"{self} loaded")

    def __str__(self):
        name = self.name
        desc = self.desc
        return f"Scenario({name=}, {desc=})"


class TaskManager:
    def __init__(self, tasks, start_deps):
        self.tasks = tasks
        self._state = RunnerState.STARTUP
        self.t_started = None

        self.start_order = []
        tasks_to_be_ordered = list(self.tasks.keys())
        while len(self.start_order) < len(tasks):
            can_start = [t for t in tasks_to_be_ordered if
                         t not in self.start_order and all(dep in self.start_order for dep in start_deps[t])]
            self.start_order += can_start

    def run(self, scenario_name):
        currently_starting = None
        logger.info(f"[TaskManager] STARTUP")
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
        logger.info(f"[TaskManager] RUNNING")
        while self._state == RunnerState.RUNNING:
            for task in self.tasks.values():
                task.poll()

            if datetime.now() - self.t_started > timedelta(seconds=30):
                self._state = RunnerState.TEARDOWN

        logger.info(f"[TaskManager] TEARDOWN")
        while self._state == RunnerState.TEARDOWN:
            for task in self.tasks.values():
                try:
                    task.poll()
                    task.stop()
                except:
                    pass

            time.sleep(.1)
            if all(t.state().value >= TaskState.STOPPED.value for t in self.tasks.values()):
                self._state = RunnerState.STOPPED

        logger.info(f"[TaskManager] STOPPED")
        runs_dir = f"runs/{scenario_name}"
        n_runs = len(os.listdir(runs_dir)) if os.path.isdir(runs_dir) else 0
        run_dir = f"runs/{scenario_name}/{n_runs + 1:03d}"
        os.makedirs(run_dir, exist_ok=True)
        for task in self.tasks.values():
            task.copy_artifacts(run_dir)
        logger.info(f"[TaskManager] DONE")


def parse_tasks(cfg_dict, env_vars):
    tasks = {}
    start_deps = {}

    for host in cfg_dict["hosts"]:
        if "connection" in host:
            connection = host["connection"]
        else:
            connection = None

        for task_name, task_dict in host["tasks"].items():
            if "start_deps" in task_dict:
                task_deps = task_dict["start_deps"]
                del task_dict["start_deps"]
            else:
                task_deps = []
            task = TASK_MAP[task_name](task_name, task_dict, connection, env_vars)
            start_deps[task_name] = task_deps
            tasks[task_name] = task

    return TaskManager(tasks, start_deps)


class Task:
    def __init__(self, name, cfg_dict, connection, env_vars):
        self.name = name
        self.commands = cfg_dict["start_commands"]
        self.commands.append("exit")
        self.connection = connection
        self.artifacts_loc = cfg_dict["artifact_location"] if "artifact_location" in cfg_dict else None
        logger.debug(f"{self}: connecting to shell...")

        if connection:
            shell_command = ["ssh", connection, "/bin/bash"]
        else:
            shell_command = ["/bin/bash"]

        self.shell = subprocess.Popen(shell_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                      bufsize=1,
                                      universal_newlines=True)
        os.set_blocking(self.shell.stdout.fileno(), False)
        logger.debug(f"{self}: shell connected")

        for var_name, value in env_vars.items():
            self.shell.stdin.write(f'export {var_name}="{value}"\n')

        logger.debug(f"{self}: exported environment vars")
        self._state = TaskState.INITIALIZED
        self._line_actions = []

    def copy_artifacts(self, target_dir):
        if not self.artifacts_loc:
            return 0

        task_dir = os.path.join(target_dir, self.name)
        os.makedirs(task_dir)

        if self.connection:
            logger.info(f"[{self}] Copying {self.connection}:{self.artifacts_loc} to {task_dir}")
            scp = subprocess.Popen(["scp", "-r", f"{self.connection}:{self.artifacts_loc}", task_dir])
            return scp.wait(60)

        logger.info(f"[{self}] Copying {self.artifacts_loc} to {task_dir}")
        cp = subprocess.Popen(["cp", "-r", self.artifacts_loc, task_dir])
        return cp.wait(60)

    def poll(self):
        try:
            new_lines = self.shell.stdout.readlines()
        except Exception:
            logger.error(f"{self}: pipe closed")
            return
        for line in new_lines:
            logger.info(f"[{self}] {line.rstrip()}")
        self._process_line_actions(new_lines)

    def _process_line_actions(self, new_lines):
        leftover_lines = new_lines

        if self._line_actions:
            action = self._line_actions[0]
            leftover_lines, done = action(leftover_lines)
            if done:
                self._line_actions = self._line_actions[1:]

    def state(self):
        shell_status = self.shell.poll()

        if shell_status is None:  # running
            pass
        elif shell_status == 0:  # exited (code 0)
            self._state = TaskState.STOPPED
        else:
            self._state = TaskState.CRASHED

        return self._state

    def _wait_for_line(self, line_content):
        def action(lines: list):
            while lines and (line := lines.pop(0)):
                if line_content in line:
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

    def _do_action(self, callback):
        def action(lines: list):
            print("action")
            callback()
            return [], True

        self._line_actions.append(action)

    def start(self):
        if self.state() != TaskState.INITIALIZED:
            return True
        self._state = TaskState.STARTING
        logger.info(f"{self}: starting...")
        self.shell.stdin.write('\n'.join(self.commands) + '\n')
        logger.info(f"{self}: started")

    def stop(self):
        if self._state.value >= TaskState.STOPPED.value:
            return
        self._state = TaskState.STOPPING
        logger.info(f"{self}: stopping")
        self.shell.send_signal(signal.SIGINT)

    def __str__(self):
        return f"Task(name={self.name})"


class AutowareTask(Task):
    def start(self):
        if super().start():
            return True
        self._wait_for_line("waiting for self pose...")
        self._wait_seconds(3)
        self._do_action(lambda: logger.info(f"{self}: ready"))
        self._do_action(lambda: setattr(self, "_state", TaskState.RUNNING))


class TracingTask(Task):
    def start(self):
        if super().start():
            return True
        self._wait_for_line("press enter to stop...")
        self._do_action(lambda: logger.info(f"{self}: ready"))
        self._do_action(lambda: setattr(self, "_state", TaskState.RUNNING))

    def stop(self):
        if self.state() != TaskState.RUNNING:
            return
        self.shell.stdin.write("\n")
        self._wait_for_line("stopping & destroying tracing session")
        self._do_action(super().stop)


class PerfTask(Task):
    def start(self):
        if super().start():
            return True
        self._wait_seconds(2)  # Let perf _actually_ start (it doesn't output anything)
        self._do_action(lambda: setattr(self, "_state", TaskState.RUNNING))


class RosbagTask(Task):
    def start(self):
        if super().start():
            return True
        self._do_action(lambda: setattr(self, "_state", TaskState.RUNNING))


class AwsimTask(Task):
    def start(self):
        if super().start():
            return True
        self._wait_seconds(15)  # Let AWSIM _actually_ start (it doesn't output anything)
        self._do_action(lambda: setattr(self, "_state", TaskState.RUNNING))


class MessagesTask(Task):
    def start(self):
        if super().start():
            return True
        self._state = TaskState.RUNNING


TASK_MAP = {
        "autoware": AutowareTask,
        "awsim": AwsimTask,
        "tracing": TracingTask,
        "perf": PerfTask,
        "rosbag": RosbagTask,
        "messages": MessagesTask
}


def run(scenario_path, config_path, env):
    yaml = ruamel.yaml.YAML()

    with open(os.path.join(scenario_path, "config.yml")) as f:
        scenario_cfg = yaml.load(f)
    scenario = Scenario(scenario_cfg)

    env["AWSIM_RUNNER_SCENARIO"] = os.path.basename(scenario_path)
    if scenario.rosbag:
        env["AWSIM_RUNNER_SCENARIO_ROSBAG_PATH"] = scenario.rosbag
    if scenario.poses:
        env["AWSIM_RUNNER_SCENARIO_POSES_PATH"] = scenario.poses

    for env_var, value in env.items():
        os.environ[env_var] = str(value)
        logger.debug(f"{env_var}={value}")

    with open(config_path) as f:
        runner_cfg = yaml.load(f)

    mgr = parse_tasks(runner_cfg, env)
    mgr.run(env["AWSIM_RUNNER_SCENARIO"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Autoware in a predefined scenario, with tracing.')
    parser.add_argument('scenario_name', metavar='SCENARIO_NAME', type=str,
                        help='Name of the scenario. Scenarios are found in ./scenarios/, scenario names are the names '
                             'of the subfolders.')
    parser.add_argument('--config', '-c', metavar="CONFIG_PATH", type=str, default="config/scenario_runner.yml",
                        help='Configuration file (.yml). Default: ./config/scenario_runner.yml')

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

    scenario_path = os.path.join("scenarios", args.scenario_name)
    env["ROS_DOMAIN_ID"] = 62
    run(scenario_path, args.config, env)
