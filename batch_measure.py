#!/usr/bin/python3

import argparse
import datetime
import os
import subprocess
import sys
import logging
from typing import Dict, List

logging.basicConfig()

rootLogger = logging.getLogger()

fileHandler = logging.FileHandler("batch_measure.log")
rootLogger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
rootLogger.addHandler(consoleHandler)

LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(logging.INFO)

RUNS = {
    "edgar@edgar-hil-x86": [
        "./config/aw_awsim_taskset_00-05.yml",
        "./config/aw_awsim_taskset_00-09.yml",
        "./config/aw_awsim_taskset_00-11.yml",
        "./config/aw_awsim_taskset_00-13.yml",
    ]
}

def run(runs: Dict[str, List[str]], reps_per_run, retry_count):
    total_runs = reps_per_run * sum(map(len, RUNS.values()))

    t_start_total = datetime.datetime.now()
    LOGGER.info(f"Started at {t_start_total}")
    LOGGER.info(f"Performing a total of {total_runs} runs on {len(RUNS)} hosts.")
    LOGGER.info(f"Retrying failing runs up to {retry_count - 1} times.")
    total_complete = 0
    for host, configs in runs.items():
        try:
            user, hostname = host.split("@")
        except Exception:
            LOGGER.error(f"'{host}' is not in the form 'username@hostname'")
            sys.exit(1)
        
        for cfg in configs:
            for i in range(reps_per_run):
                success = False
                for j in range(retry_count):
                    progress_str = f"({total_complete+1} / {total_runs})"

                    if j > 0:
                        print_progress = LOGGER.warning
                        verb = f"Re-running (try {j+1}/{retry_count})"
                    else:
                        print_progress = LOGGER.info
                        verb = "Running"
                    print_progress(f"{progress_str} {verb} {cfg} on {user}@{hostname} ({i+1} / {reps_per_run})...")

                    t_start = datetime.datetime.now()
                    env = os.environ.copy()
                    env.update({"SC_AW_HOSTNAME": hostname, "SC_AW_USERNAME": user, "SC_CFG_PATH": cfg})
                    completed_process = subprocess.run(["./launch.bash"], env=env, shell=True)
                    runtime = datetime.datetime.now() - t_start
                    if completed_process.returncode == 0:
                        LOGGER.info(f"{progress_str} Done (finished cleanly), took {runtime.total_seconds():.2f} s.")
                        total_complete += 1
                        success = True
                        break

                    LOGGER.warning(f"{progress_str} Failed (returned code {completed_process.returncode}), took {runtime.total_seconds():.2f} s.")
                
                if not success:
                    LOGGER.error(f"{progress_str} Failed {retry_count} times, giving up on {cfg} on {user}@{hostname}.")

    t_end_total = datetime.datetime.now()
    LOGGER.info(f"Finished at {t_end_total}, ran for {t_end_total - t_start_total}.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Batch execution and orchestration of Autoware runs.')
    parser.add_argument('--reps-per-scenario', '-r', type=int, default=3,
                        help='Number of runs per configured runner scenario')
    parser.add_argument('--retry-count', type=int, default=3, 
                        help="Number of tries per repetition until a failing scenario is given up on.")

    args = parser.parse_args()

    run(RUNS, args.reps_per_run, args.retry_count)
