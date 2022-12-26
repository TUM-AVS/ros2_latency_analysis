#!/usr/bin/python3

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
        "./config/aw_awsim_dds_connext.yml",
        "./config/aw_awsim_dds_gurum.yml",
    ]
}

def run(runs: Dict[str, List[str]], reps_per_run=3):
    total_runs = reps_per_run * sum(map(len, RUNS.values()))

    t_start_total = datetime.datetime.now()
    LOGGER.info(f"Started at {t_start_total}")
    LOGGER.info(f"Performing a total of {total_runs} runs on {len(RUNS)} hosts.")
    total_complete = 0
    for host, configs in runs.items():
        try:
            user, hostname = host.split("@")
        except Exception:
            LOGGER.error(f"'{host}' is not in the form 'username@hostname'")
            sys.exit(1)
        
        for cfg in configs:
            for i in range(reps_per_run):
                progress_str = f"({total_complete+1} / {total_runs})"
                LOGGER.info(f"{progress_str} Running {cfg} on {user}@{hostname} ({i+1} / {reps_per_run})...")
                t_start = datetime.datetime.now()
                env = os.environ.copy()
                env.update({"SC_AW_HOSTNAME": hostname, "SC_AW_USERNAME": user, "SC_CFG_PATH": cfg})
                completed_process = subprocess.run(["./launch.bash"], env=env, shell=True)
                runtime = datetime.datetime.now() - t_start
                if completed_process.returncode == 0:
                    LOGGER.info(f"{progress_str} Done (finished cleanly), took {runtime.total_seconds():.2f} s.")
                else:
                    LOGGER.warning(f"{progress_str} Failed (returned code {completed_process.returncode}), took {runtime.total_seconds():.2f} s.")
                total_complete += 1
    t_end_total = datetime.datetime.now()
    LOGGER.info(f"Finished at {t_end_total}, ran for {t_end_total - t_start_total}.")

if __name__ == "__main__":
    run(RUNS)
