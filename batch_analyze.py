#!/usr/bin/python3
import datetime
import glob
import logging
import os
import argparse
import shutil

import papermill as pm

logging.basicConfig()

rootLogger = logging.getLogger()

fileHandler = logging.FileHandler("batch_analyze.log")
rootLogger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
rootLogger.addHandler(consoleHandler)

LOGGER = logging.getLogger(__name__)
LOGGER.setLevel(logging.INFO)


def main(base_dir, name_filter):
    while True:
        artifacts = set(glob.glob(os.path.join(base_dir, name_filter)))
        unprocessable = {a for a in artifacts if os.path.isfile(os.path.join(a, "cannot_process"))}
        unprocessed = {a for a in artifacts if not os.path.isfile(os.path.join(a, "output", "plot_e2es_violin_labels.csv"))}

        unprocessed -= unprocessable
        if not unprocessed:
            break

        print(f"Found {len(unprocessed)} unprocessed and {len(unprocessable)} unprocessable artifacts.")

        current_artifact = unprocessed.pop()
        print(f"Now working on {current_artifact}.")

        out_dir = os.path.join(current_artifact, 'output')
        shutil.rmtree(out_dir, ignore_errors=True)
        os.makedirs(out_dir, exist_ok=False)

        os.environ["ANA_NB_OUT_PATH"] = f"'{out_dir}'"
        os.environ["ANA_NB_TR_PATH"] = f"'{os.path.join(current_artifact, 'tracing/max-ma-trace/ust')}'"

        try:
            pm.execute_notebook(
                "./trace-analysis.ipynb",
                os.path.join(current_artifact, "output", "trace-analysis.ipynb"),
                log_output=True
            )
        except Exception as e:
            LOGGER.exception(e)

        if not os.path.isfile(os.path.join(current_artifact, "output", "plot_e2es_violin_labels.csv")):
            with open(os.path.join(current_artifact, "cannot_process"), "w"):
                pass

    print("All artifacts processed.")


if __name__ == "__main__":
    LOGGER.info(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    parser = argparse.ArgumentParser()

    parser.add_argument('--base-directory', '-d', default=os.path.expandvars('$HOME/Projects/ma-measurements'),
                        help='The base directory containing all artifacts directories to be processed')

    parser.add_argument('--name-filter', '-f', default="artifacts_*", help="A shell-style wildcard expression to filter artifact folder names within the base directory. E.g. 'artifacts_2023*'.")

    args = parser.parse_args()

    print(f'Batch analyzing {args.base_directory}/{args.name_filter}')
    main(args.base_directory, args.name_filter)

