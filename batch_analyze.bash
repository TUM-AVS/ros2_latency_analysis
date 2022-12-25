#!/bin/bash

base="$HOME/Projects/ma-measurements"

date -u -Iseconds | tee -a batch_analyze.log

for file in "$base"/artifacts*
do
  echo "=== Working on $file" | tee -a batch_analyze.log
  out="$file/output"
  mkdir -p "$out"
  export ANA_NB_OUT_PATH="'$out'"
  export ANA_NB_TR_PATH="'$file/tracing/max-ma-trace/ust'"
  papermill ./trace-analysis.ipynb "$out"/trace-analysis.ipynb | tee -a batch_analyze.log
done

echo "Done." | tee -a batch_analyze.log
