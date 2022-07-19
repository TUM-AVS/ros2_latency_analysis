#!/bin/bash

cur_dir=$PWD
ros_project_dir="/home/max/Projects/autoware"

source_dir="$ros_project_dir/src"
build_dir="$ros_project_dir/build"

out_dir="$cur_dir/output"
mkdir -p $out_dir

node_tus=$(find $source_dir -name *node.cpp)
total_tus=$(find $source_dir -name *node.cpp | wc -l)

worker_task() {
  my_j=$1
  source_dir=$2
  build_dir=$3
  out_dir=$4
  node_tu=$5

  out_name=${node_tu//\//-}
  rel_name=$(realpath --relative-to $source_dir $node_tu)
  json_name=${rel_name//\//-}
  json_name=${json_name//'.cpp'/'.json'}

  echo "$my_j/$total_tus $rel_name"

  tu_in_db=$(cat $build_dir/compile_commands.json | grep "$node_tu")
  if [ -z "$tu_in_db" ]; then
    echo -e "  \e[31mFile not found in compile DB: $(realpath --relative-to $source_dir $node_tu)\e[0m"
    echo "null" >"$out_dir/$json_name"
  else
    ./dep-check --extra-arg=-w -p $build_dir $node_tu >/dev/null
    mv "./$out_name" "$out_dir/$json_name"
  fi
}

N=8
i=0
j=0
(
  for node_tu in $node_tus; do
    ((j++))
    ((i = i % N))
    ((i++ == 0)) && wait
    worker_task $j $source_dir $build_dir $out_dir $node_tu &
  done
)

echo "Done."