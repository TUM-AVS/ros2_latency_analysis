#!/bin/bash

cur_dir=$PWD
ros_project_dir="/home/max/Projects/autoware"

source_dir="$ros_project_dir/src"
build_dir="$ros_project_dir/build"

out_dir="$cur_dir/output"
mkdir -p $out_dir

node_tus=$(find $source_dir -name *node.cpp)

worker_task(){
  source_dir=$1
  build_dir=$2
  out_dir=$3
  node_tu=$4

  out_name=${node_tu//\//-}
  json_name=$(realpath --relative-to $source_dir $node_tu)
  json_name=${json_name//\//-}
  json_name=${json_name//'.cpp'/'.json'}

  i=$(($i + 1))
  echo "$i $out_name"

  tu_in_db=$(cat $build_dir/compile_commands.json | grep "$node_tu")
  if [ -z "$tu_in_db" ]
  then
    echo "File not found in compile DB: $(realpath --relative-to $source_dir $node_tu)"
    echo "null" > "$out_dir/$json_name"
  else
    ./dep-check --extra-arg=-w -p $build_dir $node_tu > /dev/null
    mv "./$out_name" "$out_dir/$json_name"
  fi
}

N=8
i=0
(
for node_tu in $node_tus ; do
   ((i=i%N)); ((i++==0)) && wait
   worker_task $source_dir $build_dir $out_dir $node_tu &
done
)
