#!/bin/bash
# if empty will use current dir
if [ $# -ge 1 ]; then
	STAGE_DIR=$1
else
	STAGE_DIR=stage
fi

if [ $# -ge 2 ]; then
	bundle_name=$2
else
	bundle_name=../ardupilot.tgz
fi

cd $STAGE_DIR && tar -czvf $bundle_name . && cd - && echo Created $bundle_name

