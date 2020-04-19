#!/bin/bash
# if empty will use current dir
if [ $# -ge 1 ]; then
	bundle_name=$1
else
	bundle_name=../ardupilot.tgz
fi

if [ $# -ge 2 ]; then
	STAGE_DIR=$2
else
	STAGE_DIR=stage
fi

cd $STAGE_DIR && tar -czvf $bundle_name . && cd -
