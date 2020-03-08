#!/bin/bash
# if empty will use current dir
if [ $# -ge 1 ]; then
	bundle_name=$1
else
	bundle_name=../ardupilot.tgz
fi

cd stage && tar -czvf $bundle_name . && cd ..
