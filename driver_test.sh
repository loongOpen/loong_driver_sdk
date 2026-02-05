#!/bin/bash
export LD_LIBRARY_PATH=.
if [ $# -eq 1 ]; then
    echo "driver_test_`uname -m` $1"
    ./driver_test_`uname -m` $1
elif [ $# -eq 2 ]; then
    echo "driver_test_`uname -m` $1 $2"
    ./driver_test_`uname -m` $1 $2
else
    echo "driver_test_`uname -m`"
    ./driver_test_`uname -m`
fi