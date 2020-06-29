#!/bin/bash
set -e

#set vars
EXEC=/data/utils/build/utils_test_project

# RUN INPUT
if [ "$1" = 'start' ]; then
    $EXEC
else 
    exec "$@"
fi
