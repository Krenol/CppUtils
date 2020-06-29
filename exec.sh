#!/bin/bash

mkdir build


export BUILD_TEST=ON

cmake -S utils/ -B build/

cmake --build ./build || exit 1


./utils/build/utils_test_project