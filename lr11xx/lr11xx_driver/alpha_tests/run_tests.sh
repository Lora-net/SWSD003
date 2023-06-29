#!/bin/bash

# cleanup
cd ..
rm -f _alpha_tests/project.yml

# create project
ceedling new _alpha_tests
cp alpha_tests/project.yml _alpha_tests

# execute tests
cd _alpha_tests
ceedling clobber
ceedling gcov:all utils:gcov
