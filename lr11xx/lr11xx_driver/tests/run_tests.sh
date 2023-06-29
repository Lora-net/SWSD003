#!/bin/bash

# cleanup
cd ..
rm -f _tests/project.yml

# create project
ceedling new _tests
cp tests/project.yml _tests

# execute tests
cd _tests
ceedling clobber
ceedling gcov:all utils:gcov
