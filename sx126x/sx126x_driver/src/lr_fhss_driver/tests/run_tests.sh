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

echo "Global coverage:" $(echo "cat //html/body/table[1]/tr[3]/td/table/tr[2]/td[7]" | xmllint --html --shell build/artifacts/gcov/GcovCoverageResults.html | grep 'td class' | cut -d'>' -f2 | cut -d'%' -f1)"%"
