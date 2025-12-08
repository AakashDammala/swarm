#!/bin/bash
#
# Script to run tests and generate code coverage report
#

set -xue -o pipefail

##############################
# 0. start from scratch
##############################
rm -rf build/ install/
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
set -u                          # re-enable undefined variable check

##############################
# 1. Build for test coverage
##############################
colcon build --cmake-args -DCOVERAGE=1
set +u                          # stop checking undefined variable  
source install/setup.bash
set -u                          # re-enable undefined variable check

##############################
# 2. run all tests
##############################
# Launch simulation in background for integration tests
echo "Starting simulation..."
# ros2 launch swarm_view swarm_view.launch.py &
SIM_PID=$!
sleep 15 # Wait for Webots to initialize

colcon test

# Kill simulation
echo "Stopping simulation..."
# kill $SIM_PID || true

##############################
# 3. get return status
##############################
colcon test-result --verbose

##############################
# 4. generate coverage reports
##############################
## create output directory
COMBINED_TEST_COVERAGE=combined_test_coverage
if [[ -d $COMBINED_TEST_COVERAGE ]]; then
   rm -rf $COMBINED_TEST_COVERAGE
fi
mkdir $COMBINED_TEST_COVERAGE

## generate coverage info
lcov --capture --directory build --output-file build/test_coverage_merged.info
lcov --remove build/test_coverage_merged.info \
    '/usr/*' \
    '/opt/*' \
    '*/test/*' \
    '*/build/*' \
    '*/install/*' \
    --output-file build/test_coverage_merged.info

## generate html report
genhtml --output-dir $COMBINED_TEST_COVERAGE build/test_coverage_merged.info

##############################
# 5. show the combined coverage report
##############################
# open $COMBINED_TEST_COVERAGE/index.html || true
