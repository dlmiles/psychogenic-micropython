#!/bin/bash -e
#
# Run this locally:
#  cd ports/rp2/modules/i2cslave_alt
#  ./build_and_run_gtest.sh
#
# WARNING note after running this the created directories contain python items
#  which the micropyton build system will attempt to pickup and will error.
#  See the "rm -rf ....." items a few lines below
#
if [ -d build-gtest ]
then
    true
    # These dirs need to be removed to ensure
    #rm -rf build-gtest
    #rm -rf html-coverage
    #rm -rf venv
fi

mkdir -p build-gtest 2>/dev/null || true
cd build-gtest

# Debug build (helpful to: gdb ./build-gtest/test_i2cslave)
#cmake -Wno-dev -DCMAKE_BUILD_TYPE=Debug ..

# Debug build (helpful to: gdb ./build-gtest/test_i2cslave)
cmake -Wno-dev -DCMAKE_BUILD_TYPE=Coverage ..

# CMAKE_BUILD_ARGS="--verbose"
cmake --build . $CMAKE_BUILD_ARGS -- --no-print-directory


# There is an error/bug that pops up that cites: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=68080
# This seems to reduce incidence of issue
rm -f build-gtest/CMakeFiles/test_i2cslave.dir/*.gcda
rm -f build-gtest/CMakeFiles/test_i2cslave.dir/*.gcno
rm -f i2cslave_sock_seqpacket*.gcov

# Hmm search for this with the workaround, what reduces test iterations to get gcovr report out
export COVERAGE_GCOVR_PARSE_BUG="true"
set -o pipefail
./test_i2cslave 2>&1 | tee test_i2cslave.log
rc=$?
cd ..

basefile=build-gtest/CMakeFiles/test_i2cslave.dir/lib/i2cslave_sock_seqpacket.c
objfile="${basefile}.o"

if [ $rc -eq 0 ] && [ -f $objfile ]
then
    if which --skip-alias arm-none-eabi-size 2>/dev/null >/dev/null
    then
        size $objfile
        #arm-none-eabi-size $objfile
    fi
fi

if [ -f $objfile ]
then
    echo "### "
    echo "####### Coverage "
    echo "### "
    gcov $objfile

    test -d html-coverage || mkdir -p html-coverage
    #genhtml -o html-coverage "${basefile}.gcda"
fi

if [ -f "${basefile}.gcda" ]
then
    test -d venv || python3 -m venv venv
    source venv/bin/activate
    test -x venv/bin/gcovr || pip3 install gcovr

    test -d html-coverage || mkdir -p html-coverage
    if gcovr --html-nested -o html-coverage/index.html 2>GCOVR.stderr.tmp
    then
        echo "###### Coverage report at: html-coverage/index.html"
    else
        echo "###### WARNING coverage report generation failed with Python gcovr see GCOVR.stderr.tmp"
    fi
else
    echo "###### WARNING no *.gcda coverage data exists, no report produced"
fi

echo "##### Run test again manually with: ./build-gtest/test_i2cslave (maybe under gdb)"

echo "EXIT=$rc"
