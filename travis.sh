#!/usr/bin/env bash

set -e
set -o pipefail

if [ "$TESTENV" == "lint" ]; then
  ./scripts/ci/travis-clang-format-check.sh
  ./scripts/ci/travis-clang-tidy-check.sh
elif [ "$TESTENV" == "mesta" ]; then
  make -r -C mesta
  ./mesta/mesta
else
  export PATH=$PATH:~/gcc-arm-none-eabi/bin;
  make PIKSI_HW=v3 PIKSI_REV=prod -j2;
  make PIKSI_HW=v3 PIKSI_REV=base -j2;
  make run_tests
fi
