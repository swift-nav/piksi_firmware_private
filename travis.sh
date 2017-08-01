#!/usr/bin/env bash

if [ "$TESTENV" == "lint" ]; then
  ./scripts/ci/travis-clang-format-check.sh
else
  export PATH=$PATH:~/gcc-arm-none-eabi/bin;
  make PIKSI_HW=v3 PIKSI_REV=prod;
fi
