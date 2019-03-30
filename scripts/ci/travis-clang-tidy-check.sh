#!/usr/bin/env bash
make clang-tidy-all
                           
if [ -e "fixes.yaml" ]; then
  echo "clang-tidy warnings found"
  cat fixes.yaml
  exit 1
else
  echo "clang-tidy passed"
  exit 0
fi
