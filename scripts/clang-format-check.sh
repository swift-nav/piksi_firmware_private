#!/usr/bin/env bash
make clang-format-all

if [[ `git status --porcelain` ]]; then
  echo "clang-format failed."
  exit 1
else
  echo "clang-format passed."
  exit 0
fi
