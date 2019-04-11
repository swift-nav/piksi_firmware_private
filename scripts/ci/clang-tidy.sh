#!/usr/bin/env bash	

files=$(git ls-files -- 'src/*.[ch]' | grep -E -v "$CLANG_TIDY_EXCLUDES")

clang-tidy-6.0 -export-fixes=fixes.yaml $files -- \
  ${CLANG_TIDY_FLAGS} ${CLANG_TIDY_INCLUDES}
