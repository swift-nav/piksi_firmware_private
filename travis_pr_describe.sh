#!/bin/bash

# Copyright (C) 2016 Swift Navigation Inc.
# Contact: Jonathan Diamond <jonathan@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
#
# Script for publishing built binaries to S3.

set -e

DESCRIPTION_FILE="build_v3_prod/pr_description.yaml"

if ! [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
  echo "---
  pr_number: $TRAVIS_PULL_REQUEST
  commit: $TRAVIS_PULL_REQUEST_SHA" >> $DESCRIPTION_FILE
  ./publish.sh $DESCRIPTION_FILE
fi
