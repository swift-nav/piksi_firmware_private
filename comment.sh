#!/bin/bash

# Copyright (C) 2016 Swift Navigation Inc.
# Contact: Mark Fine <mark@swiftnav.com>
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

REPO="${PWD##*/}"
BUCKET="${BUCKET:-swiftnav-artifacts}"

BUILD_VERSION="$(git describe --tags --dirty --always)"
BUILD_PATH="$REPO/$BUILD_VERSION"

URL="https://api.github.com/repos/swift-nav/$REPO/issues/$PULL_REQUEST/comments"
COMMENT="## $BUILD_VERSION\n+ [s3://$BUCKET/$BUILD_PATH](https://console.aws.amazon.com/s3/home?region=us-west-2&bucket=swiftnav-artifacts-pull-requests&prefix=$BUILD_PATH/)\n+ [pull-requests/$BUILD_PATH](https://swiftnav-artifacts.herokuapp.com/pull-requests/$BUILD_PATH)\n+ [pull-requests/$BUILD_PATH/requirements.yaml](https://swiftnav-artifacts.herokuapp.com/pull-requests/$BUILD_PATH/requirements.yaml)"

echo "{\"body\":\"$COMMENT\"}"
curl -u "$GITHUB_TOKEN:" -X POST "$URL" -d "{\"body\":\"$COMMENT\"}"

