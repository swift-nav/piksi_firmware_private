#!/bin/bash

# Copyright (C) 2017 Swift Navigation Inc.
# Contact: Swift Navigation <dev@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
#
# Script for kicking off HITL smoke tests on piksi_firmware_private Pull Requests.

set -e

if [ "$TRAVIS_OS_NAME" != "linux" ]; then
    exit
fi

if [ "$TESTENV" == "lint" ]; then
    exit
fi

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    exit
fi

HITL_API_GITHUB_USER="swiftnav-travis"
HITL_API_URL="http://hitlapi-hitlapi-v3v5zi4-1951594375.us-west-2.elb.amazonaws.com"
# From https://github.com/travis-ci/travis-ci/issues/8557, it is not trivial to
# get the name / email of the person who made the PR, so we'll use the email of
# the commit instead.
TESTER_EMAIL="$(git log --format='%ae' HEAD | head -n 1)"

BUILD_TYPE="pull_request"
BUILD_VERSION="$(git describe --tags --dirty --always)"

# Kick off HITL smoke tests:
#   1x `live-roof-1543-mission`
#   1x `live-roof-1543-mission-dropouts-zero-baseline`
#   1x `live-roof-1543-mission-skylark-glonass`
curl -u $HITL_API_GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission"
curl -u $HITL_API_GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission-dropouts-zero-baseline"
curl -u $HITL_API_GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission-skylark-glonass"

