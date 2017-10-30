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

GITHUB_USER="cbeighley"
HITL_API_GITHUB_TOKEN="01d15035bd5e9100385c2f845533a6c593c60879"
HITL_API_URL="http://hitlapi-hitlapi-v3v5zi4-1951594375.us-west-2.elb.amazonaws.com"
TESTER_EMAIL="colin@swiftnav.com"

#BUILD_TYPE="pull_request"
#BUILD_VERSION="$(git describe --tags --dirty --always)"
BUILD_TYPE="release_candidate"
BUILD_VERSION="v1.2.12"

# Kick off HITL smoke tests:
#   1x `live-roof-1543-mission`
#   1x `live-roof-1543-mission-dropouts-zero-baseline`
#   1x `live-roof-1543-mission-skylark-glonass`
curl -u $GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission"
curl -u $GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission-dropouts-zero-baseline"
curl -u $GITHUB_USER:$HITL_API_GITHUB_TOKEN -v -X POST "$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=1&scenario_name=live-roof-1543-mission-skylark-glonass"

