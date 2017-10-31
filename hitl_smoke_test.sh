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
# Script for kicking off HITL smoke tests on piksi_firmware_private Pull
# Requests. Expects HITL_API_GITHUB_TOKEN and GITHUB_COMMENT_TOKEN to be in
# environmental variables.

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
#HITL_API_URL="https://hitlapi.swiftnav.com"
HITL_API_URL="https://hitlapi.thisshouldfail.com"
# From https://github.com/travis-ci/travis-ci/issues/8557, it is not trivial to
# get the name / email of the person who made the PR, so we'll use the email of
# the commit instead.
TESTER_EMAIL="$(git log --format='%ae' HEAD | head -n 1)"

BUILD_TYPE="pull_request"
BUILD_VERSION="$(git describe --tags --dirty --always)"

REPO="${PWD##*/}"
COMMENT_URL="https://api.github.com/repos/swift-nav/$REPO/issues/$TRAVIS_PULL_REQUEST/comments"
COMMENT_HEADER="## HITL smoke tests: $BUILD_VERSION"

TRAVIS_BUILD_URL="https://travis-ci.com/swift-nav/piksi_firmware_private/builds/$TRAVIS_BUILD_ID"

# HITL scenarios to kick off, and # of runs for each scenario.
SCENARIOS=\
("live-roof-1543-mission"
"live-roof-1543-mission-dropouts-zero-baseline"
"live-roof-1543-mission-skylark-glonass"
)
RUNS=\
("1"
"1"
"1"
)

# Capture IDs of kicked off jobs.
capture_ids=()

# Kick off jobs, record capture ID of each job. If a job fails to be kicked
# off, post a comment to the Pull Request and fail Travis.
set +e
for index in ${!SCENARIOS[@]}; do
    URL="$HITL_API_URL/jobs?&build_type=$BUILD_TYPE&build=$BUILD_VERSION&tester_email=$TESTER_EMAIL&runs=${RUNS[$index]}&scenario_name=${SCENARIOS[$index]}"
    echo "Posting to HITL API URL: \"$URL\""
    capture_ids+=($(curl -u $HITL_API_GITHUB_USER:$HITL_API_GITHUB_TOKEN -X POST $URL | python -c "import sys, json; print json.load(sys.stdin)['id']"))
    if [ ! $? -eq 0 ]; then
        curl -u "$GITHUB_COMMENT_TOKEN:" -X POST "$COMMENT_URL" -d "{\"body\":\"$COMMENT_HEADER\nThere was an error using the HITL API to kick off smoke tests for this commit. See $TRAVIS_BUILD_URL.\"}"
        echo "There was an error using the HITL API. Posted comment to GitHub PR, exiting."
        exit 1
    fi
done
set -e

# Comment on the PR with links to the hitl-dashboard and gnss-analysis.
hitl_links(){
    echo -n "$COMMENT_HEADER"
    echo -n "\nThese tests are kicked off whenever you push a new commit to this PR."
    echo -n "\n### capture jobs status"
    for index in ${!capture_ids[@]}; do
        echo -n "\n+ "[${SCENARIOS[$index]} runs]"("http://hitl-dashboard.swiftnav.com/job/${capture_ids[$index]}")"
    done
    echo -n "\n### gnss-analysis results"
    echo -n "\n+ "[passfail]"(""https://gnss-analysis.swiftnav.com/summary_type=q50&metrics_preset=pass_fail&scenario="
    for index in ${!SCENARIOS[@]}; do
      echo -n "${SCENARIOS[$index]},"
    done
    echo -n "&build_type=pr&firmware_versions=$BUILD_VERSION&groupby_key=firmware&display_type=table)"
    echo -n "\n+ "[detailed]"(""https://gnss-analysis.swiftnav.com/summary_type=q50&metrics_preset=detailed&scenario="
    for index in ${!SCENARIOS[@]}; do
      echo -n "${SCENARIOS[$index]},"
    done
    echo -n "&build_type=pr&firmware_versions=$BUILD_VERSION&groupby_key=firmware&display_type=table)"
}
COMMENT="$(hitl_links)"
echo "PR comment:"
echo -e "$COMMENT"
curl -u "$GITHUB_COMMENT_TOKEN:" -X POST "$COMMENT_URL" -d "{\"body\":\"$COMMENT\"}"

