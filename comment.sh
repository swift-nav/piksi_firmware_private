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

if [ "$TRAVIS_OS_NAME" != "linux" ]; then
    exit
fi

REPO="${PWD##*/}"
BUCKET="swiftnav-artifacts-pull-requests"
BUILD_SOURCE="pull-request"
BUILD_VERSION="$(git describe --tags --dirty --always)"
BUILD_PATH="$REPO/$BUILD_VERSION"
ARTIFACTS_PATH="pull-requests/$BUILD_PATH"

RELEASES=""
SCENARIOS="live-roof-1543-mission%2Clive-roof-1800"

HITL_PASS_RUNS=10
STATUS_HITL_CONTEXT="hitl/pass-fail"

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    BUCKET="swiftnav-artifacts"
    ARTIFACTS_PATH="$BUILD_PATH"
    BUILD_SOURCE="release"
fi

LINKS=\
("http://hitl-dashboard.swiftnav.com/hitl?source=$BUILD_SOURCE&build=$BUILD_VERSION&runs=$HITL_PASS_RUNS"
"http://sbp-log-analysis.swiftnav.com/#/d/0/q/x/firmware/y/metric/f/metric/p/passfail/f/scenario/sv/$SCENARIOS/f/firmware/sv/$RELEASES%2C$BUILD_VERSION"
"http://sbp-log-analysis.swiftnav.com/#/d/0/q/x/firmware/y/metric/f/metric/p/piksi-multi-PRD/f/scenario/sv/$SCENARIOS/f/firmware/sv/$RELEASES%2C$BUILD_VERSION"
"https://github.com/swift-nav/piksi_firmware_private/commits/$BUILD_VERSION"
"http://hitl-dashboard.swiftnav.com/files/$BUCKET/$REPO/$BUILD_VERSION/"
)

TITLES=\
("Run a HITL test set for this build"
"HITL Results - pass/fail checks"
"HITL Results - performance metrics"
"Commit Log"
"Firmware Artifacts"
)

slack_links(){
    echo -n $BUILD_PATH
    for index in ${!LINKS[@]}; do
        echo -ne "\n"${LINKS[$index]}
    done
}

github_links(){
    echo -n "## $BUILD_VERSION"
    echo -n "\nYou must manually submit the form in the link for 'Run a HITL test set for this build' in order to get data for the 'HITL Results' links."
    echo -n "\n$HITL_PASS_RUNS runs must pass for the $STATUS_HITL_CONTEXT status to be marked successful"
    echo -n "\nAny HITL failures require a new commit to start a new test set"
    echo -n "\nNote: the following links are for this Pull Request's ***merge*** commit"
    for index in ${!LINKS[@]}; do
        echo -n "\n+ "[${TITLES[$index]}]"("${LINKS[$index]}")"
    done
}

echo "Comment PULL_REQUEST ($TRAVIS_PULL_REQUEST)"
echo "Comment BRANCH ($TRAVIS_BRANCH)"
echo "Comment TAG ($TRAVIS_TAG)"

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    if [[ "$TRAVIS_BRANCH" == master || "$TRAVIS_TAG" == v* || "$TRAVIS_BRANCH" == v*-release ]]; then
        COMMENT="$(slack_links)"
        URL="https://slack.com/api/chat.postMessage?token=$SLACK_TOKEN&channel=$SLACK_CHANNEL"
        DATA="text=$COMMENT"
        curl --data-urlencode "$DATA" "$URL"
    fi
elif [ ! -z "$GITHUB_TOKEN" ]; then
    COMMENT="$(github_links)"
    COMMENT_URL="https://api.github.com/repos/swift-nav/$REPO/issues/$TRAVIS_PULL_REQUEST/comments"
    # set status of HITL testing to pending
    STATUS_URL="https://api.github.com/repos/swift-nav/piksi_firmware_private/statuses/$TRAVIS_PULL_REQUEST_SHA"
    STATUS_DESCRIPTION="Waiting for HITL tests to be run and complete"
    STATUS_TARGET_URL="http://sbp-log-analysis.swiftnav.com/#/d/0/q/x/firmware/y/metric/f/metric/p/passfail/f/scenario/sv/$SCENARIOS/f/firmware/sv/$RELEASES%2C$BUILD_VERSION"
    STATUS_STATE="pending"
    curl -i -X POST -u "$GITHUB_TOKEN:" $STATUS_URL -d "{\"state\": \"$STATUS_STATE\",\"target_url\": \"$STATUS_TARGET_URL\", \"description\": \"$STATUS_DESCRIPTION\", \"context\": \"$STATUS_HITL_CONTEXT\"}"
fi

