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

if [ "$TESTENV" == "lint" ]; then
    exit
fi

if [ "$TESTENV" == "mesta" ]; then
    exit
fi

REPO="${PWD##*/}"
BUCKET="swiftnav-artifacts-pull-requests"
BUILD_SOURCE="pull-request"
BUILD_VERSION="$(git describe --tags --dirty --always)"
BUILD_PATH="$REPO/$BUILD_VERSION"

SCENARIO="live-roof-650-townsend"

STATUS_HITL_CONTEXT="hitl/pass-fail"

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    BUCKET="swiftnav-artifacts"
    BUILD_SOURCE="release"
fi

if [ "$BUILD_SOURCE" == "pull-request" ]; then
    HITL_BUILD_SOURCE="pr"
else
    HITL_BUILD_SOURCE="master"
fi

hitl_viewer_link() {
    local PAGE=$1
    local ARGS=$2
    echo "https://gnss-analysis.swiftnav.com$PAGE/$ARGS"
}
hitl_metrics_link() {
    local PRESET=$1
    echo $(hitl_viewer_link "" "metrics_preset=$PRESET&scenario=$SCENARIO&build_type=$HITL_BUILD_SOURCE&firmware_versions=$BUILD_VERSION")
}
hitl_pass_fail_link () {
    echo $(hitl_metrics_link "pass_fail")
}
hitl_high_level_link () {
    echo $(hitl_metrics_link "detailed")
}
hitl_artifacts_link () {
    echo $(hitl_viewer_link "/artifacts" "build_type=$HITL_BUILD_SOURCE&name_filter=$BUILD_VERSION")
}
hitl_test_runs_link () {
    echo $(hitl_viewer_link "/test_runs" "scenario=$SCENARIO&build_type=$HITL_BUILD_SOURCE&firmware_versions=$BUILD_VERSION")
}

LINKS=\
("https://gnss-analysis.swiftnav.com/schedule/firmware=$BUILD_VERSION&build_type=$HITL_BUILD_SOURCE&num_runs=10"
$(hitl_pass_fail_link)
$(hitl_high_level_link)
$(hitl_test_runs_link)
"https://github.com/swift-nav/piksi_firmware_private/commits/$BUILD_VERSION"
$(hitl_artifacts_link)
)

TITLES=\
("Run a HITL test set for this build"
"HITL Results - aggregated pass/fail checks (click on < 100 pass rates for failing runs)"
"HITL Results - aggregated performance metrics"
"HITL Runs - performance metrics for individual runs"
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
    echo -n "\nNote:"
    echo -n "\n- Check the status of HITL runs through the [hitl-viewer](https://gnss-analysis.swiftnav.com/jobs)."
    echo -n "\n"
    echo -n "\nThe following links are for this Pull Request's ***merge*** commit:"
    for index in ${!LINKS[@]}; do
        echo -n "\n+ "[${TITLES[$index]}]"("${LINKS[$index]}")"
    done
}

echo "Comment PULL_REQUEST ($TRAVIS_PULL_REQUEST)"
echo "Comment BRANCH ($TRAVIS_BRANCH)"
echo "Comment TAG ($TRAVIS_TAG)"

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    if [[ "$TRAVIS_BRANCH" == master || "$TRAVIS_TAG" == v* || "$TRAVIS_BRANCH" == v*-release || "$TRAVIS_BRANCH" == v*-dev ]]; then
        COMMENT="$(slack_links)"
        URL="https://slack.com/api/chat.postMessage?token=$SLACK_TOKEN&channel=$SLACK_CHANNEL"
        DATA="text=$COMMENT"
        curl --data-urlencode "$DATA" "$URL"
    fi
elif [ ! -z "$GITHUB_COMMENT_TOKEN" ]; then
    COMMENT="$(github_links)"
    COMMENT_URL="https://api.github.com/repos/swift-nav/$REPO/issues/$TRAVIS_PULL_REQUEST/comments"
    curl -u "$GITHUB_COMMENT_TOKEN:" -X POST "$COMMENT_URL" -d "{\"body\":\"$COMMENT\"}"
    # set status of HITL testing to pending
    STATUS_URL="https://api.github.com/repos/swift-nav/piksi_firmware_private/statuses/$TRAVIS_PULL_REQUEST_SHA"
    STATUS_DESCRIPTION="Waiting for HITL tests to be run and complete"
    STATUS_TARGET_URL=$(hitl_pass_fail_link)
    STATUS_STATE="pending"
    curl -i -X POST -u "$GITHUB_COMMENT_TOKEN:" $STATUS_URL -d "{\"state\": \"$STATUS_STATE\",\"target_url\": \"$STATUS_TARGET_URL\", \"description\": \"$STATUS_DESCRIPTION\", \"context\": \"$STATUS_HITL_CONTEXT\"}"
fi

