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

BUILD_VERSION="$(git describe --tags --dirty --always)"
BUILD_PATH="$REPO/$BUILD_VERSION"
ARTIFACTS_PATH="pull-requests/$BUILD_PATH"

RELEASES="piksi_firmware_v0.21.hex"

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    BUCKET="swiftnav-artifacts"
    ARTIFACTS_PATH="$BUILD_PATH"
fi

LINKS=\
("http://sbp-log-analysis.swiftnav.com/#/d/0/q/x/firmware/y/metric/f/metric/p/passfail/f/firmware/sv/$RELEASES%2C$BUILD_VERSION"
"http://sbp-log-analysis.swiftnav.com/#/d/0/q/x/firmware/y/metric/f/metric/p/piksi-multi-PRD/f/firmware/sv/$RELEASES%2C$BUILD_VERSION"
"https://github.com/swift-nav/piksi_firmware_private/commits/$BUILD_VERSION"
"https://console.aws.amazon.com/s3/home?region=us-west-2&bucket=$BUCKET&prefix=$BUILD_PATH/"
"https://swiftnav-artifacts.herokuapp.com/$ARTIFACTS_PATH/requirements.yaml")

TITLES=\
("HITL Results - $BUILD_PATH - pass/fail checks"
"HITL Results - $BUILD_PATH - performance metrics"
"Commit Log - $BUILD_PATH"
"Firmware Artifacts - $BUILD_PATH"
"Piksi Multi Artifacts - $BUILD_PATH")

slack_links(){
    echo -n $BUILD_PATH
    for index in ${!LINKS[@]}; do
        echo -ne "\n"${LINKS[$index]}
    done
}

github_links(){
    echo -n "## $BUILD_VERSION"
    for index in ${!LINKS[@]}; do
        echo -n "\n+ "[${TITLES[$index]}]"("${LINKS[$index]}")"
    done
}

if [ "$TRAVIS_PULL_REQUEST" == "false" ]; then
    if [ "$TRAVIS_BRANCH" == "master" ]; then
        COMMENT="$(slack_links)"
        URL="https://slack.com/api/chat.postMessage?token=$SLACK_TOKEN&channel=$SLACK_CHANNEL"
        DATA="text=$COMMENT"
        curl --data-urlencode "$DATA" "$URL"
    fi
elif [ ! -z "$GITHUB_TOKEN" ]; then
    COMMENT="Note: the following links are for this Pull Request's ***merge*** commit$(github_links)"
    URL="https://api.github.com/repos/swift-nav/$REPO/issues/$TRAVIS_PULL_REQUEST/comments"
    curl -u "$GITHUB_TOKEN:" -X POST "$URL" -d "{\"body\":\"$COMMENT\"}"
fi

