#!/usr/bin/env bash

set -euo pipefail

D=$( (cd "$(dirname "$0")" || exit 1 >/dev/null; pwd -P) )

VERSION_TAG=$(cat "$D/docker_version_tag")
DOCKER_REPO_NAME=swiftnav/clang-tidy
DOCKER_TAG=$DOCKER_REPO_NAME:$VERSION_TAG

if [[ -z "${NON_INTERACTIVE_BUILD:-}" ]]; then
  INTERACTIVE_ARGS=$(tty &>/dev/null && echo "--tty --interactive")
else
  INTERACTIVE_ARGS=
fi

ROOT=$( (cd "$(dirname "$0")/../.." || exit 1 >/dev/null; pwd -P) )

find_sysroot() {
  echo '#include <assert.h>' | \
    arm-none-eabi-gcc -M -xc - | \
    sed -e 's@ \\@@g' -e 's@^ /@/@' | \
    grep -v '^-:' | \
    head -1 | \
    xargs dirname
}

sysroot=$(find_sysroot)

docker run \
  -v "$PWD:/work" \
  -v "$sysroot:/sysroot" \
  ${INTERACTIVE_ARGS} \
  --rm \
  --name swiftnav-clang-tidy \
  "$DOCKER_TAG" "${@//$ROOT/\/work}" \
  -isystem/sysroot
