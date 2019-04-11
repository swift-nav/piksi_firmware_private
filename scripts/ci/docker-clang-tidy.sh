#!/usr/bin/env bash

set -euo pipefail

D=$( (cd "$(dirname "$0")" || exit 1 >/dev/null; pwd -P) )
ROOT=$( (cd "$(dirname "$0")/../.." || exit 1 >/dev/null; pwd -P) )

VERSION_TAG=$(cat "$D/docker_version_tag")
DOCKER_REPO_NAME=swiftnav/clang-tidy
DOCKER_TAG=$DOCKER_REPO_NAME:$VERSION_TAG

if [[ -z "${NON_INTERACTIVE_BUILD:-}" ]]; then
  INTERACTIVE_ARGS=$({ tty &>/dev/null && echo "--tty --interactive"; } || echo)
else
  INTERACTIVE_ARGS=
fi

find_sysroot() {
  echo '#include <assert.h>' | \
    arm-none-eabi-gcc -M -xc - | \
    sed -e 's@ \\@@g' -e 's@^ /@/@g' | \
    grep -v '^-:' | \
    head -1 | \
    xargs dirname
}

sysroot=$(find_sysroot)

docker rm -f swiftnav-clang-tidy &>/dev/null || :

docker run \
  -v "$PWD:/work" \
  -v "$sysroot:/sysroot" \
  -e CLANG_TIDY_INCLUDES="${CLANG_TIDY_INCLUDES//$ROOT/\/work}" \
  -e CLANG_TIDY_EXCLUDES="${CLANG_TIDY_EXCLUDES}" \
  -e CLANG_TIDY_FLAGS="$CLANG_TIDY_FLAGS -isystem/sysroot" \
  ${INTERACTIVE_ARGS} \
  --rm \
  --name swiftnav-clang-tidy \
  "$DOCKER_TAG" \
  ./scripts/ci/clang-tidy.sh \
  | sed -e "s@/work@$ROOT@g" -e "s@/sysroot@$sysroot@g"
