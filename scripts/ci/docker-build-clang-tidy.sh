#!/usr/bin/env bash

set -euo pipefail
IFS=$'\n\t'

D=$( (cd "$(dirname "$0")" || exit 1 >/dev/null; pwd -P) )

VERSION_TAG=$(cat "$D/docker_version_tag")
DOCKER_REPO_NAME=swiftnav/clang-tidy
DOCKER_USER=swiftnav

build_dir=$(mktemp -d)
trap 'rm -rfv $build_dir' EXIT

DOCKERFILE=Dockerfile.clang-tidy

cp -v "$D/$DOCKERFILE" "${build_dir}"
cd "${build_dir}"

echo '>>> Running docker build command...'

if [[ -z "${USE_CACHE:-}" ]];then
  docker build \
    --force-rm \
    --no-cache \
    -f $DOCKERFILE \
    -t "$DOCKER_REPO_NAME:$VERSION_TAG" \
    .
else
  docker build \
    -f $DOCKERFILE \
    -t "$DOCKER_REPO_NAME:$VERSION_TAG" \
    .
fi

echo '>>> Pushing build to Docker Hub...'

if [[ -n "${DOCKER_PASS:-}" ]]; then
  echo "$DOCKER_PASS" | docker login --username="${DOCKER_USER:-swiftnav}" --password-stdin
  docker push "$DOCKER_REPO_NAME:$VERSION_TAG"
else
  echo "WARNING: Not pushing new image to Docker Hub"
fi
