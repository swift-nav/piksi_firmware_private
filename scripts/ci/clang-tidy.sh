#!/usr/bin/env bash

set -euo pipefail

[[ -z "${DEBUG:-}" ]] || set -x

usage() {
cat << EOF
usage: $(basename "$0") [<OPTIONS>]
Run clang-tidy with configured options

GENERAL OPTIONS:
   -h, --help            Show this message.
   --gcc <gcc-path>      Configure the gcc clang-tidy will use, sysroot
                         headers will be discovered from here.

MODE SELECTION OPTIONS:
   --run                 Run clang-tidy.
   --docker-run          Run dockerized clang-tidy.
   --find-sysroot        Find and print sysroot that clang-tidy will use,
                         do not run clang-tidy.

OPTIONS TO CONFIGURE CLANG-TIDY:
   --includes <includes> Configure clang-tidy includes.
   --excludes <excludes> Configure clang-tidy excludes.
   --options  <options>  General options passed to clang-tidy.

OPTIONS TO CONFIGURE DOCKER:
   --no-tty              Don't attempt to allocated a tty.
EOF
}

### Setup ###########################################################

D=$( (cd "$(dirname "$0")" || exit 1 >/dev/null; pwd -P) )
ROOT=$( (cd "$(dirname "$0")/../.." || exit 1 >/dev/null; pwd -P) )

if ! [[ -d $ROOT/.git ]]; then
  echo "ERROR: script appears to have been moved, or is not in a git repo" >&2
  exit 1
fi

### Parse options ####################################################

# Configuration
CLANG_TIDY_INCLUDES=${CLANG_TIDY_INCLUDES:-}
CLANG_TIDY_EXCLUDES=${CLANG_TIDY_EXCLUDES:-}
CLANG_TIDY_FLAGS=${CLANG_TIDY_FLAGS:-}
CLANG_TIDY_GCC=${CLANG_TIDY_GCC:-}
CLANG_TIDY_SYSROOT=${CLANG_TIDY_SYSROOT:-}
NON_INTERACTIVE_BUILD=${NON_INTERACTIVE_BUILD:-}

# Run modes
CLANG_TIDY_RUN=
CLANG_TIDY_DOCKER_RUN=
CLANG_TIDY_FIND_SYSROOT=

while true; do
  case "${1-}" in

    -h|--help)      usage; exit 0;;

    --includes)     CLANG_TIDY_INCLUDES="$2";  shift 2;;
    --excludes)     CLANG_TIDY_EXCLUDES="$2";  shift 2;;
    --options)      CLANG_TIDY_FLAGS="$2";     shift 2;;
    --gcc)          CLANG_TIDY_GCC="$2";       shift 2;;
    --run)          CLANG_TIDY_RUN=y;          shift 1;;
    --docker-run)   CLANG_TIDY_DOCKER_RUN=y;   shift 1;;
    --find-sysroot) CLANG_TIDY_FIND_SYSROOT=y; shift 1;;
    --no-tty)       NON_INTERACTIVE_BUILD=y;   shift 1;;

    *) break;;
  esac
done

### Find sysroot ####################################################

find_sysroot() {
  if command -v "$CLANG_TIDY_GCC" &>/dev/null; then
    echo '#include <assert.h>' \
      | $CLANG_TIDY_GCC -M -xc - \
      | grep -v '^-:' \
      | sed -e 's@ \\@@g' -e 's@^ /@/@g' \
      | head -1 \
      | xargs dirname
  fi
}

### Ensure sysroot ###################################################

ensure_sysroot() {

  if [[ -z "$CLANG_TIDY_SYSROOT" ]]; then
    CLANG_TIDY_SYSROOT=$(find_sysroot)
    if [[ -z "$CLANG_TIDY_SYSROOT" ]]; then
      echo "ERROR: failed to find sysroot for $CLANG_TIDY_GCC" >&2
      exit 1
    fi
  fi
}

### Find input files ################################################

find_input_files() {

  git ls-files -- 'src/*.[ch]' | grep -E -v "$CLANG_TIDY_EXCLUDES"
}

### Run clang-tidy ###################################################

run_clang_tidy() {

  ensure_sysroot

  CLANG_TIDY_FLAGS="${CLANG_TIDY_FLAGS:- }-isystem${CLANG_TIDY_SYSROOT}"

  local input_files
  input_files=$(find_input_files)

  clang-tidy-6.0 -export-fixes=fixes.yaml $input_files -- \
    ${CLANG_TIDY_FLAGS} ${CLANG_TIDY_INCLUDES}
}

### Dockerized clang-tidy ############################################

docker_run_clang_tidy() {

  local version_tag
  version_tag=$(cat "$D/docker_version_tag")

  local docker_repo=swiftnav/clang-tidy
  local docker_tag=$docker_repo:$version_tag

  if [[ -z "${NON_INTERACTIVE_BUILD:-}" ]]; then
    INTERACTIVE_ARGS=$({ tty &>/dev/null && echo "--tty --interactive"; } || echo)
  else
    INTERACTIVE_ARGS=
  fi

  ensure_sysroot

  docker rm -f swiftnav-clang-tidy &>/dev/null || :

  docker run \
    -v "$ROOT:/work" \
    -v "$CLANG_TIDY_SYSROOT:/sysroot" \
    -e CLANG_TIDY_INCLUDES="${CLANG_TIDY_INCLUDES//$ROOT/\/work}" \
    -e CLANG_TIDY_EXCLUDES="$CLANG_TIDY_EXCLUDES" \
    -e CLANG_TIDY_FLAGS="$CLANG_TIDY_FLAGS" \
    -e CLANG_TIDY_SYSROOT="/sysroot" \
    -e DEBUG="${DEBUG-}" \
    ${INTERACTIVE_ARGS} \
    --rm \
    --name swiftnav-clang-tidy \
    "$docker_tag" \
    ./scripts/ci/clang-tidy.sh --run \
    | sed -e "s@/work@$ROOT@g" -e "s@/sysroot@$CLANG_TIDY_SYSROOT@g"
}

### Main ############################################################

main() {

  if [[ -n "$CLANG_TIDY_FIND_SYSROOT" ]]; then
    find_sysroot
  elif [[ -n "$CLANG_TIDY_RUN" ]]; then
    run_clang_tidy
  elif [[ -n "$CLANG_TIDY_DOCKER_RUN" ]]; then
    docker_run_clang_tidy
  fi
}

main
