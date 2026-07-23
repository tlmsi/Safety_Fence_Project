#!/usr/bin/env bash
set -Eeuo pipefail

RUN_DIR="$(
    cd -- "$(
        dirname -- "${BASH_SOURCE[0]}"
    )"
    pwd
)"

exec \
    "$RUN_DIR/04_red_automation.sh" \
    run
