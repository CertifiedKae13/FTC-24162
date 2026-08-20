#!/usr/bin/env bash
# The real FTC gradle project lives in the FTC-Robot/ subdirectory.
# This wrapper lets you run `./gradlew ...` from the repo root directly.
cd "$(dirname "$0")/FTC-Robot" || exit 1
exec ./gradlew "$@"
