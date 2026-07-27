#!/bin/bash
# Build the R1 Lite development image from a clean tree with a pinned
# platform and OCI identity labels. Run from anywhere inside the repo.
#
#   docker/galaxea-r1lite/build.sh [tag-suffix]
#
# Tag: ghcr.io/dimensionalos/dimos-r1lite:<version>-r1lite-dev.<N>
# where <version> comes from pyproject.toml and <N> defaults to 0.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

if [ -n "$(git status --porcelain)" ]; then
    echo "FAIL: refusing to build from a dirty tree; commit or stash first" >&2
    git status --short >&2
    exit 1
fi

REVISION="$(git rev-parse HEAD)"
VERSION="$(grep -m1 '^version' pyproject.toml | sed 's/.*"\(.*\)".*/\1/')"
DEV_N="${1:-0}"
TAG="ghcr.io/dimensionalos/dimos-r1lite:${VERSION}-r1lite-dev.${DEV_N}"

docker build \
    --platform linux/amd64 \
    -f docker/galaxea-r1lite/Dockerfile \
    --label "org.opencontainers.image.source=https://github.com/dimensionalOS/dimos" \
    --label "org.opencontainers.image.revision=${REVISION}" \
    --label "org.opencontainers.image.version=${VERSION}-r1lite-dev.${DEV_N}" \
    -t "$TAG" \
    .

echo "Built $TAG at revision $REVISION"
echo "Digest after push: docker inspect --format '{{index .RepoDigests 0}}' $TAG"
