#!/usr/bin/env bash

set -euo pipefail

if [[ -z "${SSH_AUTH_SOCK:-}" ]]
then
  eval $(ssh-agent)
fi

if ! ssh-add -l >/dev/null 2>&1; then
  ssh-add ~/.ssh/id_rsa
fi

PROJECT_NAME="tvp_image"
MODE="${1:-normal}"

case "${MODE}" in
  normal)
    docker compose --project-name "${PROJECT_NAME}" build --ssh default
    ;;
  debug)
    # Verbose BuildKit output helps identify the exact failing layer.
    docker compose --project-name "${PROJECT_NAME}" build --ssh default --progress=plain
    ;;
  no-cache)
    # Rebuild everything from scratch to avoid stale cache artifacts.
    docker compose --project-name "${PROJECT_NAME}" build --ssh default --no-cache
    ;;
  rebuild-python)
    # Rebuild starts only from tvp gitlab packages
    docker compose --project-name "${PROJECT_NAME}" build --ssh default \
      --build-arg REBUILD_PYTHON_PACKAGES=$(date +%s)
    ;;
  rebuild-git)
    # Rebuild starts only from tvp gitlab packages
    docker compose --project-name "${PROJECT_NAME}" build --ssh default \
      --build-arg REBUILD_GITLAB_PACKAGES=$(date +%s)
    ;;
  rebuild-freq)
    # Rebuild starts only from frequenlty changing navigation packages
    docker compose --project-name "${PROJECT_NAME}" build --ssh default \
      --build-arg REBUILD_FREQUENTLY_CHANGING_NAVIGATION_PACKAGES=$(date +%s)
    ;;
  *)
    echo "Usage: $0 [normal|debug|no-cache|rebuild-python|rebuild-git|rebuild-freq]"
    exit 1
    ;;
esac
