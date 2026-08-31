#!/bin/bash

cd /home/tnswo/ECM-19.0 || exit 1

BRANCH=$(git branch --show-current)

if [ "$BRANCH" != "codex-dev" ]; then
    echo "Current branch is not codex-dev."
    exit 1
fi

git add -A

if git diff --cached --quiet; then
    echo "No changes."
    exit 0
fi

git commit -m "Codex sync $(date '+%Y-%m-%d %H:%M:%S')"
git push origin codex-dev

