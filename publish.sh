#!/bin/bash
# Publish everything to GitHub: commit all changes, pull teammates' commits, push.
# Usage:  ./publish.sh "what I changed"      (message optional)
set -e
cd "$(dirname "$0")"
MSG="${1:-update $(date '+%Y-%m-%d %H:%M')}"
git add -A
if git diff --cached --quiet; then
  echo "Nothing new to commit."
else
  echo "Committing:"; git diff --cached --stat | tail -15
  git commit -q -m "$MSG"
fi
git pull --rebase --autostash -q          # get teammates' commits first (stops here on a conflict)
git push -q
echo "Published: $(git log -1 --format='%h %s') -> $(git rev-parse --abbrev-ref @{u})"
