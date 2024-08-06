#!/usr/bin/env bash

set -euo pipefail

echo -n "Setting up Git pre-commit formatting check... "

ln -s ../../ci/git-pre-commit-hook .git/hooks/pre-commit

echo "done!"
