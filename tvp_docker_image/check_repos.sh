#!/usr/bin/env bash

find . -type d -name ".git" | while read gitdir; do
    repo=$(dirname "$gitdir")
    
    status=$(git -C "$repo" status --porcelain)

    if [ -n "$status" ]; then
        echo "=============================="
        echo "Repo: $repo"
        echo "------------------------------"
        git -C "$repo" status -s
        echo
    fi
done
