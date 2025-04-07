#!/bin/bash

# get the directory of the script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# get all sub directories of the script directory, excluding the current directory
MOD_DIRS=$(find "$SCRIPT_DIR" -mindepth 1 -maxdepth 1 -type d)

# copy MOD_DIRS to SCRIPT_DIR/.., overriding the existing files
for MOD_DIR in $MOD_DIRS; do
    MOD_NAME=$(basename "$MOD_DIR")
    MOD_TARGET_DIR="$SCRIPT_DIR/../"
    cp -r "$MOD_DIR" "$MOD_TARGET_DIR"
done

