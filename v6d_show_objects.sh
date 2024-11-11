#!/bin/bash

v6d_console="./tmp/vineyardctl-v0.24.2-linux-amd64"
ipc_socket="/soft/data/vineyard.sock"

# Default limit value
limit=100

# Function to display usage
usage() {
    echo "Usage: $0 [--limit <number>] [--help]"
    echo "  --limit <number>  Set the limit for the number of objects to show (default is 100)"
    echo "  --help            Display this help message"
}

# Parse input arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --limit) limit="$2"; shift ;;
        --help) usage; exit 0 ;;
        *) echo "Unknown parameter passed: $1"; usage; exit 1 ;;
    esac
    shift
done

echo "show v6d objects with limit $limit"
$v6d_console ls objects --limit $limit --ipc-socket $ipc_socket