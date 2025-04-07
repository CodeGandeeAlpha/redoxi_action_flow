#!/bin/bash

# get the dir of this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export RDX_SHM_SERVICE_TYPE=v6d
export RDX_SHM_REGION_KEY="$DIR/tmp/vineyard.sock"
export RDX_SHM_DEFAULT_ALIVE_DURATION_SEC="10.0"

echo "Setting environment variables for vineyard"
echo "Exported RDX_SHM_SERVICE_TYPE: $RDX_SHM_SERVICE_TYPE"
echo "Exported RDX_SHM_REGION_KEY: $RDX_SHM_REGION_KEY"
echo "Exported RDX_SHM_DEFAULT_ALIVE_DURATION_SEC: $RDX_SHM_DEFAULT_ALIVE_DURATION_SEC"
