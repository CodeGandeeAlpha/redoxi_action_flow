#!/bin/bash

# dir of this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source $DIR/xenable_v6d.bash

echo "Starting vineyard with service type: $RDX_SHM_SERVICE_TYPE and region key: $RDX_SHM_REGION_KEY"

python3 -m vineyard --socket $RDX_SHM_REGION_KEY
