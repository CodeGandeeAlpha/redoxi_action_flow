#!/bin/bash

export RDX_SHM_SERVICE_TYPE=vineyard
export RDX_SHM_REGION_KEY=/soft/data/vineyard.sock

echo "Setting environment variables for vineyard"
echo "Exported RDX_SHM_SERVICE_TYPE: $RDX_SHM_SERVICE_TYPE and RDX_SHM_REGION_KEY: $RDX_SHM_REGION_KEY"
