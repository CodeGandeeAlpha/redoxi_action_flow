#!/bin/bash

# set the shared memory service type and region key
export RDX_SHM_SERVICE_NAME=vineyard
echo "RDX_SHM_SERVICE_NAME: $RDX_SHM_SERVICE_NAME"

export RDX_SHM_REGION_KEY=/soft/data/vineyard.sock
echo "RDX_SHM_REGION_KEY: $RDX_SHM_REGION_KEY"
