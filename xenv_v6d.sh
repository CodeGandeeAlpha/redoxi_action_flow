#!/bin/bash

# set the shared memory service type and region key
export RDX_SHM_SERVICE_TYPE=vineyard
echo "RDX_SHM_SERVICE_TYPE: $RDX_SHM_SERVICE_TYPE"

export RDX_SHM_REGION_KEY=/soft/data/vineyard.sock
echo "RDX_SHM_REGION_KEY: $RDX_SHM_REGION_KEY"
