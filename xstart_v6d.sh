#!/bin/bash

# dir of this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 -m vineyard --socket /soft/app/vineyard.sock