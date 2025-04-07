#!/bin/bash

# get the dir of this script
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# look for PYTHONPATH in env variable, set it to py_path
py_path=$(printenv PYTHONPATH)

# if it is not empty, write the python path to .env file
if [ ! -z "$py_path" ]; then
    echo "PYTHONPATH=$py_path" > "$DIR/.env"
    echo "python path updated, now is $py_path"
fi