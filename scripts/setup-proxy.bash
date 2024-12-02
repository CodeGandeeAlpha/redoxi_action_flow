#!/bin/bash

# if PEI_HTTP_PROXY_2 is set, use it for proxy settings
if [ ! -z "$PEI_HTTP_PROXY_2" ]; then
    echo "Setting proxy to $PEI_HTTP_PROXY_2"
    export http_proxy=$PEI_HTTP_PROXY_2
    export https_proxy=$PEI_HTTP_PROXY_2
fi