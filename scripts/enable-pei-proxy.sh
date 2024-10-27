#!/bin/bash

# Check and set HTTP proxy if PEI_HTTP_PROXY_1 exists
if [ ! -z "${PEI_HTTP_PROXY_1}" ]; then
  export http_proxy="${PEI_HTTP_PROXY_1}"
  export HTTP_PROXY="${PEI_HTTP_PROXY_1}"
fi

# Check and set HTTPS proxy if PEI_HTTPS_PROXY_1 exists 
if [ ! -z "${PEI_HTTPS_PROXY_1}" ]; then
  export https_proxy="${PEI_HTTPS_PROXY_1}"
  export HTTPS_PROXY="${PEI_HTTPS_PROXY_1}"
fi

