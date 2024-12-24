#!/bin/bash

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root"
    exit 1
fi

echo "Adding all users to video group in order to access the rknpu"

# Get all users
users=$(cut -d: -f1 /etc/passwd)

# Add each user to video group
for user in $users; do
    # Skip system users
    if id -u "$user" >/dev/null 2>&1 && [ $(id -u "$user") -ge 1000 ]; then
        echo "Adding user $user to video group"
        usermod -a -G video "$user"
    fi
done
