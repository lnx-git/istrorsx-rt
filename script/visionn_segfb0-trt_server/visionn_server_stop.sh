#!/bin/bash
# NOTE: legacy used "sudo pkill -f visionn_server.py" -- sudo dropped here,
# not needed to signal your own background process (only legacy's package
# installs actually needed root, e.g. install/*.sh elsewhere in this project).
pkill -f visionn_server.py
ps -ef | grep visionn_server.py | grep -v grep
