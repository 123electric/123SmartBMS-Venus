#!/bin/bash
#
# Start script for BMV vedirect
#	First parameter: tty device to use
#
# Keep this script running with daemon tools. If it exits because the
# connection crashes, or whatever, daemon tools will start a new one.
#

. /opt/victronenergy/serial-starter/run-service.sh $tty

if [ ! -h $alias ]; then ln -s /dev/$tty $alias; fi
app="/usr/bin/python /data/123SmartBMS-Venus/smartbms.py"
start -d /dev/$tty
