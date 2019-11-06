#!/usr/bin/env bash

################################################################################
# This script runs the dnsmasq service and allows it to be done from a launch
# file. Simply set the password for your user in the appropriate place and make
# this file executable. Then it can be added to a launch file and run as a node.
################################################################################

# Start the dnsmasq program
echo [user_password] | sudo -S systemctl stop dnsmasq
echo [user_password] | sudo -S systemctl start dnsmasq
