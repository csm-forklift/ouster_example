#!/usr/bin/env bash

# Start the dnsmasq program
echo csmrobotics | sudo -S systemctl stop dnsmasq
echo csmrobotics | sudo -S systemctl start dnsmasq
