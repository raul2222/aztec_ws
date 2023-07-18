#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the synexens usb connection as /dev/"
echo ""

sudo cp synexens-usb.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm trigger --action=change
