#!/bin/bash
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_usb.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_cp2102.rules

service udev reload
sleep 2
service udev restart

