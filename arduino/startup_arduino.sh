#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="arduino"' >/etc/udev/rules.d/99-arduino.rules


service udev reload
sleep 2
service udev restart

