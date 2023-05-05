#!/bin/bash
sudo rm   /etc/udev/rules.d/mycar.rules
sudo service udev reload
sudo service udev restart
