#!/bin/bash
sudo cp mycar.rules  /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
