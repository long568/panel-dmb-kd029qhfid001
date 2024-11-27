#!/bin/bash
set -e

make clean
rm -f panel-dmb-kd029qhfid001.ko.xz

make
xz panel-dmb-kd029qhfid001.ko
scp panel-dmb-kd029qhfid001.ko.xz lo@192.168.0.105:/home/lo/
