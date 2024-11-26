#!/bin/bash
set -e

make clean
rm -f panel-dmb-kd029qhfid001.ko.xz

make
xz panel-dmb-kd029qhfid001.ko
