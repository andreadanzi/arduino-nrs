#!/bin/sh

echo 18 > /sys/class/gpio/export
echo "high" > /sys/class/gpio/gpio18/direction
echo 1 > /sys/class/gpio/gpio18/value
echo 0 > /sys/class/gpio/gpio18/value
echo 18 > /sys/class/gpio/unexport
