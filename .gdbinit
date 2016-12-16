set mem inaccessible-by-default off
target extended-remote /dev/ttyACM0
mon jtag_scan 6 4
attach 1
