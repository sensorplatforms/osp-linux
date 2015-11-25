To install this snap shot:

1. Flash kernel/ramdisk to the N7.
fastboot flash:raw boot debug-13 OSP-ramdisk.gz

2. Install daemon and HAL.
  (a) Create /data/tmp. Make it world read/write to min. permissions issues.
You will need to be root to do this.
  (b) Push N7.config to /data/tmp
  (c) Install osp-start and OSPDaemon to /system/bin
  (d) Make OSPDaemon and osp-start mode 755.
  (e) Rename/replace the existing /system/lib/hw/sensors.grouper.so
  (f) Install sensors.osp-25sep2015.so as /system/lib/hw/sensors.grouper.so

Reboot.
