import init.grouper.usb.rc

on early-init
    # Set init and its forked children's oom_adj.
    write /proc/1/oom_score_adj -1000

    mount debugfs debugfs /sys/kernel/debug

on early-boot
    write /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor interactive
    write /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor interactive
    write /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor interactive
    write /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor interactive

on fs
    setprop ro.crypto.umount_sd false
    mount_all /fstab.grouper

on post-fs-data
    mkdir /data/media 0770 media_rw media_rw

    # change back to bluetooth from system
    chown bluetooth net_bt_stack /data/misc/bluetooth

    # sensors-config
    mkdir /data/sensors 751
    # /data/sensors was owned by system/system earlier.
    # Force it to root/root if it already exists.
    chown root root /data/sensors
    mkdir /data/lightsensor 751
    # /data/lightsensor was owned by system/system earlier.
    # Force it to root/root if it already exists.
    chown root root /data/lightsensor
    mkdir /data/calibration
    mkdir /data/amit
    # OSP-SH
    mkdir /data/tmp 777

    # gps
    mkdir /data/gps
    chown gps system /data/gps
    chmod 1770 /data/gps
    write /sys/class/gpio/export 162
    write /sys/class/gpio/gpio162/value 0
    write /sys/class/gpio/gpio162/direction out
    chown gps system /sys/class/gpio/gpio162/value
    chmod 0644 /sys/class/gpio/gpio162/value
    chown gps system /dev/ttyHS1
    chmod 0660 /dev/ttyHS1

    # Set indication (checked by vold) that we have finished this action
    setprop vold.post_fs_data_done 1

on boot
    # OSP-SH
    write /sys/bus/i2c/devices/i2c-2/new_device "ospsh 0x18"

# bluetooth
    # UART device
    chmod 0660 /dev/ttyHS2
    chown bluetooth net_bt_stack /dev/ttyHS2

    # power up/down interface
    chmod 0660 /sys/class/rfkill/rfkill0/state
    chmod 0660 /sys/class/rfkill/rfkill0/type
    chown bluetooth net_bt_stack /sys/class/rfkill/rfkill0/state
    chown bluetooth net_bt_stack /sys/class/rfkill/rfkill0/type

    # bluetooth MAC address programming
    chown bluetooth net_bt_stack ro.bt.bdaddr_path
    chown bluetooth net_bt_stack /system/etc/bluetooth
    chown bluetooth net_bt_stack /data/misc/bluetooth
    setprop ro.bt.bdaddr_path "/data/misc/bluetooth/bdaddr"

    # bluetooth LPM
    chmod 0220 /proc/bluetooth/sleep/lpm
    chmod 0220 /proc/bluetooth/sleep/btwrite
    chown bluetooth net_bt_stack /proc/bluetooth/sleep/lpm
    chown bluetooth net_bt_stack /proc/bluetooth/sleep/btwrite

# NFC
    setprop ro.nfc.port "I2C"
    chmod 0660 /dev/pn544
    chown nfc nfc /dev/pn544

# backlight
    chown system system /sys/class/backlight/pwm-backlight/brightness

# didim
    chown system system /sys/class/graphics/fb0/device/smartdimmer/enable
    chown system system /sys/class/graphics/fb0/device/smartdimmer/aggressiveness

# power
    chown system system /sys/kernel/tegra_cap/core_cap_level
    chown system system /sys/kernel/tegra_cap/core_cap_state
    chown system system /sys/module/cpu_tegra/parameters/cpu_user_cap

# Sensor
# iio Audience Start
    chown system system /dev/iio:device0
    chown system system /dev/iio:device1
    chown system system /dev/iio:device2
    chown system system /dev/iio:device3
    chown system system /dev/iio:device4
    chown system system /dev/iio:device5
    chown system system /dev/iio:device6
    chown system system /dev/iio:device7
    chown system system /dev/iio:device8
    chown system system /dev/iio:device9
    chown system system /dev/iio:device10
    chown system system /dev/iio:device11
    chown system system /dev/iio:device12
    chown system system /dev/iio:device13
    
    chown system system /sys/bus/iio/devices/trigger0/name
    chown system system /sys/bus/iio/devices/trigger1/name
    chown system system /sys/bus/iio/devices/trigger2/name
    chown system system /sys/bus/iio/devices/trigger3/name
    chown system system /sys/bus/iio/devices/trigger4/name
    chown system system /sys/bus/iio/devices/trigger5/name
    chown system system /sys/bus/iio/devices/trigger6/name
    chown system system /sys/bus/iio/devices/trigger7/name
    chown system system /sys/bus/iio/devices/trigger8/name
    chown system system /sys/bus/iio/devices/trigger9/name
    chown system system /sys/bus/iio/devices/trigger10/name
    chown system system /sys/bus/iio/devices/trigger11/name
    chown system system /sys/bus/iio/devices/trigger12/name
    chown system system /sys/bus/iio/devices/trigger13/name
    
    chown system system /sys/bus/iio/devices/iio:device0/enable
    chown system system /sys/bus/iio/devices/iio:device0/buffer/length
    chown system system /sys/bus/iio/devices/iio:device0/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en
    chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en
    chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en
    chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device0/trigger/current_trigger
    
    chown system system /sys/bus/iio/devices/iio:device1/enable
    chown system system /sys/bus/iio/devices/iio:device1/buffer/length
    chown system system /sys/bus/iio/devices/iio:device1/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_x_en
    chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_y_en
    chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_z_en
    chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device1/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device2/enable
    chown system system /sys/bus/iio/devices/iio:device2/buffer/length
    chown system system /sys/bus/iio/devices/iio:device2/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device2/scan_elements/in_incli_x_en
    chown system system /sys/bus/iio/devices/iio:device2/scan_elements/in_incli_y_en
    chown system system /sys/bus/iio/devices/iio:device2/scan_elements/in_incli_z_en
    chown system system /sys/bus/iio/devices/iio:device2/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device2/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device3/enable
    chown system system /sys/bus/iio/devices/iio:device3/buffer/length
    chown system system /sys/bus/iio/devices/iio:device3/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device3/scan_elements/in_anglvel_x_en
    chown system system /sys/bus/iio/devices/iio:device3/scan_elements/in_anglvel_y_en
    chown system system /sys/bus/iio/devices/iio:device3/scan_elements/in_anglvel_z_en
    chown system system /sys/bus/iio/devices/iio:device3/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device3/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device4/enable
    chown system system /sys/bus/iio/devices/iio:device4/buffer/length
    chown system system /sys/bus/iio/devices/iio:device4/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device4/scan_elements/in_anglvel_x_en
    chown system system /sys/bus/iio/devices/iio:device4/scan_elements/in_anglvel_y_en
    chown system system /sys/bus/iio/devices/iio:device4/scan_elements/in_anglvel_z_en
    chown system system /sys/bus/iio/devices/iio:device4/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device4/trigger/current_trigger
    
    chown system system /sys/bus/iio/devices/iio:device5/enable
    chown system system /sys/bus/iio/devices/iio:device5/buffer/length
    chown system system /sys/bus/iio/devices/iio:device5/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device5/scan_elements/in_accel_x_en
    chown system system /sys/bus/iio/devices/iio:device5/scan_elements/in_accel_y_en
    chown system system /sys/bus/iio/devices/iio:device5/scan_elements/in_accel_z_en
    chown system system /sys/bus/iio/devices/iio:device5/scan_elements/in_timestamp_en    
    chown system system /sys/bus/iio/devices/iio:device5/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device6/enable
    chown system system /sys/bus/iio/devices/iio:device6/buffer/length
    chown system system /sys/bus/iio/devices/iio:device6/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device6/scan_elements/in_accel_x_en
    chown system system /sys/bus/iio/devices/iio:device6/scan_elements/in_accel_y_en
    chown system system /sys/bus/iio/devices/iio:device6/scan_elements/in_accel_z_en
    chown system system /sys/bus/iio/devices/iio:device6/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device6/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device7/enable
    chown system system /sys/bus/iio/devices/iio:device7/buffer/length
    chown system system /sys/bus/iio/devices/iio:device7/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device7/scan_elements/in_quaternion_r_en
    chown system system /sys/bus/iio/devices/iio:device7/scan_elements/in_quaternion_z_en
    chown system system /sys/bus/iio/devices/iio:device7/scan_elements/in_quaternion_y_en
    chown system system /sys/bus/iio/devices/iio:device7/scan_elements/in_quaternion_x_en
    chown system system /sys/bus/iio/devices/iio:device7/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device7/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device8/enable
    chown system system /sys/bus/iio/devices/iio:device8/buffer/length
    chown system system /sys/bus/iio/devices/iio:device8/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device8/scan_elements/in_magn_x_en
    chown system system /sys/bus/iio/devices/iio:device8/scan_elements/in_magn_y_en
    chown system system /sys/bus/iio/devices/iio:device8/scan_elements/in_magn_z_en
    chown system system /sys/bus/iio/devices/iio:device8/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device8/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device9/enable
    chown system system /sys/bus/iio/devices/iio:device9/buffer/length
    chown system system /sys/bus/iio/devices/iio:device9/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device9/scan_elements/in_quaternion_r_en
    chown system system /sys/bus/iio/devices/iio:device9/scan_elements/in_quaternion_z_en
    chown system system /sys/bus/iio/devices/iio:device9/scan_elements/in_quaternion_y_en
    chown system system /sys/bus/iio/devices/iio:device9/scan_elements/in_quaternion_x_en
    chown system system /sys/bus/iio/devices/iio:device9/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device9/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device10/enable
    chown system system /sys/bus/iio/devices/iio:device10/buffer/length
    chown system system /sys/bus/iio/devices/iio:device10/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device10/scan_elements/in_anglvel_x_en
    chown system system /sys/bus/iio/devices/iio:device10/scan_elements/in_anglvel_y_en
    chown system system /sys/bus/iio/devices/iio:device10/scan_elements/in_anglvel_z_en
    chown system system /sys/bus/iio/devices/iio:device10/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device10/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device11/enable
    chown system system /sys/bus/iio/devices/iio:device11/buffer/length
    chown system system /sys/bus/iio/devices/iio:device11/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device11/scan_elements/in_accel_x_en
    chown system system /sys/bus/iio/devices/iio:device11/scan_elements/in_accel_y_en
    chown system system /sys/bus/iio/devices/iio:device11/scan_elements/in_accel_z_en
    chown system system /sys/bus/iio/devices/iio:device11/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device11/trigger/current_trigger
    
    chown system system /sys/bus/iio/devices/iio:device12/enable
    chown system system /sys/bus/iio/devices/iio:device12/buffer/length
    chown system system /sys/bus/iio/devices/iio:device12/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device12/scan_elements/in_accel_x_en
    chown system system /sys/bus/iio/devices/iio:device12/scan_elements/in_accel_y_en
    chown system system /sys/bus/iio/devices/iio:device12/scan_elements/in_accel_z_en
    chown system system /sys/bus/iio/devices/iio:device12/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device12/trigger/current_trigger

    chown system system /sys/bus/iio/devices/iio:device13/enable
    chown system system /sys/bus/iio/devices/iio:device13/buffer/length
    chown system system /sys/bus/iio/devices/iio:device13/buffer/enable
    chown system system /sys/bus/iio/devices/iio:device13/scan_elements/in_quaternion_r_en
    chown system system /sys/bus/iio/devices/iio:device13/scan_elements/in_quaternion_z_en
    chown system system /sys/bus/iio/devices/iio:device13/scan_elements/in_quaternion_y_en
    chown system system /sys/bus/iio/devices/iio:device13/scan_elements/in_quaternion_x_en
    chown system system /sys/bus/iio/devices/iio:device13/scan_elements/in_timestamp_en
    chown system system /sys/bus/iio/devices/iio:device13/trigger/current_trigger
  
    chmod 600 /dev/iio:device0
    chmod 600 /dev/iio:device1
    chmod 600 /dev/iio:device2
    chmod 600 /dev/iio:device3
    chmod 600 /dev/iio:device4
    chmod 600 /dev/iio:device5
    chmod 600 /dev/iio:device6
    chmod 600 /dev/iio:device7
    chmod 600 /dev/iio:device8
    chmod 600 /dev/iio:device9
    chmod 600 /dev/iio:device10
    chmod 600 /dev/iio:device11
    chmod 600 /dev/iio:device12
    chmod 600 /dev/iio:device13
    
    chmod -R 600 /sys/bus/iio/devices/iio:device0
    chmod -R 600 /sys/bus/iio/devices/iio:device1
    chmod -R 600 /sys/bus/iio/devices/iio:device2
    chmod -R 600 /sys/bus/iio/devices/iio:device3
    chmod -R 600 /sys/bus/iio/devices/iio:device4
    chmod -R 600 /sys/bus/iio/devices/iio:device5
    chmod -R 600 /sys/bus/iio/devices/iio:device6
    chmod -R 600 /sys/bus/iio/devices/iio:device7
    chmod -R 600 /sys/bus/iio/devices/iio:device8
    chmod -R 600 /sys/bus/iio/devices/iio:device9
    chmod -R 600 /sys/bus/iio/devices/iio:device10
    chmod -R 600 /sys/bus/iio/devices/iio:device11
    chmod -R 600 /sys/bus/iio/devices/iio:device12
    chmod -R 600 /sys/bus/iio/devices/iio:device13
# iio Audiecne End
# iio
    #chown system system /dev/iio:device0
    #chown system system /sys/bus/iio/devices/trigger0/name
    #chown system system /sys/bus/iio/devices/iio:device0/accl_enable
    #chown system system /sys/bus/iio/devices/iio:device0/accl_matrix
    #chown system system /sys/bus/iio/devices/iio:device0/buffer/length
    #chown system system /sys/bus/iio/devices/iio:device0/buffer/enable
    #chown system system /sys/bus/iio/devices/iio:device0/compass_enable
    #chown system system /sys/bus/iio/devices/iio:device0/compass_matrix
    #chown system system /sys/bus/iio/devices/iio:device0/dmp_on
    #chown system system /sys/bus/iio/devices/iio:device0/dmp_int_on
    #chown system system /sys/bus/iio/devices/iio:device0/gyro_enable
    #chown system system /sys/bus/iio/devices/iio:device0/gyro_matrix
    #chown system system /sys/bus/iio/devices/iio:device0/in_accel_scale
    #chown system system /sys/bus/iio/devices/iio:device0/in_anglvel_scale
    #chown system system /sys/bus/iio/devices/iio:device0/in_magn_scale
    #chown system system /sys/bus/iio/devices/iio:device0/key
    #chown system system /sys/bus/iio/devices/iio:device0/power_state
    #chown system system /sys/bus/iio/devices/iio:device0/sampling_frequency
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_x_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_y_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_z_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_x_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_y_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_z_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en
    #chown system system /sys/bus/iio/devices/iio:device0/temperature
    #chown system system /sys/bus/iio/devices/iio:device0/trigger/current_trigger

    ## new in MA 5.1.5
    #chown system system /sys/bus/iio/devices/iio:device0/secondary_name
    #chown system system /sys/bus/iio/devices/iio:device0/dmp_firmware
    #chown system system /sys/bus/iio/devices/iio:device0/firmware_loaded
    #chown system system /sys/bus/iio/devices/iio:device0/dmp_event_int_on
    #chown system system /sys/bus/iio/devices/iio:device0/dmp_output_rate
    #chown system system /sys/bus/iio/devices/iio:device0/in_accel_x_offset
    #chown system system /sys/bus/iio/devices/iio:device0/in_accel_y_offset
    #chown system system /sys/bus/iio/devices/iio:device0/in_accel_z_offset
    #chown system system /sys/bus/iio/devices/iio:device0/gyro_fsr
    ### LPQ
    #chown system system /sys/bus/iio/devices/iio:device0/quaternion_on
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_z_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_y_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_x_en
    #chown system system /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_r_en
    ### Screen orientation
    #chown system system /sys/bus/iio/devices/iio:device0/event_display_orientation
    #chown system system /sys/bus/iio/devices/iio:device0/display_orientation_on
    ### SMD
    #chown system system /sys/bus/iio/devices/iio:device0/event_smd
    #chown system system /sys/bus/iio/devices/iio:device0/smd_enable
    #chown system system /sys/bus/iio/devices/iio:device0/smd_threshold
    #chown system system /sys/bus/iio/devices/iio:device0/smd_delay_threshold
    #chown system system /sys/bus/iio/devices/iio:device0/smd_delay_threshold2
    ### AMI compass sensor
    #chown system system /dev/iio:device1
    #chown system system /sys/bus/iio/devices/trigger1/name
    #chown system system /sys/bus/iio/devices/iio:device1/buffer/length
    #chown system system /sys/bus/iio/devices/iio:device1/buffer/enable
    #chown system system /sys/bus/iio/devices/iio:device1/compass_enable
    #chown system system /sys/bus/iio/devices/iio:device1/compass_matrix
    #chown system system /sys/bus/iio/devices/iio:device1/in_magn_scale
    #chown system system /sys/bus/iio/devices/iio:device1/power_state
    #chown system system /sys/bus/iio/devices/iio:device1/sampling_frequency
    #chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_x_en
    #chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_y_en
    #chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_z_en
    #chown system system /sys/bus/iio/devices/iio:device1/scan_elements/in_timestamp_en
    #chown system system /sys/bus/iio/devices/iio:device1/trigger/current_trigger

    #chmod 600 /dev/iio:device0
    #chmod 600 /sys/bus/iio/devices/trigger0/name
    #chmod 600 /sys/bus/iio/devices/iio:device0/accl_enable
    #chmod 600 /sys/bus/iio/devices/iio:device0/accl_matrix
    #chmod 600 /sys/bus/iio/devices/iio:device0/buffer/length
    #chmod 600 /sys/bus/iio/devices/iio:device0/buffer/enable
    #chmod 600 /sys/bus/iio/devices/iio:device0/compass_enable
    #chmod 600 /sys/bus/iio/devices/iio:device0/compass_matrix
    #chmod 600 /sys/bus/iio/devices/iio:device0/dmp_on
    #chmod 600 /sys/bus/iio/devices/iio:device0/dmp_int_on
    #chmod 600 /sys/bus/iio/devices/iio:device0/gyro_enable
    #chmod 600 /sys/bus/iio/devices/iio:device0/gyro_matrix
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_accel_scale
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_anglvel_scale
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_magn_scale
    #chmod 600 /sys/bus/iio/devices/iio:device0/key
    #chmod 600 /sys/bus/iio/devices/iio:device0/power_state
    #chmod 600 /sys/bus/iio/devices/iio:device0/sampling_frequency
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_x_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_y_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_anglvel_z_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_x_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_y_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_magn_z_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/temperature
    #chmod 600 /sys/bus/iio/devices/iio:device0/trigger/current_trigger
    ## new in MA 5.1.5
    #chmod 600 /sys/bus/iio/devices/iio:device0/secondary_name
    #chmod 600 /sys/bus/iio/devices/iio:device0/dmp_firmware
    #chmod 600 /sys/bus/iio/devices/iio:device0/firmware_loaded
    #chmod 600 /sys/bus/iio/devices/iio:device0/dmp_event_int_on
    #chmod 600 /sys/bus/iio/devices/iio:device0/dmp_output_rate
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_accel_x_offset
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_accel_y_offset
    #chmod 600 /sys/bus/iio/devices/iio:device0/in_accel_z_offset
    #chmod 600 /sys/bus/iio/devices/iio:device0/gyro_fsr
    ### LPQ
    #chmod 600 /sys/bus/iio/devices/iio:device0/quaternion_on
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_z_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_y_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_x_en
    #chmod 600 /sys/bus/iio/devices/iio:device0/scan_elements/in_quaternion_r_en
    ### Screen orientation
    #chmod 600 /sys/bus/iio/devices/iio:device0/event_display_orientation
    #chmod 600 /sys/bus/iio/devices/iio:device0/display_orientation_on
    ### SMD
    #chmod 600 /sys/bus/iio/devices/iio:device0/event_smd
    #chmod 600 /sys/bus/iio/devices/iio:device0/smd_enable
    #chmod 600 /sys/bus/iio/devices/iio:device0/smd_threshold
    #chmod 600 /sys/bus/iio/devices/iio:device0/smd_delay_threshold
    #chmod 600 /sys/bus/iio/devices/iio:device0/smd_delay_threshold2
    ### AMI compass sensor
    #chmod 600 /dev/iio:device1
    #chmod 600 /sys/bus/iio/devices/trigger1/name
    #chmod 600 /sys/bus/iio/devices/iio:device1/buffer/length
    #chmod 600 /sys/bus/iio/devices/iio:device1/buffer/enable
    #chmod 600 /sys/bus/iio/devices/iio:device1/compass_enable
    #chmod 600 /sys/bus/iio/devices/iio:device1/compass_matrix
    #chmod 600 /sys/bus/iio/devices/iio:device1/in_magn_scale
    #chmod 600 /sys/bus/iio/devices/iio:device1/power_state
    #chmod 600 /sys/bus/iio/devices/iio:device1/sampling_frequency
    #chmod 600 /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_x_en
    #chmod 600 /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_y_en
    #chmod 600 /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_z_en
    #chmod 600 /sys/bus/iio/devices/iio:device1/scan_elements/in_timestamp_en
    #chmod 600 /sys/bus/iio/devices/iio:device1/trigger/current_trigger

# Power management settings
    write /sys/module/cpu_tegra3/parameters/no_lp 0
    #write /sys/module/tegra3_emc/parameters/emc_enable 0
    #write /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 1000000
    write /sys/devices/system/cpu/cpufreq/interactive/boost_factor 2
    write /sys/devices/system/cpu/cpufreq/interactive/input_boost 1
    write /sys/devices/system/cpu/cpufreq/interactive/sustain_load 80
    write /sys/module/cpu_tegra3/parameters/auto_hotplug 1
    #write /sys/module/cpuidle_t3/parameters/lp2_0_in_idle 0
    write /sys/module/cpuidle/parameters/lp2_in_idle 0

# Interactive governor settings
    chown system system /sys/devices/system/cpu/cpufreq/interactive/boost_factor
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/boost_factor
    chown system system /sys/devices/system/cpu/cpufreq/interactive/core_lock_count
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/core_lock_count
    chown system system /sys/devices/system/cpu/cpufreq/interactive/core_lock_period
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/core_lock_period
    chown system system /sys/devices/system/cpu/cpufreq/interactive/go_maxspeed_load
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/go_maxspeed_load
    chown system system /sys/devices/system/cpu/cpufreq/interactive/io_is_busy
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/io_is_busy
    chown system system /sys/devices/system/cpu/cpufreq/interactive/max_boost
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/max_boost
    chown system system /sys/devices/system/cpu/cpufreq/interactive/sustain_load
    chmod 0660 /sys/devices/system/cpu/cpufreq/interactive/sustain_load
    chown system system /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
    chmod 0660 /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
    chown system system /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
    chmod 0660 /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
    chown system system /sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq
    chmod 0660 /sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq
    chown system system /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq
    chmod 0660 /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq
    chown system system /sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq
    chmod 0660 /sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq
    chown system system /sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq
    chmod 0660 /sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq
    chown system system /sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq
    chmod 0660 /sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq
    chown system system /sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq
    chmod 0660 /sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq

# Default Read Ahead value for sdcards
    write /sys/block/mmcblk0/queue/read_ahead_kb 2048
    write /sys/block/mmcblk1/queue/read_ahead_kb 2048

# Load WiFi driver

# BB mapping symbolic name to the logging ttyACM port
    symlink /dev/ttyACM2 /dev/log_modem

# Touch
    chown system system /dev/elan-iap
    chown system system /proc/ektf_dbg  
    start touch_fw_update

service wpa_supplicant /system/bin/wpa_supplicant \
    -iwlan0 -Dnl80211 -c/data/misc/wifi/wpa_supplicant.conf \
    -I/system/etc/wifi/wpa_supplicant_overlay.conf \
    -e/data/misc/wifi/entropy.bin -g@android:wpa_wlan0
    #   we will start as root and wpa_supplicant will switch to user wifi
    #   after setting up the capabilities required for WEXT
    #   user wifi
    #   group wifi inet keystore
    class main
    socket wpa_wlan0 dgram 660 wifi wifi
    disabled
    oneshot

service p2p_supplicant /system/bin/wpa_supplicant \
    -iwlan0 -Dnl80211 -iwlan0 -c/data/misc/wifi/wpa_supplicant.conf \
    -I/system/etc/wifi/wpa_supplicant_overlay.conf -N \
    -ip2p0 -Dnl80211 -c /data/misc/wifi/p2p_supplicant.conf \
    -I/system/etc/wifi/p2p_supplicant_overlay.conf \
    -puse_p2p_group_interface=1 -e/data/misc/wifi/entropy.bin \
    -g@android:wpa_wlan0
#   we will start as root and wpa_supplicant will switch to user wifi
#   after setting up the capabilities required for WEXT
#   user wifi
#   group wifi inet keystore
    class main
    socket wpa_wlan0 dgram 660 wifi wifi
    disabled
    oneshot

service dhcpcd_wlan0 /system/bin/dhcpcd -aABDKL
    class main
    disabled
    oneshot

service dhcpcd_p2p /system/bin/dhcpcd -aABKL
    class main
    disabled
    oneshot

service dhcpcd_eth0 /system/bin/dhcpcd -ABDKL -f/system/etc/dhcpcd/dhcpcd.conf
    class main
    disabled
    oneshot

service dhcpcd_bt-pan /system/bin/dhcpcd -ABKL
    class main
    disabled
    oneshot

service iprenew_wlan0 /system/bin/dhcpcd -n
    class main
    disabled
    oneshot

service iprenew_p2p /system/bin/dhcpcd -n
    class main
    disabled
    oneshot

service iprenew_eth0 /system/bin/dhcpcd -n
    class main
    disabled
    oneshot

service iprenew_bt-pan /system/bin/dhcpcd -n
    class main
    disabled
    oneshot

#Sensor: load calibration files.
    service sensors-config /system/bin/sensors-config
    class main
    user root
    oneshot
#Sensor load calibration files end


# bugreport is triggered by the VOLUME-DOWN and VOLUME-UP keys
service bugreport /system/bin/dumpstate -d -p -B \
        -o /data/data/com.android.shell/files/bugreports/bugreport
    class main
    disabled
    oneshot
    keycodes 115 114

# Start GPS daemon
service gps-daemon /system/bin/gps_daemon.sh
  user gps
  group system
  class late_start

# Recovery daemon: configure MSC partition
service recoveryd /system/bin/recoveryd /dev/block/platform/sdhci-tegra.3/by-name/MSC
    disabled
    class main
    user root
    group root
    oneshot

on property:sys.shutdown.requested=1recovery
    start recoveryd

# start tf_daemon service
service tf_daemon /system/bin/tf_daemon -storageDir /data/tf -d
    class main
    user root
    group shell

# may restart due to /data/tf not existing yet
on property:init.svc.tf_daemon=restarting
    mkdir /data/tf

# create filesystems if necessary
service setup_fs /system/bin/setup_fs \
        /dev/block/platform/sdhci-tegra.3/by-name/UDA \
        /dev/block/platform/sdhci-tegra.3/by-name/CAC
    class core
    user root
    group root
    oneshot

service touch_fw_update /system/bin/touch_fw_update.sh
    class main
    disabled
    user root
    group root
    oneshot


