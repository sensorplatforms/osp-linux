cd /sys/bus/iio/devices
for f in iio:device*; do 
	cd $f;
	echo $1 > enable
	cd ..
done
