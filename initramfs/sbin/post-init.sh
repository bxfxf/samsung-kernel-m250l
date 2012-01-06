#!/sbin/busybox8 sh

sleep 5
##### Install SU #####

if /sbin/busybox8 test -f /system/xbin/su || /sbin/busybox8 test -f /system/bin/su;
then
	echo "su already exists"
else
	echo "Copying su binary"
	/sbin/busybox8 mount /system -o remount,rw
	/sbin/busybox8 rm /system/bin/su
	/sbin/busybox8 rm /system/xbin/su
	/sbin/busybox8 cp /res/misc/su /system/bin/su
	/sbin/busybox8 chown 0.0 /system/bin/su
	/sbin/busybox8 chmod 4755 /system/bin/su
	/sbin/busybox8 ln -s /system/bin/su /system/xbin/su
	/sbin/busybox8 mount /system -o remount,ro
fi
if /sbin/busybox8 test -f /system/app/Superuser.apk || /sbin/busybox8 test -f /data/app/Superuser.apk;
then
	echo "Superuser.apk already exists"
else
	echo "Copying Superuser.apk"
	/sbin/busybox8 mount /system -o remount,rw
	/sbin/busybox8 rm /system/app/Superuser.apk
	/sbin/busybox8 rm /data/app/Superuser.apk
	/sbin/busybox8 cp /res/misc/Superuser.apk /system/app/Superuser.apk
	/sbin/busybox8 chown 0.0 /system/app/Superuser.apk
	/sbin/busybox8 chmod 644 /system/app/Superuser.apk
	/sbin/busybox8 mount /system -o remount,ro
fi
