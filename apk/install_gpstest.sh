#!/system/bin/sh
APK="/data/openpilot/apk/GPSTest.apk"
PKG="com.chartcross.gpstest"

if pm list packages | grep -q "$PKG"; then
  echo "GPS Test already installed"
else
  chmod 644 "$APK"
  pm install -r -g "$APK"
fi
