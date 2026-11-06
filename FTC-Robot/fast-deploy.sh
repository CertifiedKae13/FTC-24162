#!/bin/bash
# Incremental build + install to the Control Hub. Gradle only recompiles/redexes what changed,
# and the APK is only pushed when it actually changed since the last deploy.
# Usage: ./fast-deploy.sh            (Wi-Fi, hub at $HUB_IP, default 192.168.43.1)
#        USB=1 ./fast-deploy.sh      (hub on USB: much faster install)
set -e
cd "$(dirname "$0")"
export ANDROID_HOME="${ANDROID_HOME:-$HOME/Library/Android/sdk}"
ADB="${ADB:-$ANDROID_HOME/platform-tools/adb}"
HUB_IP="${HUB_IP:-192.168.43.1}"
APK="TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"
STAMP=".last-deployed-apk.md5"

./gradlew --offline :TeamCode:assembleDebug -q 2>&1 | grep -v "^Note" || true
[ -f "$APK" ] || { echo "Build failed: no APK"; exit 1; }

NEW=$(md5 -q "$APK")

if [ -z "$USB" ]; then
  MYIP=$(ipconfig getifaddr en0 2>/dev/null || echo "none")
  case "$HUB_IP:$MYIP" in
    192.168.43.1:192.168.43.*) ;;   # on the Control Hub's own Wi-Fi: fine
    192.168.43.1:*)
      echo "Cannot deploy: this Mac is on $MYIP, not the Control Hub's Wi-Fi (192.168.43.x)."
      echo "Join the hub's Wi-Fi (its name is shown in the Driver Station / Program and Manage), or plug the hub in over USB and run: USB=1 ./fast-deploy.sh"
      exit 1 ;;
  esac
  OUT=$("$ADB" connect "$HUB_IP:5555" 2>&1) || true
  echo "$OUT"
  case "$OUT" in *"connected to"*) ;; *) echo "Cannot deploy: adb could not connect to $HUB_IP:5555."; exit 1 ;; esac
fi

PKG=com.qualcomm.ftcrobotcontroller
# Skip the install only if the APK is unchanged AND the app is actually still on the hub (it may have been uninstalled).
if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$NEW" ] && "$ADB" shell pm path "$PKG" 2>/dev/null | grep -q package:; then
  echo "APK unchanged since last deploy; skipping the install (the app is still restarted below)."
else
  # --fastdeploy sends only the changed parts of the APK; fall back to a normal install if unsupported.
  "$ADB" install -r --fastdeploy "$APK" 2>/dev/null || "$ADB" install -r "$APK"
  echo "$NEW" > "$STAMP"
  echo "Deployed."
fi

# Restart the Robot Controller app so it loads the new code and re-registers the OpModes.
# `adb install` leaves the app stopped, and the hub stays blue until the app is running again.
ACT=org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper   # the APK's launcher activity
"$ADB" shell am force-stop "$PKG" || true
"$ADB" shell am start -n "$PKG/$ACT" || { echo "Could not start the Robot Controller app."; exit 1; }
sleep "${START_WAIT:-8}"
if "$ADB" shell pidof "$PKG" | grep -q '[0-9]'; then
  echo "Robot Controller app restarted and running. The hub light should turn green once the Driver Station connects."
else
  echo "The Robot Controller app is NOT running after the restart: it crashed at startup. Last crash lines:"
  "$ADB" logcat -d -b crash 2>/dev/null | tail -25 || true
  "$ADB" logcat -d -s AndroidRuntime:E 2>/dev/null | tail -25 || true
  exit 1
fi
