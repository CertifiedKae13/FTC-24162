#!/bin/bash
# 部署 FTC 代码到 Control Hub（WiFi ADB）
# 前提：Mac 已连上机器人 WiFi（本机 IP 应为 192.168.43.x）

ADB="${ADB:-adb}"
APK="/Users/mac/Desktop/FTC-Robot/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"

# 1. 读本机 IP，确认在机器人网段
IP=$(ipconfig getifaddr en0 2>/dev/null || echo "")
if [[ "$IP" != 192.168.43.* ]]; then
  echo "⚠️  本机 IP: ${IP:-<无>}，不在机器人网段 192.168.43.x"
  echo "    先连上机器人 WiFi（24490-RC），并确认没跳回 VIL-BYOD。"
  exit 1
fi
echo "本机 IP: $IP"

# 2. 机器人 = 热点网关 192.168.43.1
echo "连接机器人 192.168.43.1 ..."
"$ADB" connect 192.168.43.1:5555
sleep 1
"$ADB" devices

# 3. 安装
echo "安装 APK ..."
"$ADB" install -r "$APK"
echo "✅ 完成"
