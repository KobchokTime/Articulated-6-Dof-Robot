#!/bin/bash

# แจ้งผู้ใช้ให้ถอด–เสียบ USB ใหม่ก่อนเริ่ม
zenity --question \
  --title="USB Reset Required" \
  --text="🔌 Please unplug and plug in the robot's USB cable.\n\nClick **Next** to continue restarting the service." \
  --ok-label="Next" \
  --width=400

# ถ้าผู้ใช้กด Cancel จะออกจากสคริปต์
if [ $? -ne 0 ]; then
    zenity --info --title="Cancelled" --text="❌ Operation cancelled by user."
    exit 1
fi

# ดำเนินการ reload และ restart
echo "🔁 Reloading systemd..."
sudo systemctl daemon-reload

echo "🚀 Restarting robot-arm-docker.service..."
sudo systemctl restart robot-arm-docker.service

# แสดง progress ระหว่างรอให้ระบบเสถียร
(
    echo "10" ; sleep 2
    echo "30" ; sleep 4
    echo "60" ; sleep 6
    echo "90" ; sleep 6
    echo "100" ; sleep 2
) | zenity --progress \
    --title="Waiting for Robot Service to stabilize..." \
    --text="⏳ Initializing services... Please wait..." \
    --percentage=0 \
    --auto-close \
    --width=400

# แจ้งเตือนเสร็จ
zenity --info --title="Robot Service" --text="✅ Restarted and ready!"
