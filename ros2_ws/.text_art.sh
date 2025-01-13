#!/bin/bash

# ฟังก์ชันในการเปลี่ยนสีข้อความ
color_text() {
    local colors=("31" "32" "33" "34" "35" "36")
    while IFS= read -r line; do
        local color=${colors[RANDOM % ${#colors[@]}]}
        echo -e "\e[${color}m${line}\e[0m"
        sleep 0.1  # หน่วงเวลาการแสดงผลทีละบรรทัด
    done
}

# ข้อความที่จะพิมพ์ทีละบรรทัด
text='''
 _______       ______      _______       ______     ___________  
|   _  "\     /    " \    /"      \     /    " \   ("     _   ") 
(. |_)  :)   // ____  \  |:        |   // ____  \   )__/  \\__/  
|:     \/   /  /    ) :) |_____/   )  /  /    ) :)     \\_ /     
(|  _  \\   (: (____/ //   //      /  (: (____/ //      |.  |     
|: |_)  :)  \        /   |:  __   \   \        /       \:  |     
(_______/    \"_____/    |__|  \___)   \"_____/         \__|     
'''

# แสดงข้อความทีละบรรทัดพร้อมเปลี่ยนสี
echo -e "$text" | color_text

# ฟังก์ชันในการแสดงข้อความตรงกลาง
center_text() {
    local text="$1"
    local color_code="$2"
    local term_width=$(tput cols)
    local text_length=${#text}
    local padding=$(( (term_width - text_length) / 2 ))

    printf "%*s\n" $((padding + text_length)) "$(tput setaf $color_code)$text$(tput sgr0)"
}

# แสดงข้อความสีแดงตรงกลางหน้าจอ
center_text " Website: https://www.buildownrobots.com" 1