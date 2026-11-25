#!/usr/bin/env bash
# 노트북에서 실행: 이 저장소의 파일을 로버(Pi)에 올리고 서비스를 다시 시작한다. (macOS / Linux / WSL)
#   ./deploy.sh                # 기본 주소 takyeji@172.30.1.162
#   ./deploy.sh 사용자@주소
# 비밀번호는 처음 한 번만 묻는다. 로버가 달리는 중이면 멈춘 뒤에 실행할 것 (UI 서비스가 다시 시작된다).
set -euo pipefail
target="${1:-takyeji@172.30.1.162}"
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sock="$HOME/.ssh/cm-tak-deploy"
mkdir -p "$HOME/.ssh"
ssh -M -S "$sock" -o ControlPersist=120 -o ServerAliveInterval=15 -fN "$target"
trap 'ssh -S "$sock" -O exit "$target" 2>/dev/null || true' EXIT
ssh -S "$sock" "$target" 'mkdir -p ~/rover_ui/systemd ~/rover_animals/out ~/robot_ws/.runtime ~/rover_runs'
scp -q -o ControlPath="$sock" "$here"/rover_ui/rover_ui.py "$here"/rover_ui/index.html "$here"/rover_ui/rover.sh "$target":rover_ui/
scp -q -o ControlPath="$sock" "$here"/rover_ui/systemd/*.service "$target":rover_ui/systemd/
scp -q -o ControlPath="$sock" "$here"/rover_animals/rover_extra.py "$target":rover_animals/
scp -q -o ControlPath="$sock" "$here"/tools/rover_uno_drive.py "$here"/tools/uno_flash.py "$target":
# 대회장 조명에 맞춘 인식 설정은 로버에 없을 때만 올린다 (현장에서 고친 값을 덮어쓰지 않게)
for name in venue_yolo.yaml venue_yellow_road.yaml; do
    if ! ssh -S "$sock" "$target" "test -f ~/robot_ws/.runtime/$name"; then
        scp -q -o ControlPath="$sock" "$here/pi_config/$name" "$target":robot_ws/.runtime/
    fi
done
ssh -S "$sock" "$target" 'chmod +x ~/rover_ui/rover.sh && python3 -m py_compile ~/rover_ui/rover_ui.py \
    && bash ~/rover_ui/rover.sh install >/dev/null \
    && systemctl --user restart rover-ui.service rover-extra.service && sleep 5 \
    && echo "서비스: $(systemctl --user is-active rover-perception rover-extra rover-ui | tr "\n" " ")" \
    && echo "접속 번호: $(cat ~/rover_ui/pin.txt)"'
echo "끝. 브라우저에서 http://${target#*@}:8000 을 여세요."
