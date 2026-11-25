#!/usr/bin/env bash
# tak 로버 서비스 관리 (선·표지판 인식 + 동물·손 인식 + 웹 UI). Pi 에서 실행한다.
#   ~/rover_ui/rover.sh install    서비스 등록, 부팅 때 자동 시작, 지금 시작
#   ~/rover_ui/rover.sh start | stop | restart | status
#   ~/rover_ui/rover.sh logs [ui|extra|perception] [줄 수]
#   ~/rover_ui/rover.sh disable    부팅 때 자동 시작 끄기 (enable 로 다시 켬)
set -euo pipefail
units=(rover-perception.service rover-extra.service rover-ui.service)
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-status}" in
    install)
        mkdir -p ~/.config/systemd/user
        cp "$here"/systemd/*.service ~/.config/systemd/user/
        systemctl --user daemon-reload
        systemctl --user enable --now "${units[@]}"
        ;;
    start|stop|restart|enable|disable)
        systemctl --user "$1" "${units[@]}"
        ;;
    status)
        systemctl --user --no-pager --lines=0 status "${units[@]}" || true
        ;;
    logs)
        journalctl --no-pager -n "${3:-40}" --user-unit "rover-${2:-ui}.service"
        ;;
    *)
        echo "install | start | stop | restart | status | logs [ui|extra|perception] [줄 수] | enable | disable" >&2
        exit 2
        ;;
esac
