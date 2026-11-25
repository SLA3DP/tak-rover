# tak 로버 제어 UI

메카넘 4륜 로버 "tak" (Raspberry Pi 4 + Arduino Uno)를 웹 화면에서 조종하는 코드입니다.
자율 주행(선 추종 + 표지판·동물 규칙), 손 추적, 수동 조종을 한 화면에서 합니다.

로버 안에서 서버가 돌기 때문에, **조종만 할 때는 노트북에 아무것도 설치하지 않습니다.**

## 1. 다른 노트북에서 조종만 할 때

1. 노트북을 로버와 같은 와이파이(캠퍼스 `robot` 또는 `robot-5G`)에 연결합니다.
2. 로버 전원을 켜고 1분쯤 기다립니다. 서비스가 스스로 뜹니다.
3. 브라우저에서 `http://172.30.1.162:8000` 을 엽니다.
4. 접속 번호 4자리를 넣습니다. 번호는 로버의 `~/rover_ui/pin.txt` 에 있습니다 (브라우저마다 한 번만 입력).

로버 주소가 바뀌었으면 로버에서 `hostname -I` 로 확인합니다.

## 2. 다른 노트북에서 코드를 고칠 때

```bash
git clone https://github.com/SLA3DP/tak-rover.git
cd tak-rover
```

고친 것을 GitHub에 다시 올리려면 저장소에 쓸 수 있는 계정으로 로그인해야 합니다 (`gh auth login`).

파일을 고친 뒤 로버에 올립니다. 로버 비밀번호를 한 번 묻습니다.

```bash
./deploy.sh
```

`deploy.sh` 는 macOS, Linux, WSL용입니다. Windows 명령 프롬프트에서는 아래처럼 직접 올립니다 (명령마다 비밀번호를 묻습니다).

```bash
scp rover_ui/rover_ui.py rover_ui/index.html rover_ui/rover.sh takyeji@172.30.1.162:rover_ui/
scp rover_animals/rover_extra.py takyeji@172.30.1.162:rover_animals/
ssh takyeji@172.30.1.162 "systemctl --user restart rover-ui.service rover-extra.service"
```

화면(`index.html`)만 고쳤으면 서비스를 다시 시작하지 않아도 됩니다. 새로 고침만 하면 됩니다.

## 3. Claude Code로 이어서 할 때

복제한 `tak-rover` 폴더에서 Claude Code를 시작합니다. 지금까지의 배선, 규칙, 확인된 것과 안 된 것이 `CLAUDE.md` 에 적혀 있어서 그대로 이어 갈 수 있습니다.
Claude는 비밀번호를 입력하지 못하므로, 로버 접속 비밀번호는 터미널에 직접 입력해야 합니다.

## 화면 사용법

| 탭 | 동작 |
|---|---|
| 자율 주행 | 노란 선을 따라갑니다. 선이 안 보이면 벽까지 직진한 뒤 90도 돕니다. STOP이 가까우면 멈추고, 다른 표지판(파란 유턴 제외)이나 동물이 길에 있으면 돌아서 다른 선을 찾습니다. |
| 손 추적 | 손바닥을 보여 주면 다가와 손 앞에서 멈춥니다. 파란 유턴 표지판은 180도 회전, STOP·유턴 금지는 정지입니다. |
| 수동 조종 | 누르는 동안만 움직입니다 (W A S D, 게걸음 Q E). 기록해 둔 주행 재생도 여기 있습니다. |

- 오른쪽 위 `정지` 또는 스페이스 바는 언제나 멈춥니다.
- 책상 위에서 시험할 때는 아래쪽 `모터 출력`을 `끔`으로 두면 계산만 하고 바퀴는 돌지 않습니다.
- 바꾼 설정값은 로버에 저장됩니다 (`~/rover_ui/params.json`).

### 트랙에서 맞춰야 하는 값

| 증상 | 조정 (자율 주행 탭) |
|---|---|
| 벽에서 90도보다 덜 / 더 돈다 | `90도 도는 시간`을 늘림 / 줄임 |
| 벽에 닿기 전에 돈다 | `벽이 화면을 가린 뒤 더 가는 시간`을 늘림 |
| 표지판에 너무 늦게 / 일찍 반응한다 | `표지판을 가깝다고 보는 크기`를 줄임 / 늘림 |
| 유턴 표지판에서 180도가 안 맞는다 | 손 추적 탭의 `180도 도는 시간` |

## 로버 안의 서비스

```bash
~/rover_ui/rover.sh status                 # 상태
~/rover_ui/rover.sh restart                # 다시 시작
~/rover_ui/rover.sh stop                   # 모두 끄기 (start 로 다시 켬)
~/rover_ui/rover.sh logs ui                # 기록 보기 (ui | extra | perception)
~/rover_ui/rover.sh disable                # 전원을 켤 때 자동 시작하지 않게
```

| 서비스 | 하는 일 |
|---|---|
| `rover-perception` | 카메라 + 선·표지판 인식 (로버에 원래 있던 `~/robot_ws` 코드, 도커) |
| `rover-extra` | 동물·손바닥 인식 (`rover_animals/rover_extra.py`, 도커) |
| `rover-ui` | 웹 화면 + 주행 제어 + 아두이노 통신 (`rover_ui/rover_ui.py`, 포트 8000) |

## 폴더

| 경로 | 내용 |
|---|---|
| `rover_ui/` | 제어 서버, 화면, 서비스 파일, 관리 스크립트 |
| `rover_animals/` | 동물·손바닥 인식기 |
| `firmware/rover_uno_l298/` | 아두이노 우노 모터 펌웨어 (L298N 두 개) |
| `tools/` | `uno_flash.py` (우노에 펌웨어 올리기), `rover_uno_drive.py` (명령줄 주행 도구) |
| `pi_config/` | 대회장 조명에 맞춘 인식 설정 (로버의 `~/robot_ws/.runtime/` 에 둠) |
| `runs/` | 수동 조종으로 기록한 코스 주행 |

이 저장소에 없는 것: 로버의 `~/robot_ws` (인식 코드와 표지판 모델), 도커 이미지 `rover-jazzy:ncnn`, 동물 인식 모델 `~/rover_animals/yolo11n_coco.pt`. 모두 로버에 이미 들어 있습니다.

## 자주 생기는 문제

| 증상 | 확인할 것 |
|---|---|
| 화면이 안 열린다 | 같은 와이파이인지, 로버 주소가 맞는지, `rover.sh status` |
| 카메라 화면이 멈춘다 | 8초 넘게 멈추면 스스로 인식 서비스를 다시 시작합니다. 계속 그러면 카메라 케이블과 배터리 전압을 확인 |
| 시작을 눌러도 안 움직인다 | 위쪽 `아두이노` 표시, `모터 출력`이 `켬`인지, 상태 줄의 이유 |
| `rover_uno_drive.py` 가 "busy" | UI 서비스가 아두이노 포트를 잡고 있습니다. `systemctl --user stop rover-ui` 뒤에 사용 |
