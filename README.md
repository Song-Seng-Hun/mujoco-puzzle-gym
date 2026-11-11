<p align="center"><b>한국어</b> · <a href="docs/README.en.md">English</a> · <a href="docs/README.ja.md">日本語</a></p>

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/banner-dark.svg">
    <img src="assets/banner-light.svg" width="640" alt="픽셀아트 배너. Claude가 터미널에 room.yaml을 한 줄씩 적을 때마다 오른쪽 방 평면도에 칸막이에 틈이 난 벽, 노란 발판, 갈색 문, 파란 골 원, 발판과 문을 잇는 신호선이 반짝이며 나타납니다. 점검 줄에 초록 체크가 뜨면 작은 오리 로봇이 발판으로 걸어가 밟고, 발판이 초록으로 바뀌며 문이 미끄러져 열립니다. 오리가 문을 지나 골에 도착하면 반짝임과 함께 터미널에 success 0.99가 찍히고 Claude가 고개를 끄덕입니다. 오리는 Claude가 아니라 학습된 정책으로 움직입니다.">
  </picture>
</p>

# mujoco-puzzle-gym — 에이전트가 만드는 퍼즐 실험장

AI 코딩 에이전트가 MuJoCo 물리 시뮬레이션 안에 퍼즐 방을 직접 짓고, 그 방에서 로봇을 학습시키는 프레임워크입니다. 로봇을 움직이는 것은 그 방에서 강화학습한 정책이고, 에이전트가 로봇을 직접 조종하지는 않습니다. 퍼즐은 정해진 장면이 아니라 프로그램입니다. 발판, 문, 신호, 논리를 이어 상태가 바뀌는 방을 YAML로 짓고, 모자란 부품은 파이썬 클래스로 하나 더 등록하면 됩니다. 에이전트는 방을 짓고, 풀 수 있는지 점검하고, 짧게 학습시키고, 실패하면 이유를 진단받아 고치는 과정을 스스로 반복합니다.

<p align="center"><img src="assets/demo.gif" width="480" alt="학습된 정책으로 움직이는 마이크로덕 시뮬레이션. 벽으로 나뉜 방의 왼쪽에서 출발한 오리 로봇이 노란 발판으로 걸어가 밟자 발판이 초록색으로 바뀌고 가운데 칸막이의 문이 열립니다. 오리는 열린 문을 지나 오른쪽의 파란 골 원에 도착합니다."></p>
<p align="center"><sub>15분 학습한 정책이 처음 보는 배치에서 발판을 밟아 문을 열고 골에 가는 모습입니다. 로봇 모델: Pollen Robotics 마이크로덕(CC BY-NC-SA 4.0).</sub></p>

## 이게 뭔가요

- **퍼즐을 프로그램처럼 짓습니다.** 방 하나는 `room.yaml` 하나입니다. 벽, 부품 위치, 에피소드마다 위치를 무작위로 바꿀 범위, 부품 사이 신호 연결을 적습니다. 새 장치는 파이썬 클래스 하나로 더합니다.
- **풀 수 있는지 먼저 봅니다.** 학습에 시간을 쓰기 전에, 풀이 순서가 있는지, 물리 시뮬레이션이 안정적인지, 정답을 아는 스크립트로 실제로 풀리는지를 몇 초 만에 확인합니다.
- **학습하고, 왜 안 되는지 알려 줍니다.** 실패하면 원인(환경 버그, 넘어짐, 탐색 부족, 보상 꼼수)과 근거, 고칠 방향을 줍니다. AI 에이전트가 사람 없이 이 과정을 반복할 수 있습니다.
- **보상도 설계합니다.** 보상은 항목과 가중치로 `task.yaml`에 적습니다(진행, 단계, 성공, 넘어짐 등). 진단이 "보상 꼼수"나 "탐색 부족"을 짚으면 에이전트가 가중치를 고치거나 새 보상 항목을 파이썬 함수 하나로 등록해 다시 학습합니다.

에이전트는 이 저장소에 들어 있지 않습니다. 평소 쓰는 AI 코딩 에이전트(Claude Code 등)에게 YAML과 파이썬 파일 수정과 `mpg` 명령 실행을 맡기면 됩니다. 사람도 같은 명령으로 직접 쓸 수 있습니다.

처음 들어 있는 로봇은 Pollen Robotics의 [마이크로덕](https://github.com/pollen-robotics/microduck_rl)입니다. 공개된 걷기 정책을 그대로 쓰고, 그 위에서 "어디로 얼마나 빨리 걸을지"만 새로 학습합니다.

## 주요 기능

- **부품은 여러 속성의 묶음입니다.** 부품 하나는 종류(`type`)와 속성(위치, 크기, 한 번 밟으면 켜진 채로 남는지, 열리는 방향, 에피소드마다 위치를 무작위로 바꿀 범위 등)으로 이루어집니다. 부품 목록과 각 속성은 [부품 문서](docs/parts.md)에 따로 정리했습니다.
- **에이전트가 부품을 조립해 퍼즐을 설계합니다.** 부품을 골라 놓고 신호선(`links`)으로 이으면 퍼즐이 됩니다. 발판 → 문 → 골 같은 단순한 방부터 발판 여러 개가 모두 켜져야 열리는 문(`require: all`)처럼 조건을 겹친 방까지 같은 방식으로 짓고, 모자란 부품은 직접 만들어 끼웁니다.
- **랜덤화를 지원합니다.** 지금은 부품 위치와 출발 위치·방향을 정한 범위 안에서 매 에피소드 조금씩 다르게 배치해, 정책이 배치를 외우지 않고 퍼즐을 풀게 합니다. 구역 안 아무 곳에 놓기, 후보 중 하나 고르기, 방 구조 자체를 바꾸는 랜덤화는 마일스톤 2에서 들어옵니다.
- **기준(베이스라인) 환경이 들어 있습니다.** 예제 방, 검증한 PPO 학습 설정, 미리 학습한 정책(`mpg demo`), 같은 시드에서 무작위 정책과 비교(`mpg eval --policy random`), 일부러 망가뜨린 설정 모음([`examples/broken/`](examples/broken/))이 있어서 새 방이나 새 알고리즘을 견줄 출발점이 됩니다.
- **로봇을 바꿀 수 있습니다.** 퍼즐 쪽 코드는 어떤 로봇인지 모르고, 몸 어댑터가 이동 명령을 받아 로봇을 움직입니다. 몸 어댑터를 새로 만들어 붙이면 다른 로봇도 같은 방에서 학습합니다(방법은 [사용 안내](docs/guide.md#몸-어댑터)). 지금 들어 있는 로봇은 마이크로덕 하나입니다.

## 설치

macOS나 리눅스에서 아래 한 줄이면 됩니다. 필요한 것(uv, Python 3.12, 의존성, 로봇 모델과 걷기 정책 약 22 MB)을 모두 설치하고, 예제 방으로 잘 깔렸는지 확인까지 합니다.

```bash
git clone https://github.com/Song-Seng-Hun/mujoco-puzzle-gym.git && cd mujoco-puzzle-gym && ./install.sh
```

macOS(Apple Silicon)에서 확인했습니다. 윈도우는 아직 시험하지 않았습니다.

## 해 보기

학습된 오리가 예제 방을 푸는 영상을 바로 만듭니다(`demo.mp4`).

```bash
uv run mpg demo
```

직접 학습시켜 봅니다. 12코어 환경에서 15분 학습했을 때 처음 보는 배치의 99%를 풀었습니다([학습 기록](docs/training-log.md)).

```bash
uv run mpg train examples/task_ppo.yaml --minutes 15 --id first
```

```bash
uv run mpg render runs/first
```

## 내 퍼즐 방 만들기

예제 방 [`examples/switch_door.yaml`](examples/switch_door.yaml)을 복사해 고치고, 풀 수 있는지 점검한 뒤 학습시키면 됩니다. 발판을 밟으면 문이 열리고 골에 닿으면 성공인 방입니다.

```yaml
name: switch_door
size: [2.4, 1.8]                  # 방 안쪽 크기 [m], 원점이 가운데
wall: {thickness: 0.06, height: 0.25}
walls:                            # x = 0.3의 칸막이, 가운데 0.5 m가 뚫려 있음
  - {from: [0.3, -0.9], to: [0.3, -0.28]}
  - {from: [0.3, 0.28], to: [0.3, 0.9]}
spawn: {pos: [-0.75, 0.0], yaw: 0.0, random: [0.2, 0.35, 3.14]}
parts:
  - {name: switch_a, type: switch, pos: [-0.25, 0.5], size: [0.3, 0.3], random: [0.15, 0.1]}
  - {name: door_b, type: door, pos: [0.3, 0.0], width: 0.5, axis: y}
  - {name: goal, type: goal, pos: [0.85, 0.0], radius: 0.15, random: [0.1, 0.45]}
links:
  - {from: switch_a, to: door_b}  # switch_a가 켜지면 door_b가 열림
```

```bash
uv run mpg check my_room.yaml
```

이 방으로 학습하려면 [`examples/task_ppo.yaml`](examples/task_ppo.yaml)도 `my_task.yaml`로 복사해 `room: my_room.yaml`로 바꾸고 `uv run mpg train my_task.yaml --minutes 15 --id my-room`을 실행합니다. 방 경로는 학습 설정 파일이 있는 폴더를 기준으로 찾습니다.

명령 전체, 학습 설정, 새 부품·보상 넣기, 몸 바꾸기는 [사용 안내](docs/guide.md)에 있습니다.

## 진행 상황

- [x] **마일스톤 1**: 발판 → 문 → 골 퍼즐을 짓고, 점검하고, 학습하고, 진단하고, 평가하는 전체 과정. 내장 부품은 발판, 문, 골 세 가지이고, 다른 장치는 플러그인으로 넣을 수 있습니다.
- [ ] **마일스톤 2**: 버튼, 센서, 논리 게이트, 카운터, 코인, 워프, 함정, 부리로 집기, 체크포인트와 단계별 학습 같은 부품 카탈로그 (진행 중)
- [ ] **마일스톤 3**: 팔, 바퀴 로봇처럼 다른 몸을 위한 퍼즐 부품

## 한계

- 상위 정책(어디로 걸을지 정하는 정책)은 퍼즐 상태를 그대로 읽는 특권 관측을 씁니다. 실물 센서(카메라, 거리 센서)만으로 푸는 정책은 아직 없습니다.
- 상위 정책은 처음 보는 배치로는 일반화했지만, 학습한 방은 순서가 늘 같고(스위치 → 문 → 골) 보상도 정답 순서 쪽을 가리킵니다. 순서를 스스로 추론하는지는 아직 시험하지 않았습니다.
- 걷기 정책은 공식 배포된 것을 그대로 고정해 씁니다. 이 정책은 뒤로 걷지 못하고, 앞으로는 약 0.12–0.22 m/s로 걷습니다. 방 크기와 시간 제한은 이 속도에 맞춰야 합니다.
- 시뮬레이션만 다룹니다. 실물 오리로 옮기는 일(sim2real)은 범위 밖입니다.
- macOS(Apple Silicon, CPU MuJoCo)에서 확인했습니다. 리눅스 렌더링은 아직 확인하지 않았습니다.

## AI와 함께 만든 프로젝트

이 저장소의 기획, 설계 문서, 코드, 테스트, 학습 실험은 제작자와 Claude(Anthropic)가 함께 만들었습니다. 설계 문서는 [docs/specs/](docs/specs/)에 있습니다.

## 출처와 라이선스

- 이 저장소의 코드: [Apache-2.0](LICENSE).
- 로봇 모델과 메시: © Pollen Robotics, [microduck_rl](https://github.com/pollen-robotics/microduck_rl) 고정 커밋 `0daab160`. MJCF는 Apache-2.0, 3D 메시(`*.stl`)는 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)(비상업, 동일 조건)입니다. 메시는 이 저장소에 들어 있지 않고, `mpg fetch-assets`가 원본 그대로 받아 옵니다. MJCF 원본은 고치지 않고, 실행 중 메모리에서만 고칩니다.
- 걷기 정책: [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies)의 `velstand.onnx`, Apache-2.0.
- 데모 영상과 그림에는 CC BY-NC-SA 4.0 메시가 쓰였습니다.
- 이 프로젝트는 비공식 프로젝트이며 Pollen Robotics와 관계가 없습니다.

## 더 보기

- [사용 안내](docs/guide.md): 동작 흐름, 명령, 학습 설정, 확장, 몸 어댑터
- [부품 목록과 속성](docs/parts.md): 지금 들어 있는 부품과 각 속성
- [학습 기록](docs/training-log.md): 학습 결과와 일부러 망가뜨린 설정에 대한 진단
- [속도 측정](docs/bench.md)
- [설계 문서](docs/specs/)
