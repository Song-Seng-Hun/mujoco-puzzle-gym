# 부품과 프리셋 안내

[사용 안내로 돌아가기](guide.md)

퍼즐 방은 부품을 조립해 만듭니다. 이 문서는 부품이 신호로 이어지는 방식, 프리셋으로 방을 짜는 순서, 점검과 단계별 학습, 그리고 모든 부품의 값 표를 다룹니다. 값 표는 `mpg parts --markdown`이 만든 것이고, 나머지는 사람이 쓴 설명입니다.

## 부품, 프리셋, 조립 프리셋

- **부품**은 `room.yaml`의 `parts:` 한 줄입니다. 종류(`type`)와 값들로 이루어집니다. 종류마다 받는 값과 단위, 기본값, 허용 범위가 정해져 있고(값 스키마), 범위 밖 값은 `mpg build`와 `mpg check`가 이유와 범위를 들어 거절합니다.
- **부품 프리셋**은 부품 종류 하나와 기본값 묶음입니다. 코드는 없습니다. `src/mujoco_puzzle_gym/presets/parts/<분류>/<이름>.yaml`에 있고, 분류는 `input`(버튼·센서), `logic`(게이트·카운터·트랜시버), `device`(문·워프·골대·플랫폼·기둥·회전판·컨베이어·스프링·설정 변경기), `floor`, `trap`, `object`, `beak`, `npc`(스스로 움직이는 로봇)입니다.
- **조립 프리셋**은 여러 부품을 묶은 것, 또는 방 하나 전체입니다. `presets/puzzles/<주제>/<이름>.yaml`에 있습니다.

`mpg parts --json`은 등록된 부품과 값 스키마, 프리셋 목록을 냅니다. 직접 만든 부품까지 보려면 `--plugin my_parts`를 붙입니다.

## 프리셋으로 방 짜기

부품 줄에 `preset:`을 쓰면 프리셋의 종류와 값을 가져오고, 그 줄에 적은 값이 프리셋 값을 덮어씁니다.

```yaml
parts:
  - {name: button_a, preset: input/floor_oneshot, pos: [-0.35, 0.55]}
  - {name: door, preset: device/sliding_door, pos: [0.4, 0.0], width: 0.4, in: [button_a]}
```

방 맨 위에 `preset:`(이름 하나나 목록)을 쓰면 조립 프리셋을 깔고 그 위에 방의 값을 얹습니다.

```yaml
preset: puzzles/coins/coin_lock     # 코인 3개, 코인 잠금 lock, 문 lock_door
size: [2.4, 1.8]
spawn: {pos: [-0.9, -0.7]}
parts:
  - {name: lock, need: 2}                         # 같은 이름: 값만 바꿈
  - {name: goal, type: goal, pos: [0.85, 0.0]}    # 새 이름: 더함
```

합치는 규칙은 다음과 같습니다. 목록 순서대로 합치고 방 파일이 마지막입니다.

- 맨 위 값(`size`, `spawn`, `demo` 등)은 뒤의 것이 바꿉니다. 프리셋의 `name`은 물려받지 않습니다.
- `parts`는 `name`으로 맞춥니다. 같은 이름의 줄에 `type`이나 `preset`이 있으면 부품을 통째로 바꾸고, 없으면 적은 값만 바꿉니다. 새 이름은 뒤에 더합니다.
- `links`는 이어 붙입니다.

덮어쓴 값을 포함한 최종 값은 학습할 때 `runs/<id>/task.resolved.yaml`의 `room_parts`에 남습니다.

방을 짜는 순서:

1. `mpg parts --markdown`이나 아래 표에서 쓸 프리셋을 고릅니다.
2. 벽과 출발점을 정하고, 부품 줄마다 `preset`과 `pos`, 바꿀 값을 적습니다.
3. 신호를 잇습니다. 받는 부품의 `in:`에 내는 부품의 채널 이름을 적습니다.
4. `mpg build`로 그림을 보고, `mpg check`로 점검합니다.

## 신호 버스

부품은 이름 붙은 채널로 신호를 주고받습니다.

- 채널 값은 실수입니다. 켜짐·꺼짐은 1과 0이고, 0.5보다 크면 켜짐으로 읽습니다. 카운터는 세는 수를 그대로 싣습니다.
- 부품은 `in`의 채널을 읽고 `out`의 채널에 씁니다. `out`을 적지 않으면 버튼·게이트처럼 신호를 내는 부품은 자기 이름의 채널에 씁니다. 카운터는 `<이름>.count`, `<이름>.reached`처럼 점 붙은 채널도 냅니다.
- `enable: <채널>`이 꺼져 있으면 그 부품의 출력은 멈춥니다(전원).
- 몸 스텝마다 입력 부품 → 논리 부품 → 장치 부품 순서로 한 번씩 갱신해서, 버튼에서 문까지 같은 스텝에 전해집니다.
- 상태 없는 부품만으로 도는 고리(NOT → NOT)는 거절합니다. 지연·래치처럼 상태를 갖는 부품을 낀 고리는 됩니다.
- M1의 `links: [{from, to}]`는 받는 부품의 `in`에 내는 부품의 채널을 더하는 것과 같습니다.

## 식 게이트 문법

`type: gate`는 다음 중 하나로 출력을 계산합니다.

- `expr`: 채널을 읽는 식. 예: `a and not b`, `delay(btn, 1.5)`, `latch(pulse(a), reset_btn)`, `count(lap) >= 3 or timer(plate) > 2`.
- `op`와 `in`: 채널 이름을 식에 쓰지 않는 형태라 프리셋으로 씁니다(`logic/and` 등). `and`, `or`, `xor`는 모든 입력, `not`, `delay`, `hold`, `pulse`, `timer`, `count`는 입력 하나, `latch`는 `[set, reset]`.
- `sequence: [a, b, c]`: 순서 잠금. 순서가 틀리면 `<이름>.wrong`이 한 스텝 켜집니다.
- `logic: <이름>`: `register_logic`으로 등록한 파이썬 함수.

식에 쓸 수 있는 것: 채널 이름(점이 든 `laps.count`도 채널 하나), 숫자, `and or not xor`, 비교(`< <= > >= == !=`), 괄호, 함수 `delay(x, s)`, `hold(x, s)`, `pulse(x)`, `latch(set, reset)`, `count(x)`, `timer(x)`. 산술, 속성 접근, 다른 함수 호출, 문자열은 방을 읽을 때 거절합니다. 식은 `ast`로 읽어 허용 목록을 확인하고 이 패키지가 직접 계산하므로, 남이 만든 방 파일을 열어도 코드가 실행되지 않습니다.

## 움직이는 장치와 설정 변경기

- 플랫폼(`platform`, 프리셋 `device/moving_platform`, `device/elevator`): `pos`에서 잰 `path: [[dx, dy, z], …]`(z는 윗면 높이)를 `speed`로 따라갑니다. `mode`는 `loop`, `pingpong`, `once`이고, `run`은 `always`, `while_signal`(신호가 켜진 동안), `on_signal`(켜지는 순간마다 한 번 운행)입니다. 미끄럼 관절과 위치 구동기로 움직여서 위에 놓인 물체를 함께 나릅니다.
- 승강 기둥(`pillar`), 회전판(`turntable`), 컨베이어(`conveyor`): 신호가 켜진 동안 솟고, 돌고, 띠 위의 몸과 물체를 띠 속도로 옮깁니다. 컨베이어 띠는 바닥에 그린 띠라서 마찰은 바닥과 같습니다.
- 스프링(`spring`, 프리셋 `device/spring_pad`): 패드 위의 대상에 속도 `velocity` [vx, vy, vz]를 줍니다. 프리셋은 물체만 쏩니다(`trigger: object`).
- 설정 변경기(`setter`): 신호가 켜지는 순간 `target: <부품>.<값>`을 `value`로 바꾸고, `revert: true`면 꺼질 때 되돌립니다. 아래 값 표에서 "실행 중 변경"이 "예"인 값만 바꿀 수 있습니다. 문 폭 같은 크기 값은 방을 읽을 때 거절합니다.
- 값 연결 `bind`: 모든 부품이 받는 값입니다. `bind: {speed: {from: lever.angle, map: [-1, 1, 0, 0.3]}}`처럼 실행 중 변경할 수 있는 수 값을 채널에 직선으로 잇고, 매 몸 스텝 값의 범위로 잘라 적용합니다. 같은 값을 `bind`와 설정 변경기가 함께 바꾸면 거절합니다.
- 풀이 순서와 해답은 플랫폼과 회전판 칸, 솟을 기둥 칸을 피해 갑니다. 몸이 그 위에 탄 채 버티는지는 걷기 정책이 배운 범위 밖이라 기록만 합니다. 2026-11-12 기록: 플랫폼 0.1·0.3 m/s와 띠 0.1·0.2 m/s에서 3초 동안 넘어지지 않았습니다.

## 함정

함정(`trap` 분류)은 몸을 밀거나 넘어뜨리거나 떨어뜨립니다. 모두 `in`의 신호가 켜진 동안만 움직이고(`in`이 없으면 늘), 세기는 `power` 하나로 정합니다. `enabled: 0`이면 꺼집니다(`in`이 없는 구덩이는 늘 열려 있어 끌 수 없고, `enable`·`enabled`를 쓰면 거절합니다. 켜고 끄려면 `in`을 주세요). `power`와 `enabled`는 실행 중 바꿀 수 있어서 설정 변경기, `bind`, 난이도 단계(아래)로 조절합니다.

| 함정 | 프리셋 | `power` |
|---|---|---|
| 바람 `wind` | `trap/crosswind` | 구역 안의 몸과 물체를 미는 힘 [N], 방향은 `yaw` |
| 밀대 `pusher` | `trap/pusher` | 판이 집에서 나왔다 들어가는 속도 [m/s] |
| 회전 막대 `spinner` | `trap/spinning_bar` | 바닥 높이 막대의 회전 속도 [deg/s] |
| 진자 `pendulum` | `trap/pendulum` | 흔들리는 폭 [deg], 주기는 `period` |
| 구덩이 `pit` | `trap/pit` | 없음. 빠지면 `on_fall: fail`(에피소드 실패) 또는 `reset`(출발점이나 체크포인트로). `in`이 있으면 꺼진 동안 뚜껑이 덮여 있습니다 |
| 대포 `cannon` | `trap/cannon`, `trap/cannon_turret` | 포탄을 쏘는 속도 [m/s] |
| 미끄러운 바닥, 끈적한 바닥 | `trap/slippery_floor`, `trap/sticky_floor` | 바닥 칸(`floor_tile`)의 마찰 |

- 풀이 순서와 해답은 구덩이, 회전 막대와 진자가 쓸고 가는 자리, 밀대가 나오는 자리를 구멍으로 보고 돌아갑니다.
- 대포는 `period`초마다 `burst`발을 쏩니다(`fire: edge`면 신호가 켜지는 순간마다). 겨누기 `aim`은 `fixed`(`yaw` + `aim_yaw`, `aim_pitch`), `at_body`(지금 몸 위치), `lead`(몸이 갈 곳), `signal`(`aim_yaw`, `aim_pitch`를 `bind`로 채널에 이음)입니다. `spread`는 겨눈 방향 둘레 원뿔의 반각입니다. `<이름>.fired`는 쏜 순간 한 스텝 켜지고, `<이름>.hits`는 몸에 맞은 수입니다.
- 포탄은 미리 만든 `pool`개(기본 10)를 돌려 씁니다. `lifetime`초가 지나거나 모두 나가 있으면 가장 오래된 것을 거둬 다시 쏩니다. 저장본에 날아가는 포탄도 담깁니다.
- 포탄은 버튼, 압력판, 구역을 누르지 않습니다. 맞히는 것은 과녁(`input/target`)뿐입니다. 과녁은 `min_speed` 이상으로 맞으면 `<이름>.hits`를 올리고 켜집니다(`mode: latch`는 켜진 채, `pulse`는 한 스텝).
- 포탑(`trap/cannon_turret`)은 레버 같은 신호로 겨누고 신호가 켜질 때 한 발 쏩니다. `mpg check`는 `bind`가 낼 수 있는 겨눔 범위 전체에서 포물선을 계산해, 한 방향이라도 과녁 판에 맞으면 통과시킵니다. `spread`가 있으면 원뿔 전체가 판에 들어와야 맞힌 것으로 칩니다.
- 넘어뜨리는 세기 기록(2026-11-12, 서 있는 몸, 0.4 m 거리, 옆과 앞에서 각 3번): 포탄 질량 × 속도가 0.5 N·s 이하면 한 번도 넘어지지 않았고, 0.6 N·s(0.1 kg, 6 m/s, 앞)부터 넘어지기 시작해 0.8 N·s(0.2 kg, 4 m/s)에서 6번 중 4번, 1.0 N·s(0.2 kg, 5 m/s)에서 6번 모두 넘어졌습니다. 그래서 기본값은 0.2 kg, 5 m/s입니다. 걷는 중에는 더 약해서, 0.4 N·s 포탄을 2.5초마다 맞은 해답이 20번 중 6번 넘어졌습니다.
- 포탄 질량(`shell_mass`)은 방을 지을 때만 정합니다. MuJoCo가 물체의 질량 행렬을 미리 계산해 두어서, 실행 중에 바꾸면 움직임이 틀어집니다.

## NPC 로봇

`npc` 분류는 스스로 바닥을 미끄러져 다니는 작은 로봇입니다(`type: npc`). 평면 관절 3개(x, y 미끄럼, z 회전)와 속도 구동기로 움직이고, 질량이 있어 몸·물체·벽과 부딪힙니다. 바닥 버튼과 구역은 로봇 중심을 물체처럼 봅니다(`trigger: npc`나 `any`). 위아래 관절이 없어 바닥을 누르지 않으므로, 압력판은 중심이 판 위에 있는 동안 로봇의 `mass`를 셉니다. 위에 마커가 붙습니다.

| 행동 `behavior` | 프리셋 | 하는 일 |
|---|---|---|
| `flee` | `npc/flee` | 몸이 `sense` 안에 오면 반대쪽으로 도망칩니다. 막히면 벽을 따라 미끄러집니다 |
| `follow` | `npc/follow` | `sense` 안의 몸 쪽으로 와서 `keep`만큼 떨어져 멈춥니다 |
| `lure` | `npc/lure` | 몸이 `bait` 물체(큐브)를 물고 있으면 그 물체 쪽으로 와서 `keep`만큼 떨어져 멈춥니다 |
| `patrol` | `npc/patrol` | `path`(`pos`에서 잰 점들, 한 바퀴)를 돕니다. 몸이 앞을 막으면 멈춰 기다립니다 |
| `guard` | `npc/guard` | 자리(`pos`)를 지키다가 몸이 `sense + reach` 안에 오면 `reach`까지 나와 막아섭니다 |
| `shy_gaze` | `npc/shy_gaze` | 몸이 마커를 쳐다보면(시선 센서, 방 `gaze:`) 멈추고, 안 보면 몸 쪽으로 다가옵니다. `invert: true`면 반대입니다 |
| `seek_button` | `npc/seek_button` | `in`이 켜지면 `button` 부품 위로 가서 섭니다 |
| `sleepy` | `npc/sleepy` | `awake`초 동안 `path`를 돌고 `asleep`초 동안 멈추기를 되풀이합니다 |

- 공통 값: `speed`(최고 속도, 기본 0.08 m/s), `accel`, `sense`(감지 반경), `size`, `mass`, `leash: [x0, y0, x1, y1]`(로봇 중심이 벗어나지 않는 상자, 방 좌표). `enable`이 꺼지면 그 자리에 멈춥니다. `speed`, `accel`, `sense`, `keep`, `reach`, `awake`, `asleep`은 실행 중 바꿀 수 있습니다.
- 벽 피하기: 가려는 방향과 그 ±35°, ±70°로 광선 5개를 쏴서 막히지 않은 첫 방향으로 갑니다. 다섯 다 막히면 멈춥니다. 규칙에 난수가 없어서 같은 시드면 같은 궤적입니다.
- 채널: `<이름>.moving`(움직이는 동안 1), `<이름>.on`(올라선 버튼의 마커 번호, 없으면 -1). 관측 항목 `npcs`는 로봇마다 상대 위치, 속도, 움직임, 반응 중인지(6칸)입니다.
- `speed`가 몸의 걷는 속도(약 0.16 m/s) 이상이면 `mpg check`의 `parts` 단계가 경고합니다(실패는 아님). 몸이 따라잡지 못하기 때문입니다.
- `seek_button` 로봇의 `button`(로봇이 누를 수 있는 버튼, 구역, 레이저, 압력판)은 풀이 순서에서 로봇의 `in`이 켜지면 켜진 것으로 칩니다(`in`이 없으면 처음부터). 몸은 그 위로 가지 않고, 로봇이 가서 누릅니다. 그래서 "스위치 A → 로봇(`in: [A]`) → 판 → 문" 방은 A만 누르면 풀립니다.
- 그 밖에 로봇만 누르는 입력(`trigger: npc`, `objects` 목록 없음)은 몰 수 있는 로봇(`flee`, `follow`, 미끼가 큐브인 `lure`)의 `leash` 안(`leash`가 없으면 어디든)에 있으면 풀이 순서에서 발판으로 칩니다. 해답의 `demo:`에 `herd <로봇> <부품>`(또는 `herd <로봇> <x> <y>`)을 쓰면 `flee` 로봇은 반대편에서 몰고, `follow` 로봇은 앞장서 끌고 갑니다. `lure` 로봇은 먼저 `grab <미끼>`로 미끼를 문 뒤 `herd`를 쓰면, 문 미끼로 끌고 갑니다.

## 조립 예

모두 `mpg check`의 해답 단계를 20번 중 18번 이상 통과합니다.

| 방 | 하는 일 | 쓰는 프리셋 |
|---|---|---|
| [`examples/rooms/logic_lock.yaml`](../examples/rooms/logic_lock.yaml) | 버튼 두 개를 모두 밟아야 AND 게이트가 문을 엽니다 | `input/floor_oneshot`, `logic/and`, `device/sliding_door` |
| [`examples/rooms/doors.yaml`](../examples/rooms/doors.yaml) | 미닫이 문, 여닫이 문, 같은 신호로 열리는 트랩도어(해답은 구멍을 돌아감) | `device/sliding_door`, `device/swing_door`, `device/trapdoor` |
| [`examples/rooms/four_flags.yaml`](../examples/rooms/four_flags.yaml) | 깃발 1→2→3→4를 차례로 지나 골로. 단계별 학습의 데모 방 | `input/checkpoint_flag` |
| [`examples/rooms/coin_room.yaml`](../examples/rooms/coin_room.yaml) | 코인 3개를 다 주우면 문이 열립니다. 같은 조립은 `puzzles/coins/coin_lock` | `coin`, `coin_lock` |
| [`examples/rooms/mini_soccer.yaml`](../examples/rooms/mini_soccer.yaml) | 공을 몸으로 밀어 골대에 넣으면 문이 열립니다 | `ball`, `goal_net` |
| `puzzles/ball/ball_ramp` | 높은 단 위의 공을 경사로(3.4°)로 밀어 내려 골대에 넣습니다. 글자 지도의 경사로라 걷는 몸의 경사 한도 5° 안입니다 | 글자 지도 |
| `puzzles/ball/bowling` | 공으로 핀 셋 중 하나를 쓰러뜨리면(`<핀>.down`) 문이 열립니다 | `ball`, `pin` |
| `puzzles/ball/color_sort` | 색별 골대 두 개에 공을 나눠 넣고 카운터로 문을 엽니다 | `ball`, `goal_net`, `counter` |
| [`examples/rooms/lift_spring.yaml`](../examples/rooms/lift_spring.yaml) | 버튼 하나가 문을 열고, 엘리베이터를 올리고, 설정 변경기로 스프링을 켜서 공이 튀어 오릅니다 | `device/elevator`, `device/spring_pad`, `device/setter`, `device/sliding_door` |
| [`examples/rooms/trap_run.yaml`](../examples/rooms/trap_run.yaml) | 구덩이를 돌아가고, 바닥에서 도는 막대 옆과 약한 옆바람 속을 지나 골로 | `trap/pit`, `trap/spinning_bar`, `trap/crosswind` |
| [`examples/rooms/herd.yaml`](../examples/rooms/herd.yaml) | 도망치는 로봇을 구석의 버튼 위로 몰아 두면 그동안 문이 열려 있습니다 | `npc/flee`, `input/floor_momentary`(`trigger: npc`), `device/sliding_door` |
| [`examples/rooms/warp_hop.yaml`](../examples/rooms/warp_hop.yaml) | 문 없는 벽 너머의 골로 워프 발판을 타고 넘어갑니다 | `warp` |
| [`examples/rooms/cube_plate.yaml`](../examples/rooms/cube_plate.yaml) | 받침대의 큐브를 부리로 집어 압력판에 올리면 문이 열립니다 | `cube`, `pressure`, `door` |
| [`examples/rooms/two_rooms_map.yaml`](../examples/rooms/two_rooms_map.yaml), [`stairs_map.yaml`](../examples/rooms/stairs_map.yaml) | 글자 지도로 그린 두 방, 계단과 경사로 | `button`, `door` |

공 조립 프리셋은 방 하나 전체라서 그대로 `mpg check`에 넣어도 되고, `preset: puzzles/ball/bowling`으로 깔고 값을 바꿔 써도 됩니다.

## 점검 두 단계

`mpg check room.yaml`은 정적 단계(시뮬레이션 없음)와 동적 단계로 나뉩니다. 결과의 `static.failed`, `dynamic.failed`가 실패한 단계를 순서대로 냅니다.

정적 단계:

| 단계 | 보는 것 |
|---|---|
| `values` | 값이 스키마 범위 안인지, 프리셋 이름이 있는지, 마커 번호 |
| `channels` | 아무도 쓰지 않는 채널을 읽는지, 한 채널에 쓰는 부품이 둘인지, 상태 없는 고리, 없는 부품을 가리키는 링크 |
| `demo` | `demo:` 하위 목표의 명령과 대상 |
| `body` | 부품의 `requires_body` 능력이 몸에 있는지 |
| `parts` | 부품별 점검(골대 입구보다 큰 공, 닿을 수 없는 공·코인). 몸만큼 빠른 NPC 로봇은 경고(`warnings`)만 하고 실패하지 않습니다 |
| `overlap` | 기본 배치에서 부품의 단단한 몸체가 벽이나 다른 부품에 5 mm 넘게 박혔는지. 바닥 패드·깃발·골 구역은 빠지고, 문은 자기 벽(문이 막는 틈의 벽, 같은 선 위의 벽)과만 겹쳐도 됩니다 |
| `order` | 신호와 벽으로 볼 때 골까지 가는 풀이 순서가 있는지. 글자 지도 방은 부품이 바닥 칸 위에 있는지와 막는 턱·경사도 |
| `actions` | 풀이에 필요한 행동이 task `actions`에 있는지. 물체를 옮겨야 하면(`demo:`의 `grab`, 또는 풀이 순서에 잡을 수 있는 물체만 켤 수 있는 입력) `grab`, 풀이가 누르는 벽 버튼 판 윗변이 머리를 든 부리 끝(0.21 m)보다 낮으면 `head`가 있어야 합니다 |

앞의 세 단계(`values`, `channels`, `demo`)는 방을 읽는 중에 실패하므로, 그때 결과에는 실패한 단계만 있고 해답 단계는 건너뜁니다. 정적 단계만 보려면 `--static`을 붙입니다.

동적 단계:

| 단계 | 보는 것 |
|---|---|
| `stability` | 명령 0으로 10초 동안 넘어지거나 계산이 터지지 않는지 |
| `solver` | 정답을 아는 스크립트가 무작위 배치 20개 중 18개 이상을 푸는지. 방에 `demo:`(`goto`, `press`, `wait`, `push <공> to <골대>`, `herd <로봇> <부품>`)가 있으면 그 순서를 먼저 따르고, 나머지는 격자 해답이 이어 갑니다 |

해답 단계는 지나간 깃발마다 저장본을 `runs/check/<방>/checkpoints/<깃발>.pkl`에 남깁니다.

## 체크포인트와 단계별 학습

깃발(`input/checkpoint_flag`, `order: 1, 2, …`)이 있는 방은 구간으로 나뉩니다. 먼저 `mpg check`로 깃발 저장본을 만들고, `mpg train --curriculum forward`로 구간을 하나씩 엽니다. 자세한 사용법은 [사용 안내의 단계별 학습](guide.md#단계별-학습과-구간-평가)에 있습니다.

```bash
mpg check examples/rooms/four_flags.yaml
mpg train task.yaml --minutes 60 --curriculum forward
mpg eval runs/<id> --full --segments
```

### 난이도 단계

`task.yaml`의 `difficulty:`는 함정 세기 같은 값을 단계별로 올립니다. 실행 중 변경("예")이나 에피소드 시작에 바꿀 수 있는 값만 됩니다.

```yaml
difficulty:
  - {param: cannon_a.power, from: 2.0, to: 5.0, steps: 4}   # 단계 0..3에서 2, 3, 4, 5
  - {param: wind_a.enabled, from: 0, to: 1, at_step: 2}      # 단계 2부터 켜짐
curriculum: {mode: forward, order: difficulty_first}
```

- `steps: n`은 `from`에서 `to`까지 n단계로 나누고, `at_step: k`는 k단계부터 `to`입니다. 줄마다 단계 수가 다르면 가장 긴 것에 맞추고 짧은 줄은 끝값에 머뭅니다.
- 단계별 학습은 (구간, 난이도) 쌍을 하나씩 엽니다. `order: difficulty_first`(기본)는 한 구간의 난이도를 다 올린 뒤 다음 구간으로, `segment_first`는 한 난이도로 모든 구간을 연 뒤 다음 난이도로 갑니다. 수준 이름은 `<구간>@d<단계>`이고 `curriculum.json`과 에피소드 기록에 남습니다.
- `bind`가 거는 값, 범위 밖 값, 크기 값은 방을 읽을 때 거절합니다.
- 뽑기 없이 돌리면(평가, 직접 `reset`) 마지막 단계입니다. `mpg eval runs/<id> --difficulty 1`처럼 단계를 고를 수 있습니다.

## 큐브와 부리로 집기

- `cube`는 손잡이가 달린 상자입니다(`size`, `mass`, `friction`, 손잡이 면 `handles`: `+x`, `-x`, `+y`, `-y`, `+z` 중, 기본 옆면 넷). 압력판, 소켓, 물체 버튼과 구역이 셉니다. 방에 큐브가 있으면 큐브가 켤 수 있는 입력은 풀이 순서에서 발판으로 칩니다.
- 부리로 집기는 실물 오리로 옮길 수 없는 시뮬레이션 단순화입니다. 시뮬레이션 모델에는 부리 관절이 없어서, 부리 끝(`mouth_tip`)이 손잡이에서 `grab_radius`(기본 4 cm) 안에 있을 때 잡기 명령이 0.5를 넘으면 머리와 큐브 사이 용접 구속을 켭니다. 머리와 큐브는 서로 부딪히지 않습니다. 0.5 이하로 내리면 놓습니다. 질량이 몸의 `max_lift_mass`(기본 0.05 kg)보다 크면 잡히지 않고 `beak.too_heavy`가 1이 됩니다. 붙들고 있는 힘이 `max_hold_force`를 넘으면 놓치고 `<큐브>.dropped`가 펄스로 나갑니다. 다시 잡으려면 명령을 한 번 내렸다 올려야 합니다.
- 부리 끝은 바닥에서 0.135–0.21 m 높이라 바닥의 큐브에는 닿지 않습니다. 큐브는 높이 0.17–0.19 m 받침대 앞 모서리에 손잡이를 몸 쪽으로 두면 걸어 들어가며 잡을 수 있습니다. 글자 지도에서는 높은 바닥 칸(`legend`의 `{type: floor, height: 0.18}`)이 받침대입니다.
- 채널: `beak.holding`, `beak.held_id`(잡은 큐브의 마커 번호, 없으면 -1), `beak.too_heavy`, 큐브마다 `<큐브>.held`, `<큐브>.dropped`. 잡힌 큐브는 바닥 버튼, 압력판, 구역, 소켓이 세지 않습니다. 저장본은 잡은 상태까지 되돌립니다. 워프는 물고 있는 큐브도 머리에 대한 자세를 지켜 함께 옮깁니다.
- 관측 항목 `beak`: 잡고 있는지, 잡은 물체 종류(큐브 1, 없으면 0, 다른 물체는 아래 절), 부리 끝에서 가장 가까운 손잡이까지의 상대 위치(몸 방향 기준 x, y, z). 5칸.
- 벽 버튼: `button`에 `mount: wall`을 주면 벽 면(`pos`)에서 `height` 높이에 `facing` 방향을 보는 판(`size`: [너비, 높이])이 됩니다. 부리 끝이 판 위에 있고 판 앞 6 cm 안이면 눌립니다(벽에 붙어 머리를 숙이면 가슴이 먼저 닿아 부리 끝이 판에서 4–5 cm 떨어지기 때문입니다). 판 뒤로는 판 두께 + 1 cm까지만 셉니다. 얇은 벽 건너편의 오리는 누르지 못합니다. `trigger`는 쓰지 않습니다.
- 상위 행동: `task.yaml`의 `actions: [move]`(기본)에 `head`(머리 숙이기 1칸: -1 그대로, 1 가장 낮게)와 `grab`(1칸, 0.5 초과면 잡기)을 이 순서로 더할 수 있습니다. 행동 크기는 3 + 더한 수입니다. 정책이 할 수 없는 일은 스크립트 해답도 하지 않습니다. 큐브를 옮기거나 낮은 벽 버튼을 누르는 방은 task에 `grab`·`head`가 없으면 `mpg check`의 `actions` 단계에서 실패합니다. 예: `mpg check examples/rooms/cube_plate.yaml --task examples/task_cube_plate.yaml`.
- 해답의 `demo:`: `grab <큐브>`(손잡이 앞까지 걸어가 머리 높이를 맞춰 잡음), `place <큐브> <부품>`(부품 가운데 위로 옮겨 멈춘 뒤 놓고 물러남). 벽 버튼은 `press <버튼>`이나 풀이 순서로 머리를 숙여 누릅니다.
- 예제 [`examples/rooms/cube_plate.yaml`](../examples/rooms/cube_plate.yaml): 받침대의 큐브를 집어 압력판에 올려 문을 엽니다.

## 부리로 다루는 물체와 받는 부품

`cube` 밖에도 손잡이가 있는 물체는 모두 같은 방식으로 집습니다(잡기 반경, 들 수 있는 질량, 놓치는 힘, `.held`·`.dropped` 채널이 같습니다). 프리셋은 `beak` 분류에 있습니다.

| 프리셋 | 종류 | 기본값 | 쓰임 |
|---|---|---|---|
| `beak/cube_weight` | `cube` | 4 cm, 40 g | 저울·압력판에 올리는 추. 몸의 `max_lift_mass`(0.05 kg)보다 조금 가볍습니다 |
| `beak/laundry` | `laundry` | 12 × 10 cm, 두께 6 mm, 20 g, 네 변 위에 단(`hem`) | 가벼운 빨래. 판은 단단하고 주름은 겉모양뿐입니다. 위쪽 단을 집습니다 |
| `beak/key` | `key` | 20 g, 머리 4 cm, 몸통 5 cm, `-x` 끝에 고리 | 머리에 마커가 붙습니다. `accept: [<마커 번호>]`를 준 받는 부품만 받습니다 |
| `beak/handle_ball` | `handle_ball` | 반지름 3 cm, 30 g, `-x` 쪽 고리 | 놓으면 굴러갑니다. 공 부품 `ball`과는 다릅니다 |
| `beak/egg` | `egg` | 25 g, 반지름 2.2 cm, 높이 6 cm, 옆에 손잡이 넷, `break_height` 0.05 m, `on_break: fail` | 깨지면 `<알>.broken`이 1이 됩니다 |
| `beak/stick` | `stick` | 길이 20 cm, 굵기 1.6 cm, 20 g, `-x` 끝에 손잡이 | 물고 있으면 먼 끝이 벽 버튼을 누릅니다 |
| `beak/hook` | `receptacle` `shape: hook` | 기둥 8 × 8 cm, 높이 0.19 m에 길이 6 cm 걸이, `tolerance` 0.04 m | 걸이 끝 4 cm 안에서 놓은 물체가 걸립니다 |
| `beak/basket` | `receptacle` `shape: box` | 30 × 30 cm, 벽 높이 10 cm, `require: count`, `count: 1` | `<바구니>.count`와 `<바구니>.mass`(kg)로 담긴 것을 셉니다 |
| `beak/scale` | `pressure` `trigger: object` | 20 × 20 cm, `min_mass` 5 g | 숫자 채널 `<저울>.mass`(kg, 1 g 단위). 5 g부터 `<저울>`이 1 |
| `beak/wall_button_beak_low` | `button` `mount: wall` | 판 12 × 6 cm, 가운데 높이 0.15 m, `oneshot` | 머리를 숙여야(`head` 행동) 닿습니다 |
| `beak/wall_button_beak_mid` | `button` `mount: wall` | 판 12 × 6 cm, 가운데 높이 0.2 m, `oneshot` | 선 채로 부리 끝(0.21 m)이 닿습니다 |

- 알: 닿기가 시작될 때의 법선 방향 상대 속도가 `break_height`에서 떨어진 속도(√(2·9.81·`break_height`), 기본 0.99 m/s) 이상이면 깨집니다. 기본 몸으로 잰 값(2026-11-12): 부리에서 바닥으로 놓으면 1.42 m/s(머리를 가장 낮게)–1.86 m/s(머리를 든 채), 받침대에서 집어 바닥 판에 놓으면 1.68–1.87 m/s, 집어 나르는 동안의 부딪힘은 0.24 m/s 이하(받침대에 밀어붙이면 0.49 m/s), 내려앉은 뒤 건드림은 0.35 m/s 이하입니다. 그래서 5 cm로 두면 놓을 때만 깨집니다. 기본 부품 값 `on_break`는 `none`(채널만), 프리셋은 `fail`(깨지면 에피소드 실패)입니다. 깨진 알은 색이 바뀌고, 저장본과 리셋이 되돌립니다.
- 열쇠와 소켓: 받는 부품의 `accept`에는 이름과 종류 말고 마커 번호(정수)도 쓸 수 있습니다. 소켓이 같은 번호의 마커를 달아도 되고, 이때만 마커 번호가 겹쳐도 됩니다(다른 겹침은 여전히 거절). 방향은 소켓의 `angle_tolerance`로 봅니다.
- 걸이: 물고 있지 않은 물체의 손잡이가 걸이 끝에서 `tolerance` 안에 오면 걸이에 고정되고 담긴 것으로 셉니다. 다시 물면 빠집니다. 걸이는 보이기만 하고 부딪히지 않는 논리 걸이입니다(기둥만 부딪히고, 기둥과 걸 수 있는 물체 사이 접촉은 끕니다). `facing`이 걸이 방향입니다.
- 바구니: 벽이 10 cm보다 낮으면 몸이 걸려 넘어지거나 벽을 타고 넘습니다(5 cm 벽에서 4번 중 3번 넘어짐). 10 cm 벽 너머로는 머리를 든 부리 끝이 벽 면에서 5.8–6.8 cm 앞까지 닿습니다.
- 막대: 물고 있는 막대의 먼 끝도 벽 버튼에서 부리 끝과 같은 창(판 앞 6 cm, 판 뒤로 판 두께 + 1 cm, 판 너비·높이 안)으로 셉니다. 잰 값: 스크립트로 천천히 다가가면 10번 모두 막대 끝이 판 면 앞 2.9–5.9 cm, 높이 차 2.7 cm 이하에서 눌렀고, 누르기 전 용접 힘은 0.89 N 이하였습니다. 벽으로 그냥 걸어 들어가면 6번 중 4번은 힘이 `max_hold_force`(3 N)를 넘어 막대를 놓칩니다. 머리를 숙이면 막대 끝이 크게 내려갑니다(머리 1단위에 약 0.43 rad).
- 관측 `beak`의 물체 종류: `cube` 1, `laundry` 2, `key` 3, `handle_ball` 4, `egg` 5, `stick` 6, 없으면 0.
- 해답의 `demo:`에 `drop <물체> <부품>`이 더해집니다. `place`와 같이 옮기되 머리를 든 채 놓습니다(알을 깨는 해답). `place <물체> <걸이>`는 앞장선 손잡이를 걸이 끝에 맞춰 겁니다. 막대를 물고 `press <버튼>`을 하면 막대 끝을 판 높이에 맞춰 누릅니다.
- 알려진 한계: 깨진 알도 받는 부품이 셉니다. 바구니·저울에 놓는 스크립트 해답은 방과 시드에 따라 10번 중 8–10번 성공합니다.

## 파이썬 플러그인

새 부품은 `Part`를 상속해 `register_part`로 등록합니다. 값은 `fields`에 `Field`로 적으면 범위 검사와 `mpg parts` 표가 저절로 따라옵니다. 게이트 함수는 `register_logic`으로 등록하고 방에서는 `logic: <이름>`으로 부릅니다.

```python
# my_parts.py
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_logic, register_part
from mujoco_puzzle_gym.schema import Field

@register_part("timed_switch")
class TimedSwitch(Part):
    role = "switch"
    stage = "input"
    emits = True
    fields = (Field("seconds", "float", 2.0, unit="s", min=0.1, max=60.0),)
    ...

@register_logic("majority")
def majority(inputs, state, dt):
    return float(sum(v > 0.5 for v in inputs.values()) * 2 > len(inputs))
```

`task.yaml`에 `plugins: [my_parts]`를 적으면 학습 작업자 프로세스까지 이 모듈을 먼저 불러옵니다. 프리셋 폴더는 이 패키지 안에만 있습니다. 직접 만든 부품의 기본값 묶음은 방 파일에 값으로 적습니다.

## 값 표

아래는 `mpg parts --markdown`의 출력입니다. 부품을 고치면 이 부분을 그 출력으로 바꿉니다(`tests/test_guide.py`가 확인합니다).

<!-- mpg parts --markdown: begin -->
### 모든 부품의 공통 값

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | 필수 |  |  | position [x, y] on the floor |
| `random` | vec | m | `[0.0, 0.0]` | [0, 5] |  | uniform +- range added to pos on every reset |
| `require` | enum |  | `all` | `all`, `any` |  | gates: open when all / any of their inputs are on |
| `requires_body` | list |  | `[]` |  |  | body capabilities this part needs (R2b), checked by `mpg check` |
| `in` | list |  | `[]` |  |  | channels this part reads (R1) |
| `out` | list |  | — |  |  | channels this part writes (default: its name if it emits) |
| `enable` | str |  | — |  |  | power channel; off (<= 0.5) freezes the part's output |
| `marker` | any |  | — |  |  | ArUco marker (R26): `none` or {id, size, face}; default: a free id on parts that carry one |
| `bind` | any |  | — |  |  | R13: {<run-time value>: {from: <channel>, map: [in0, in1, out0, out1]}}, applied every body step, clipped to the value's range |

### `ball`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.05` | [0.02, 0.2] |  | OQ10: >= 0.05 m pushes without falling (<= 0.3 kg) |
| `mass` | float | kg | `0.05` | [0.005, 5] |  | OQ10: <= 0.3 kg pushes without falling (radius >= 0.05 m) |
| `elasticity` | float |  | `0.5` | [0, 0.95] |  | restitution (0 = no bounce), through contact solref |
| `friction` | float |  | `0.8` | [0, 2] |  | sliding friction |
| `roll_friction` | float | m | `0.005` | [0, 0.05] |  | rolling friction (0.005: a 0.5 m/s ball stops in about 0.2 m) |
| `roll_damping` | float | N m s/rad | `0.0` | [0, 1] |  | damping on the rotational dofs |
| `color` | enum |  | `white` | `white`, `red`, `blue`, `green`, `yellow`, `orange`, `purple`, `black` |  |  |

### `button`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `kind` | enum |  | `momentary` | `momentary`, `oneshot`, `toggle` |  |  |
| `mount` | enum |  | `floor` | `floor`, `wall` |  | floor: stepped on; wall: pressed with the beak tip |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.02, 2] |  | plate size [x, y]; wall: [width, height] |
| `height` | float | m | `0.2` | [0.02, 1] |  | wall: plate centre above the floor |
| `facing` | float | deg | `0.0` | [-180, 180] |  | wall: the direction the plate faces (0 = +x) |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |

### `cannon`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 turns the trap off (setters, difficulty steps) |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | heading the cannon faces |
| `height` | float | m | `0.15` | [0.05, 1.5] |  | muzzle height above the floor |
| `power` | float | m/s | `5.0` | [0.5, 15] | 예 | launch speed |
| `shell_mass` | float | kg | `0.2` | [0.005, 1] |  | shell mass |
| `shell_radius` | float | m | `0.025` | [0.01, 0.1] |  |  |
| `period` | float | s | `2.0` | [0.2, 60] | 예 | fire: auto: time between bursts |
| `burst` | int |  | `1` | [1, 10] | 예 | shells per burst |
| `spread` | float | deg | `0.0` | [0, 30] | 예 | half angle of the random cone around the aim |
| `aim` | enum |  | `fixed` | `fixed`, `at_body`, `lead`, `signal` |  | fixed: yaw + aim_yaw, aim_pitch; at_body / lead: at the body (now / ahead); signal: aim_yaw, aim_pitch from channels (bind) |
| `aim_yaw` | float | deg | `0.0` | [-180, 180] | 예 | aim left / right of `yaw` |
| `aim_pitch` | float | deg | `10.0` | [-30, 80] | 예 | aim elevation |
| `lifetime` | float | s | `3.0` | [0.2, 30] |  | a shell is parked after this |
| `pool` | int |  | `10` | [1, 50] |  | shells built in advance (reused) |
| `fire` | enum |  | `auto` | `auto`, `edge` |  | auto: every `period` while active; edge: one burst per rising edge of `in` |

### `checkpoint`

역할 `checkpoint`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `order` | int |  | `1` | [1, 100] |  | flags turn on in this order |
| `radius` | float | m | `0.15` | [0.03, 1] |  | reach distance |
| `skip` | enum |  | `ignore` | `ignore`, `fail` |  | reaching this flag before its turn: ignore it, or fail the episode |

### `coin`

역할 `prop`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.08` | [0.02, 0.5] |  | pick-up distance from the body centre |
| `group` | str |  | `coins` |  |  | coin group (channels <group>.count, <group>.total) |

### `coin_group`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the group (it has no body) |

### `coin_lock`

역할 `prop`, 갱신 단계 `logic`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the counter (it has no body) |
| `up` | str |  | — |  |  | set to <group>.picked |
| `down` | str |  | — |  |  | channel whose turn-ons subtract 1 |
| `reset` | str |  | — |  |  | channel whose turn-on sets the count to start |
| `start` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `min` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `max` | int |  | `1000` | [-1e+06, 1e+06] |  |  |
| `wrap` | bool |  | `False` |  |  | past max -> min, below min -> max |
| `target` | int |  | `1` | [-1e+06, 1e+06] |  | `<name>.reached` is 1 at count >= target |
| `persist` | bool |  | `False` |  |  | keep the count when a snapshot is restored |
| `group` | str |  | `coins` |  |  | coin group whose picks it counts |
| `need` | int |  | — | [1, 10000] |  | coins needed to reach; `all` (default) = every coin of the group |

### `conveyor`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.8, 0.3]` | [0.1, 5] |  | belt size [x, y] |
| `axis` | enum |  | `x` | `x`, `y` |  | direction the belt runs along |
| `speed` | float | m/s | `0.1` | [-0.5, 0.5] | 예 | belt speed (+: toward +axis) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it carries |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `counter`

역할 `prop`, 갱신 단계 `logic`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the counter (it has no body) |
| `up` | str |  | 필수 |  |  | channel whose turn-ons add 1 |
| `down` | str |  | — |  |  | channel whose turn-ons subtract 1 |
| `reset` | str |  | — |  |  | channel whose turn-on sets the count to start |
| `start` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `min` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `max` | int |  | `1000` | [-1e+06, 1e+06] |  |  |
| `wrap` | bool |  | `False` |  |  | past max -> min, below min -> max |
| `target` | int |  | `1` | [-1e+06, 1e+06] |  | `<name>.reached` is 1 at count >= target |
| `persist` | bool |  | `False` |  |  | keep the count when a snapshot is restored |

### `cube`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.03` | [0.001, 10] |  | above the body's max_lift_mass the beak cannot lift it |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `size` | float | m | `0.04` | [0.02, 0.3] |  | edge length |
| `handles` | list |  | `['+x', '-x', '+y', '-y']` |  |  | faces with a beak handle: +x, -x, +y, -y, +z |
| `handle_size` | float | m | `0.015` | [0.005, 0.1] |  | how far a handle sticks out of its face |

### `door`

역할 `gate`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `width` | float | m | `0.5` | [0.1, 2] |  | opening the panel spans |
| `axis` | enum |  | `y` | `x`, `y` |  | direction the panel lies along |
| `thickness` | float | m | `0.04` | [0.01, 0.2] |  |  |
| `height` | float | m | `0.25` | [0.05, 1] |  |  |
| `open_dir` | enum |  | `1` | `1`, `-1` |  | slide toward +axis (1) or -axis (-1) |
| `speed` | float | m/s | — | [0.02, 5] |  | opening speed (default: as fast as the motor moves it) |

### `egg`

역할 `prop`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.025` | [0.005, 0.2] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `radius` | float | m | `0.022` | [0.01, 0.06] |  | egg radius (x, y) |
| `height` | float | m | `0.06` | [0.02, 0.15] |  | egg height (z) |
| `handles` | list |  | `['+x', '-x', '+y', '-y']` |  |  | faces with a knob: +x, -x, +y, -y |
| `handle_size` | float | m | `0.012` | [0.005, 0.05] |  |  |
| `break_height` | float | m | `0.05` | [0.005, 2] | 예 | a contact starting as fast as a drop from this height breaks it |
| `on_break` | enum |  | `none` | `none`, `fail` |  | fail: a broken egg fails the episode |

### `floor_tile`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `friction` | vec | - | `[1.0, 0.005, 0.0001]` | [0, 5] | 예 | slide, torsion, roll friction |
| `tilt` | float | deg | `0.0` | [-13, 13] | 예 | tilt about tilt_axis (+: raises the +y side for axis x, +x side for axis y) |
| `tilt_axis` | enum |  | `x` | `x`, `y` |  | hinge axis of the tilt |
| `height` | float | m | `0.0` | [-0.1, 0.1] | 예 | top face height above the room floor |
| `on_friction` | vec | - | — | [0, 5] | 예 | friction while the inputs are on |
| `on_tilt` | float | deg | — | [-13, 13] | 예 | tilt while the inputs are on |
| `on_height` | float | m | — | [-0.1, 0.1] | 예 | height while the inputs are on |
| `friction_rate` | float | 1/s | `5.0` | [0.01, 100] | 예 | friction change per second |
| `tilt_speed` | float | deg/s | `10.0` | [0.5, 90] | 예 |  |
| `height_speed` | float | m/s | `0.05` | [0.005, 0.5] | 예 |  |

### `gate`

역할 `prop`, 갱신 단계 `logic`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the gate (it has no body) |
| `expr` | str |  | — |  |  | safe expression over channels (R7) |
| `op` | enum |  | — | `and`, `or`, `xor`, `not`, `delay`, `hold`, `pulse`, `timer`, `count`, `latch` |  | preset logic over `in` |
| `seconds` | float | s | `1.0` | [0, 3600] |  | time for op delay / hold |
| `sequence` | list |  | — |  |  | sequence lock: channels that must turn on in this order |
| `latch` | bool |  | `False` |  |  | sequence lock: stay on once the order is complete |
| `logic` | str |  | — |  |  | registered Python logic (register_logic) |
| `stateful` | bool |  | `False` |  |  | logic plugins: output does not depend on this step's input (may sit in a loop) |

### `goal`

역할 `goal`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.15` | [0.03, 1] |  | reach distance |

### `goal_net`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `width` | float | m | `0.36` | [0.08, 2] |  | mouth width (inside) |
| `depth` | float | m | `0.2` | [0.05, 1] |  | inside depth |
| `height` | float | m | `0.12` | [0.03, 0.5] |  | wall height |
| `facing` | float | deg | `180.0` | [-180, 180] |  | direction the open side faces |
| `thickness` | float | m | `0.02` | [0.005, 0.1] |  | wall thickness |
| `accepts` | list |  | `[]` |  |  | ball colours that count (empty: any) |
| `need` | int |  | `1` | [1, 100] |  | balls in for the main channel |

### `handle_ball`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.03` | [0.001, 10] |  | above the body's max_lift_mass the beak cannot lift it |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `radius` | float | m | `0.03` | [0.015, 0.1] |  |  |
| `roll_friction` | float | m | `0.002` | [0, 0.05] |  | rolling friction (low: it rolls on once let go) |
| `handles` | list |  | `['-x']` |  |  | faces with a ring handle: +x, -x, +y, -y |
| `ring` | float | m | `0.012` | [0.005, 0.04] |  | ring radius |

### `key`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.02` | [0.001, 1] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `head` | float | m | `0.04` | [0.02, 0.1] |  | side of the square head (the marker sits on it) |
| `shaft` | float | m | `0.05` | [0.01, 0.2] |  | shaft length along +x |
| `handle_size` | float | m | `0.015` | [0.005, 0.05] |  | how far the ring sticks out of the head's -x end |

### `laser`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `to` | vec | m | 필수 |  |  | receiver position [x, y] |
| `height` | float | m | `0.05` | [0.01, 0.5] |  | beam height |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  |  |

### `laundry`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.02` | [0.001, 1] |  | a light cloth |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `size` | vec | m | `[0.12, 0.1]` | [0.04, 0.4] |  | cloth [x, y] |
| `thickness` | float | m | `0.006` | [0.002, 0.03] |  |  |
| `handles` | list |  | `['+x', '-x', '+y', '-y']` |  |  | top edges with a hem the beak grabs: +x, -x, +y, -y |
| `hem` | float | m | `0.01` | [0.004, 0.05] |  | how far a hem sticks out of its edge |

### `npc`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `behavior` | enum |  | `flee` | `flee`, `follow`, `lure`, `patrol`, `guard`, `shy_gaze`, `seek_button`, `sleepy` |  | flee, follow, lure, patrol, guard, shy_gaze, seek_button or sleepy |
| `size` | vec | m | `[0.12, 0.12, 0.07]` | [0.04, 0.5] |  | chassis [x, y, z] |
| `mass` | float | kg | `0.3` | [0.02, 5] |  |  |
| `speed` | float | m/s | `0.08` | [0, 1] | 예 | top speed; at or above the body's 0.16 m/s `mpg check` warns |
| `accel` | float | m/s^2 | `0.5` | [0.05, 10] | 예 |  |
| `sense` | float | m | `0.4` | [0.05, 5] | 예 | reacts to the body this close (flee, follow, lure, patrol, guard) |
| `leash` | vec | m | — | [-50, 50] |  | [x0, y0, x1, y1] box (world) its centre never leaves; default: none |
| `keep` | float | m | `0.3` | [0, 3] | 예 | follow, lure, shy_gaze: stops this far from the body / bait |
| `reach` | float | m | `0.25` | [0, 3] | 예 | guard: how far it steps out from its post |
| `path` | any |  | — |  |  | patrol, sleepy: [[dx, dy], ...] from `pos`, 2 or more points, a loop |
| `button` | str |  | — |  |  | seek_button: the part it goes onto |
| `bait` | str |  | — |  |  | lure: the object the body must hold |
| `awake` | float | s | `2.0` | [0.1, 60] | 예 | sleepy: seconds moving |
| `asleep` | float | s | `2.0` | [0.1, 60] | 예 | sleepy: seconds still |
| `invert` | bool |  | `False` |  |  | shy_gaze: move while looked at, stop otherwise |

### `pendulum`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 turns the trap off (setters, difficulty steps) |
| `height` | float | m | `0.6` | [0.1, 2] |  | pivot above the floor |
| `length` | float | m | `0.45` | [0.05, 1.9] |  | pivot to ball centre |
| `radius` | float | m | `0.06` | [0.01, 0.3] |  | ball radius |
| `mass` | float | kg | `0.5` | [0.01, 5] |  | ball mass |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | swing direction |
| `period` | float | s | `1.6` | [0.3, 10] | 예 | one full swing |
| `power` | float | deg | `40.0` | [0, 80] | 예 | swing amplitude while active |

### `pillar`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `size` | vec | m | `[0.1, 0.4]` | [0.03, 2] |  | footprint [x, y] |
| `height` | float | m | `0.25` | [0.05, 1] |  | top height when up |
| `speed` | float | m/s | `0.2` | [0.01, 2] | 예 | rising speed |

### `pin`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.02` | [0.01, 0.1] |  |  |
| `height` | float | m | `0.12` | [0.03, 0.5] |  |  |
| `mass` | float | kg | `0.03` | [0.005, 2] |  |  |
| `down_angle` | float | deg | `45.0` | [5, 90] |  | tilt from upright that counts as down |
| `color` | enum |  | `white` | `white`, `red`, `blue`, `green`, `yellow`, `orange`, `purple`, `black` |  |  |

### `pit`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 closes the lid; only a pit with `in` can be switched |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `on_fall` | enum |  | `fail` | `fail`, `reset` |  | fail: the episode ends; reset: back to the last checkpoint or the start |

### `platform`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `path` | any |  | 필수 |  |  | [[dx, dy, z], ...] from `pos`: points the top centre follows (2 or more) |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 2] |  | slab size [x, y] |
| `thickness` | float | m | `0.03` | [0.01, 0.2] |  |  |
| `speed` | float | m/s | `0.1` | [0.01, 1] | 예 | speed along the path |
| `mode` | enum |  | `pingpong` | `loop`, `pingpong`, `once` |  | loop: closed loop; pingpong: back and forth; once: to the end and stop |
| `run` | enum |  | `always` | `always`, `while_signal`, `on_signal` |  | always; while_signal: while `in` is on; on_signal: one trip per rising edge |

### `pressure`

역할 `sensor`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  |  |
| `min_mass` | float | kg | `0.1` | [0.001, 100] | 예 | mass needed to turn on |
| `trigger` | enum |  | `object` | `body`, `object`, `npc`, `any` |  |  |

### `pusher`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 turns the trap off (setters, difficulty steps) |
| `size` | vec | m | `[0.3, 0.1]` | [0.03, 1.5] |  | paddle [width, height] |
| `stroke` | float | m | `0.3` | [0.05, 1.5] |  | how far the paddle runs out |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | direction it pushes (0: +x) |
| `power` | float | m/s | `0.3` | [0.02, 2] | 예 | paddle speed |

### `receiver`

역할 `prop`, 갱신 단계 `logic`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `band` | str |  | 필수 |  |  | band name shared by a transmitter and its receivers |
| `delay` | float | s | `0.0` | [0, 3600] |  | hold the value back |

### `receptacle`

역할 `sensor`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `require` | enum |  | `all` | `all`, `count`, `mass`, `order` |  |  |
| `shape` | enum |  | `box` | `box`, `plate`, `hook` |  |  |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 2] |  |  |
| `wall_height` | float | m | `0.06` | [0.01, 0.5] |  |  |
| `wall_thickness` | float | m | `0.01` | [0.005, 0.1] |  |  |
| `accept` | any |  | `[]` |  |  | object names, kinds or marker ids that count (empty: any) |
| `count` | int |  | `1` | [1, 100] |  | require count: objects needed |
| `mass` | float | kg | `0.1` | [0.001, 100] |  | require mass |
| `order` | list |  | — |  |  | require order: names in the order they go in |
| `tolerance` | float | m | `0.05` | [0.005, 1] |  | plate: position error; hook: handle to peg tip |
| `angle_tolerance` | float | deg | `15.0` | [0, 180] |  | plate: heading error |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | plate: wanted heading |
| `angle_period` | float | deg | `90.0` | [1, 360] |  | plate: headings this far apart look the same |
| `wrong` | enum |  | `ignore` | `ignore`, `eject`, `lock`, `fail` |  |  |
| `eject_speed` | float | m/s | `1.0` | [0.1, 5] |  |  |
| `close_on_ok` | bool |  | `False` |  |  |  |
| `height` | float | m | `0.19` | [0.05, 1] |  | hook: peg height |
| `facing` | float | deg | `0.0` | [-180, 180] |  | hook: the way the peg points (0 = +x) |
| `peg` | float | m | `0.06` | [0.02, 0.3] |  | hook: peg tip out from the post |

### `setter`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | unused (a setter has no body) |
| `target` | str |  | 필수 |  |  | <part>.<value>: the run-time value to change |
| `value` | any |  | 필수 |  |  | new value (checked against the target value's schema) |
| `revert` | bool |  | `False` |  |  | put the old value back when the signal turns off |

### `spinner`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 turns the trap off (setters, difficulty steps) |
| `length` | float | m | `0.4` | [0.05, 1.5] |  | hub to each tip |
| `height` | float | m | `0.03` | [0.01, 0.3] |  | bar centre above the floor |
| `thickness` | float | m | `0.03` | [0.01, 0.1] |  |  |
| `power` | float | deg/s | `90.0` | [-720, 720] | 예 | turn rate while active (+: counter-clockwise) |

### `spring`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.2, 0.2]` | [0.05, 2] |  | pad size [x, y] |
| `velocity` | vec | m/s | `[0.0, 0.0, 1.5]` | [-5, 5] | 예 | launch velocity [vx, vy, vz] (world frame) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it launches |
| `cooldown` | float | s | `1.0` | [0.1, 60] |  | time a launched body or object ignores this spring |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `stick`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.02` | [0.002, 0.5] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `length` | float | m | `0.2` | [0.08, 0.5] |  | rod length along +x |
| `width` | float | m | `0.016` | [0.006, 0.04] |  | square rod side |
| `handle_size` | float | m | `0.015` | [0.005, 0.05] |  | how far the handle sticks out of the -x end |

### `swing_door`

역할 `gate`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `width` | float | m | `0.5` | [0.1, 2] |  | opening the panel spans |
| `axis` | enum |  | `y` | `x`, `y` |  | direction the closed panel lies along |
| `thickness` | float | m | `0.04` | [0.01, 0.2] |  |  |
| `height` | float | m | `0.25` | [0.05, 1] |  |  |
| `hinge` | enum |  | `-1` | `-1`, `1` |  | hinge at the -axis (-1) or +axis (1) end |
| `open_dir` | enum |  | `1` | `1`, `-1` |  | side the door swings to (+1: +x or +y) |
| `angle` | float | deg | `90.0` | [10, 170] |  | opening angle |
| `speed` | float | deg/s | — | [5, 1000] |  | opening speed (default: as fast as the motor turns it) |

### `switch`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `kind` | enum |  | `momentary` | `momentary`, `oneshot`, `toggle` |  |  |
| `mount` | enum |  | `floor` | `floor`, `wall` |  | floor: stepped on; wall: pressed with the beak tip |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.02, 2] |  | plate size [x, y]; wall: [width, height] |
| `height` | float | m | `0.2` | [0.02, 1] |  | wall: plate centre above the floor |
| `facing` | float | deg | `0.0` | [-180, 180] |  | wall: the direction the plate faces (0 = +x) |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |
| `latch` | bool |  | `True` |  |  | M1: stay on after the body leaves (oneshot); false: momentary |

### `target`

역할 `sensor`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.2, 0.2]` | [0.03, 2] |  | board [width, height] |
| `height` | float | m | `0.2` | [0.05, 2] |  | board centre above the floor |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | direction the face looks |
| `min_speed` | float | m/s | `1.0` | [0, 30] | 예 | slowest hit that counts |
| `mode` | enum |  | `latch` | `pulse`, `latch` |  | pulse: 1 for the body step of a hit; latch: stays 1 |
| `trigger` | enum |  | `shell` | `shell` |  | only cannon shells count |

### `transmitter`

역할 `prop`, 갱신 단계 `logic`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `band` | str |  | 필수 |  |  | band name shared by a transmitter and its receivers |
| `delay` | float | s | `0.0` | [0, 3600] |  | hold the value back |

### `trapdoor`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `hinge_side` | enum |  | `-x` | `-x`, `+x`, `-y`, `+y` |  | edge the tile hangs from |
| `speed` | float | deg/s | `180.0` | [10, 2000] |  | opening speed |

### `turntable`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.25` | [0.05, 1.5] |  |  |
| `height` | float | m | `0.008` | [0.004, 0.1] |  | top face above the floor |
| `speed` | float | deg/s | `45.0` | [-360, 360] | 예 | spin rate while on (+: counter-clockwise) |

### `warp`

역할 `warp`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.25, 0.25]` | [0.1, 2] |  | pad size [x, y] |
| `trigger` | enum |  | `body` | `body`, `object`, `npc`, `any` |  | what it moves: body, object, npc or any |
| `to` | any |  | — |  |  | destination: {pos: [x, y], yaw}, reset or next_stage |
| `routes` | any |  | — |  |  | conditional destinations: [{when: expr, to: dest}, ..., {else: dest}] |
| `cooldown` | float | s | `1.0` | [0, 60] |  | time a warped body or object ignores warp pads |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `wind`

역할 `prop`, 갱신 단계 `device`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 예 | 0 turns the trap off (setters, difficulty steps) |
| `size` | vec | m | `[0.6, 0.6]` | [0.05, 5] |  | zone size [x, y] |
| `height` | float | m | `0.4` | [0.05, 2] |  | zone height above the floor |
| `yaw` | float | deg | `0.0` | [-180, 180] | 예 | direction the wind blows (0: +x, 90: +y) |
| `power` | float | N | `1.0` | [0, 20] | 예 | force on each body or object in the zone |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it pushes |

### `zone`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.4, 0.4]` | [0.05, 10] |  | box size [x, y] |
| `height` | float | m | `0.5` | [0.02, 3] |  | box height above the floor |
| `trigger` | enum |  | `body` | `body`, `object`, `npc`, `any` |  |  |
| `objects` | list |  | — |  |  | only these object parts count (overrides trigger) |
| `direction` | vec |  | — |  |  | count only entries moving this way |
| `rearm` | enum |  | `always` | `always`, `leave` |  |  |
| `leave_margin` | float | m | `0.05` | [0, 1] |  | rearm leave: how far outside the box a target must go |

### 부품 프리셋

| 프리셋 | 부품 종류 | 값 |
|---|---|---|
| `beak/basket` | `receptacle` | `shape: box`, `size: [0.3, 0.3]`, `wall_height: 0.1`, `require: count`, `count: 1` |
| `beak/cube_weight` | `cube` | `size: 0.04`, `mass: 0.04` |
| `beak/egg` | `egg` | `break_height: 0.05`, `on_break: fail` |
| `beak/handle_ball` | `handle_ball` | `radius: 0.03`, `mass: 0.03` |
| `beak/hook` | `receptacle` | `shape: hook`, `size: [0.08, 0.08]`, `height: 0.19`, `peg: 0.06`, `tolerance: 0.04` |
| `beak/key` | `key` | `mass: 0.02` |
| `beak/laundry` | `laundry` | `size: [0.12, 0.1]`, `mass: 0.02` |
| `beak/scale` | `pressure` | `trigger: object`, `size: [0.2, 0.2]`, `min_mass: 0.005` |
| `beak/stick` | `stick` | `length: 0.2`, `mass: 0.02` |
| `beak/wall_button_beak_low` | `button` | `mount: wall`, `kind: oneshot`, `height: 0.15`, `size: [0.12, 0.06]` |
| `beak/wall_button_beak_mid` | `button` | `mount: wall`, `kind: oneshot`, `height: 0.2`, `size: [0.12, 0.06]` |
| `device/conveyor` | `conveyor` | `size: [0.8, 0.3]`, `axis: x`, `speed: 0.1` |
| `device/elevator` | `platform` | `size: [0.4, 0.4]`, `path: [[0.0, 0.0, 0.02], [0.0, 0.0, 0.3]]`, `speed: 0.1`, `mode: pingpong`, `run: on_signal` |
| `device/goal_net` | `goal_net` | `width: 0.36`, `depth: 0.2`, `height: 0.12` |
| `device/lift_pillar` | `pillar` | `size: [0.1, 0.4]`, `height: 0.25`, `speed: 0.2`, `mode: follow` |
| `device/moving_platform` | `platform` | `size: [0.3, 0.3]`, `path: [[0.0, 0.0, 0.03], [0.6, 0.0, 0.03]]`, `speed: 0.1`, `mode: pingpong` |
| `device/oneshot_door` | `door` | `mode: oneshot`, `width: 0.5`, `axis: y` |
| `device/reset_pad` | `warp` | `to: reset`, `size: [0.25, 0.25]` |
| `device/setter` | `setter` | `revert: False` |
| `device/sliding_door` | `door` | `mode: follow`, `width: 0.5`, `axis: y` |
| `device/spring_pad` | `spring` | `size: [0.2, 0.2]`, `velocity: [0.0, 0.0, 1.5]`, `trigger: object` |
| `device/swing_door` | `swing_door` | `mode: follow`, `width: 0.5`, `axis: y`, `angle: 90.0` |
| `device/trapdoor` | `trapdoor` | `size: [0.3, 0.3]`, `hinge_side: -x` |
| `device/turntable` | `turntable` | `radius: 0.25`, `speed: 45.0` |
| `floor/coin` | `coin` | `radius: 0.08`, `group: coins` |
| `floor/rising_step` | `floor_tile` | `size: [0.3, 0.3]`, `height: 0.0`, `on_height: 0.01` |
| `floor/tilt_bridge` | `floor_tile` | `size: [0.4, 0.3]`, `tilt: 0.0`, `on_tilt: 5.0` |
| `input/body_zone` | `zone` | `trigger: body`, `size: [0.4, 0.4]`, `height: 0.5` |
| `input/checkpoint_flag` | `checkpoint` | `order: 1`, `radius: 0.15`, `skip: ignore` |
| `input/floor_momentary` | `button` | `kind: momentary`, `trigger: any`, `size: [0.25, 0.25]` |
| `input/floor_oneshot` | `button` | `kind: oneshot`, `trigger: body`, `size: [0.25, 0.25]` |
| `input/floor_toggle` | `button` | `kind: toggle`, `trigger: body`, `size: [0.25, 0.25]` |
| `input/item_box` | `receptacle` | `shape: box`, `size: [0.3, 0.3]`, `wall_height: 0.06`, `require: all`, `wrong: ignore` |
| `input/laser_beam` | `laser` | `trigger: any`, `height: 0.05` |
| `input/object_zone` | `zone` | `trigger: object`, `size: [0.4, 0.4]`, `height: 0.3` |
| `input/pressure_plate` | `pressure` | `trigger: object`, `size: [0.25, 0.25]`, `min_mass: 0.1` |
| `input/socket` | `receptacle` | `shape: plate`, `size: [0.2, 0.2]`, `tolerance: 0.05`, `angle_tolerance: 15.0` |
| `input/target` | `target` | `size: [0.2, 0.2]`, `height: 0.2`, `mode: latch` |
| `logic/and` | `gate` | `op: and` |
| `logic/counter` | `counter` | `start: 0`, `target: 3` |
| `logic/delay` | `gate` | `op: delay`, `seconds: 1.0` |
| `logic/hold` | `gate` | `op: hold`, `seconds: 2.0` |
| `logic/latch` | `gate` | `op: latch` |
| `logic/not` | `gate` | `op: not` |
| `logic/or` | `gate` | `op: or` |
| `logic/pulse` | `gate` | `op: pulse` |
| `logic/receiver` | `receiver` | `band: radio`, `delay: 0.0` |
| `logic/transmitter` | `transmitter` | `band: radio`, `delay: 0.0` |
| `logic/xor` | `gate` | `op: xor` |
| `npc/flee` | `npc` | `behavior: flee`, `speed: 0.08`, `sense: 0.4` |
| `npc/follow` | `npc` | `behavior: follow`, `sense: 0.8`, `keep: 0.3` |
| `npc/guard` | `npc` | `behavior: guard`, `sense: 0.4`, `reach: 0.25` |
| `npc/lure` | `npc` | `behavior: lure`, `sense: 0.8`, `keep: 0.25` |
| `npc/patrol` | `npc` | `behavior: patrol`, `path: [[0.0, 0.0], [0.4, 0.0]]`, `sense: 0.35` |
| `npc/seek_button` | `npc` | `behavior: seek_button` |
| `npc/shy_gaze` | `npc` | `behavior: shy_gaze`, `keep: 0.3` |
| `npc/sleepy` | `npc` | `behavior: sleepy`, `path: [[0.0, 0.0], [0.4, 0.0]]`, `awake: 2.0`, `asleep: 2.0` |
| `object/ball` | `ball` | `radius: 0.05`, `mass: 0.05`, `elasticity: 0.5` |
| `object/bowling_pin` | `pin` | `radius: 0.02`, `height: 0.12`, `mass: 0.03` |
| `object/heavy_ball` | `ball` | `radius: 0.08`, `mass: 0.5`, `elasticity: 0.2` |
| `trap/cannon` | `cannon` | `aim: at_body` |
| `trap/cannon_turret` | `cannon` | `fire: edge`, `aim: signal`, `spread: 0.0`, `burst: 1` |
| `trap/crosswind` | `wind` | `size: [0.6, 1.0]`, `height: 0.4`, `yaw: 90.0`, `power: 1.0` |
| `trap/pendulum` | `pendulum` | `height: 0.6`, `length: 0.45`, `power: 40.0`, `period: 1.6` |
| `trap/pit` | `pit` | `size: [0.3, 0.3]`, `on_fall: fail` |
| `trap/pusher` | `pusher` | `size: [0.3, 0.1]`, `stroke: 0.3`, `power: 0.3` |
| `trap/slippery_floor` | `floor_tile` | `size: [0.4, 0.4]`, `friction: [0.2, 0.005, 0.0001]` |
| `trap/spinning_bar` | `spinner` | `length: 0.4`, `height: 0.03`, `power: 90.0` |
| `trap/sticky_floor` | `floor_tile` | `size: [0.4, 0.4]`, `friction: [2.0, 0.02, 0.01]` |

### 조립 프리셋

- `puzzles/ball/ball_ramp`
- `puzzles/ball/bowling`
- `puzzles/ball/color_sort`
- `puzzles/ball/mini_soccer`
- `puzzles/coins/coin_lock`
<!-- mpg parts --markdown: end -->
