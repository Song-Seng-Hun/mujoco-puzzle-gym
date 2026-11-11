# 부품과 프리셋 안내

[사용 안내로 돌아가기](guide.md)

퍼즐 방은 부품을 조립해 만듭니다. 이 문서는 부품이 신호로 이어지는 방식, 프리셋으로 방을 짜는 순서, 점검과 단계별 학습, 그리고 모든 부품의 값 표를 다룹니다. 값 표는 `mpg parts --markdown`이 만든 것이고, 나머지는 사람이 쓴 설명입니다.

## 부품, 프리셋, 조립 프리셋

- **부품**은 `room.yaml`의 `parts:` 한 줄입니다. 종류(`type`)와 값들로 이루어집니다. 종류마다 받는 값과 단위, 기본값, 허용 범위가 정해져 있고(값 스키마), 범위 밖 값은 `mpg build`와 `mpg check`가 이유와 범위를 들어 거절합니다.
- **부품 프리셋**은 부품 종류 하나와 기본값 묶음입니다. 코드는 없습니다. `src/mujoco_puzzle_gym/presets/parts/<분류>/<이름>.yaml`에 있고, 분류는 `input`(버튼·센서), `logic`(게이트·카운터·트랜시버), `device`(문·워프·골대), `floor`, `trap`, `object`, `beak`, `npc`입니다. `beak`과 `npc`는 아직 비어 있습니다.
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
| `parts` | 부품별 점검(골대 입구보다 큰 공, 닿을 수 없는 공·코인) |
| `overlap` | 기본 배치에서 부품의 단단한 몸체가 벽이나 다른 부품에 5 mm 넘게 박혔는지. 바닥 패드·깃발·골 구역은 빠지고, 문은 자기 벽(문이 막는 틈의 벽, 같은 선 위의 벽)과만 겹쳐도 됩니다 |
| `order` | 신호와 벽으로 볼 때 골까지 가는 풀이 순서가 있는지. 글자 지도 방은 부품이 바닥 칸 위에 있는지와 막는 턱·경사도 |

앞의 세 단계(`values`, `channels`, `demo`)는 방을 읽는 중에 실패하므로, 그때 결과에는 실패한 단계만 있고 해답 단계는 건너뜁니다. 정적 단계만 보려면 `--static`을 붙입니다.

동적 단계:

| 단계 | 보는 것 |
|---|---|
| `stability` | 명령 0으로 10초 동안 넘어지거나 계산이 터지지 않는지 |
| `solver` | 정답을 아는 스크립트가 무작위 배치 20개 중 18개 이상을 푸는지. 방에 `demo:`(`goto`, `press`, `wait`, `push <공> to <골대>`)가 있으면 그 순서를 먼저 따르고, 나머지는 격자 해답이 이어 갑니다 |

해답 단계는 지나간 깃발마다 저장본을 `runs/check/<방>/checkpoints/<깃발>.pkl`에 남깁니다.

## 체크포인트와 단계별 학습

깃발(`input/checkpoint_flag`, `order: 1, 2, …`)이 있는 방은 구간으로 나뉩니다. 먼저 `mpg check`로 깃발 저장본을 만들고, `mpg train --curriculum forward`로 구간을 하나씩 엽니다. 자세한 사용법은 [사용 안내의 단계별 학습](guide.md#단계별-학습과-구간-평가)에 있습니다.

```bash
mpg check examples/rooms/four_flags.yaml
mpg train task.yaml --minutes 60 --curriculum forward
mpg eval runs/<id> --full --segments
```

## 잡기는 아직 없습니다

큐브, 부리로 집기, 벽 버튼, 부리 장치(`beak` 분류)는 다음 묶음에서 들어옵니다. 그때의 잡기는 단순화한 것입니다. 부리 끝이 물체에 닿은 상태에서 잡기 명령을 내리면 물체를 부리 끝에 붙이고(용접 구속), 손가락 접촉이나 미끄러짐은 흉내 내지 않습니다.

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
| `mount` | enum |  | `floor` | `floor` |  | floor (wall buttons: E3) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  | plate size [x, y] |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |

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

### `laser`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `to` | vec | m | 필수 |  |  | receiver position [x, y] |
| `height` | float | m | `0.05` | [0.01, 0.5] |  | beam height |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  |  |

### `pin`

역할 `switch`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.02` | [0.01, 0.1] |  |  |
| `height` | float | m | `0.12` | [0.03, 0.5] |  |  |
| `mass` | float | kg | `0.03` | [0.005, 2] |  |  |
| `down_angle` | float | deg | `45.0` | [5, 90] |  | tilt from upright that counts as down |
| `color` | enum |  | `white` | `white`, `red`, `blue`, `green`, `yellow`, `orange`, `purple`, `black` |  |  |

### `pressure`

역할 `sensor`, 갱신 단계 `input`.

| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  |  |
| `min_mass` | float | kg | `0.1` | [0.001, 100] | 예 | mass needed to turn on |
| `trigger` | enum |  | `object` | `body`, `object`, `npc`, `any` |  |  |

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
| `shape` | enum |  | `box` | `box`, `plate` |  |  |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 2] |  |  |
| `wall_height` | float | m | `0.06` | [0.01, 0.5] |  |  |
| `wall_thickness` | float | m | `0.01` | [0.005, 0.1] |  |  |
| `accept` | list |  | `[]` |  |  | object names or kinds that count (empty: any) |
| `count` | int |  | `1` | [1, 100] |  | require count: objects needed |
| `mass` | float | kg | `0.1` | [0.001, 100] |  | require mass |
| `order` | list |  | — |  |  | require order: names in the order they go in |
| `tolerance` | float | m | `0.05` | [0.005, 1] |  | plate: position error |
| `angle_tolerance` | float | deg | `15.0` | [0, 180] |  | plate: heading error |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | plate: wanted heading |
| `angle_period` | float | deg | `90.0` | [1, 360] |  | plate: headings this far apart look the same |
| `wrong` | enum |  | `ignore` | `ignore`, `eject`, `lock`, `fail` |  |  |
| `eject_speed` | float | m/s | `1.0` | [0.1, 5] |  |  |
| `close_on_ok` | bool |  | `False` |  |  |  |

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
| `mount` | enum |  | `floor` | `floor` |  | floor (wall buttons: E3) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  | plate size [x, y] |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |
| `latch` | bool |  | `True` |  |  | M1: stay on after the body leaves (oneshot); false: momentary |

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
| `device/goal_net` | `goal_net` | `width: 0.36`, `depth: 0.2`, `height: 0.12` |
| `device/oneshot_door` | `door` | `mode: oneshot`, `width: 0.5`, `axis: y` |
| `device/reset_pad` | `warp` | `to: reset`, `size: [0.25, 0.25]` |
| `device/sliding_door` | `door` | `mode: follow`, `width: 0.5`, `axis: y` |
| `device/swing_door` | `swing_door` | `mode: follow`, `width: 0.5`, `axis: y`, `angle: 90.0` |
| `device/trapdoor` | `trapdoor` | `size: [0.3, 0.3]`, `hinge_side: -x` |
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
| `object/ball` | `ball` | `radius: 0.05`, `mass: 0.05`, `elasticity: 0.5` |
| `object/bowling_pin` | `pin` | `radius: 0.02`, `height: 0.12`, `mass: 0.03` |
| `object/heavy_ball` | `ball` | `radius: 0.08`, `mass: 0.5`, `elasticity: 0.2` |
| `trap/slippery_floor` | `floor_tile` | `size: [0.4, 0.4]`, `friction: [0.2, 0.005, 0.0001]` |
| `trap/sticky_floor` | `floor_tile` | `size: [0.4, 0.4]`, `friction: [2.0, 0.02, 0.01]` |

### 조립 프리셋

- `puzzles/ball/ball_ramp`
- `puzzles/ball/bowling`
- `puzzles/ball/color_sort`
- `puzzles/ball/mini_soccer`
- `puzzles/coins/coin_lock`
<!-- mpg parts --markdown: end -->
