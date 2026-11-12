# Plan: mujoco-puzzle-gym 마일스톤 2 (부품 카탈로그)
Status: approved 2026-11-12 (E2 split into E2a/E2b/E2c: "승인"); approved 2026-11-11 (사용자 "전부 승인": R31-R36 + R2b + review fixes rounds 2-4; coin step moved to E1 as 5b per user "2번으로")
View (read-only copy): (아직 없음)

근거: 같은 폴더의 spec.md(draft), 선행 `../2026-11-10-duck-puzzle/`. 이 문서만 보고도 만들 수 있게 쓴다.

## 0. 전제
- 시작 조건: 다음 셋이 끝난 뒤 그 위에 쌓는다.
  - M1의 PR C(`feat/train-eval`)
  - M1의 PR D(문서, CI)
  - 사용자의 병합
- 브랜치는 PR마다 새로 만든다: `feat/m2-signals`, `feat/m2-devices`, `feat/m2-grab`. push와 PR은 매번 사용자 승인을 받고, 병합은 사용자가 한다.
- 이 spec 폴더는 첫 PR에 `docs/specs/2026-11-11-parts-catalog/`로 옮긴다.
- 경로: 아래 `mujoco_puzzle_gym/...`는 `src/mujoco_puzzle_gym/...`이다.
- 새 의존성은 없을 것으로 본다. MuJoCo 4.0.0의 mocap, `eq_active`, `mj_ray`, `mj_getState`/`mj_setState`를 쓴다.

## Files that change
| File | Change | R |
|---|---|---|
| `mujoco_puzzle_gym/signals.py` | 새로. 채널 버스, 평가 순서(입력→논리→장치), 순환 검사 | R1 |
| `mujoco_puzzle_gym/expr.py` | 새로. `ast` 허용 목록 검사, 상태를 갖는 함수(delay, hold, pulse, latch, count, timer) | R7 |
| `mujoco_puzzle_gym/schema.py` | 새로. 값 스키마(단위, 기본값, 범위, 실행 중 변경 가능), 검사, 마크다운 표 생성 | R3, R23 |
| `mujoco_puzzle_gym/presets/parts/<분류>/*.yaml` | 새로. 프리셋. 분류: input, logic, device, floor, trap, object, beak, npc (spec R3) | R3 |
| `mujoco_puzzle_gym/parts/base.py` | `in/out/enable`, `schema`, 런타임 변경 훅 추가 | R1, R3, R13 |
| `mujoco_puzzle_gym/parts/input/{button,pressure,zone,laser,receptacle,target,checkpoint}.py` | 새로. 소켓·걸이·바구니는 `receptacle` 프리셋 | R4–R6 |
| `mujoco_puzzle_gym/parts/logic/{gate,counter,transceiver}.py` | 새로. M1 `switch.py`는 `button` 프리셋 `floor_momentary`의 별칭으로 남긴다 | R7, R8, R2 |
| `mujoco_puzzle_gym/parts/device/{door,trapdoor,platform,pillar,turntable,conveyor,setter,warp,spring}.py` | 새로. `setter.py`에 `bind`(R13). M1 `door.py`는 이 파일로 옮기고 별칭을 남긴다 | R9, R11–R15 |
| `mujoco_puzzle_gym/parts/trap/{wind,pusher,spinner,pendulum,pit,cannon}.py`, `parts/pool.py` | 새로. 대포는 탄 풀을 디스펜서와 공유 코드로. `cannon_turret` 프리셋과 탄도 정적 점검 | R16 |
| `mujoco_puzzle_gym/parts/floor/tile.py` | 새로. 마찰·기울기(경첩+구동기)·높이(슬라이드+구동기), 신호로 두 값 사이 이동 | R10 |
| `mujoco_puzzle_gym/parts/object/cube.py` | 새로 | R17 |
| `mujoco_puzzle_gym/parts/npc/{rover,behaviors}.py`, `presets/parts/npc/*.yaml` | 새로. 평면 관절 로봇, 행동 규칙 8종, 광선 벽 회피 | R29 |
| `mujoco_puzzle_gym/parts/device/addon_station.py`, `presets/parts/device/addon_roller.yaml`, `bodies/microduck.py`(모드 전환·관절 이름 매핑), 자산 받기(바퀴 메시·`roller.onnx`·`roller_crouch.onnx` 고정 커밋+해시) | 새로/고침. 합친 모델, `set_addon`(높이 → 속도 0 → 충돌·정책), 정책 교체 | R30 |
| `mujoco_puzzle_gym/presets/puzzles/roller/*.yaml` | 새로. 바퀴 퍼즐 조립 프리셋 9종 | R30 |
| `mujoco_puzzle_gym/parts/device/rope.py`, `parts/device/hitch_station.py`, `presets/parts/device/{rope,anchor,hitch_station}.yaml`, `bodies/microduck.py`(`tow_site`) | 새로/고침. 매듭 몸체 + 길이 제한 tendon, 걸이별 `connect` 구속, 채널·패드·부리 연결, 끊어짐, `length` 실행 중 값 | R31 |
| `mujoco_puzzle_gym/presets/puzzles/tow/*.yaml`, `examples/rooms/tow_yard.yaml` | 새로. 견인 퍼즐 조립 프리셋 8종과 예제 방 | R31 |
| `mujoco_puzzle_gym/parts/npc/owner.py`, `parts/input/zone.py`·`parts/input/receptacle.py`(`attach_to`), `presets/puzzles/pet/*.yaml`, `examples/rooms/walkies.yaml` | 새로/고침. 주인 경로·기다리기·명령, 움직이는 구역, 동행 프리셋 8종 | R32 |
| `mujoco_puzzle_gym/parts/logic/source.py`, `parts/logic/wire.py`, `parts/logic/base.py`(`body: true`) | 새로/고침. 신호기 4방식, 보이는 전선과 연결 끊김, 논리 부품 몸체 | R33 |
| `mujoco_puzzle_gym/parts/breakable.py`, `parts/device/destroyer.py`, `parts/logic/relay.py`, `check/circuit.py`, `presets/puzzles/circuit/*.yaml`, `examples/rooms/power_cut.yaml` | 새로. 파괴 속성(원인 4가지, hp, 고치기), 파괴기, 중계기, 회로 그래프 점검, 회로 프리셋 8종 | R34 |
| `mujoco_puzzle_gym/randomize.py`, `env.py`(`sample_layout` 교체, 쉬는 구역), `room.py`(샘플러 스키마, `count`, `holdout`, M1 `random:` 읽기), `check.py`(다시 뽑기 비율), `obs.py`(사용 중 마스크, 연결 비트, `randomized`), `task.py`(`randomize:`), `curriculum.py`(`randomize.<층>` 난이도, 저장본 시작 재뽑기), `evaluate.py`(`--randomize`), `examples/rooms/shuffle_switches.yaml` | 새로/고침. 샘플러 5종, 층별 세기, 다시 뽑기(그래프 순서 + 뽑힌 값 정적 점검), 연결 단서, 평가 전용 값, 뽑힌 방 기록 | R35 |
| `mujoco_puzzle_gym/parts/floor/coin.py`, `puzzles/coins/coin_lock.yaml`, `obs.py`(`coins`), `rewards.py`(`coin_pick`), `check.py`(코인 순서, 닿지 않는 코인), `examples/rooms/coin_room.yaml` | 새로/고침. 코인, 묶음 채널, 코인 잠금 프리셋 | R36 |
| `mujoco_puzzle_gym/body.py`(`capabilities`), `bodies/microduck.py`(기본 능력), `schema.py`(`requires_body`), `check.py`(능력 정적 점검) | 고침. 능력 태그 자리와 검사만, 새 몸 없음 | R2b |
| `mujoco_puzzle_gym/parts/device/rope_door.py` | 새로. 중력으로 닫히는 수동 문(R31 `cleat_hold`) | R31 |
| `mujoco_puzzle_gym/parts/beak/{grabbable,pull_cord,dial,drawer,lever,dispenser,scale}.py`, `presets/parts/beak/*.yaml` | 새로. 집을 물체 공통 클래스(질량 한도, 깨짐), 부리 장치 | R28 |
| `mujoco_puzzle_gym/markers.py`, `mujoco_puzzle_gym/presets/markers/4x4_50/*.png` | 새로. 마커 번호 배정, 텍스처 박기, 번호 검사 | R26 |
| `mujoco_puzzle_gym/parts/input/gaze.py`, `obs.py`의 `markers_seen` | 새로. 시야각·거리·면 방향·가림 판정, dwell | R27 |
| `mujoco_puzzle_gym/room.py` | 프리셋 읽기·덮어쓰기, `links:`→채널 변환, 바닥 타일, `stages:`, `demo:`, `checkpoint_states:` | R2, R3, R10, R14, R20, R21 |
| `mujoco_puzzle_gym/logic.py` → `walkmap.py` | 이름 변경. 격자 길 찾기는 그대로 | R20 |
| `mujoco_puzzle_gym/solver.py` | `demo:` 하위 목표 실행기(goto, press, grab, place, wait) | R20 |
| `mujoco_puzzle_gym/check.py` | 정적 단계와 동적 단계, 저장본 기록 | R20 |
| `mujoco_puzzle_gym/snapshot.py` | 새로. `mj_getState(mjSTATE_INTEGRATION)` 전체 상태(`eq_active` 포함) + 부품·신호·어댑터 이력 저장·복원 | R6, R21 |
| `mujoco_puzzle_gym/bodies/base.py` | `step(data, cmd, on_substep=None)`(물리 스텝 콜백), `teleport`, `kick`, `grab_site`, `view_frame`, `history_state`/`load_history` 추상 메서드 | R14, R15, R18, R6 |
| `mujoco_puzzle_gym/bodies/microduck.py` | 위 메서드 구현, 부리 끝 사이트, 머리 명령 연결 | R14, R15, R18, R19 |
| `mujoco_puzzle_gym/env.py`, `obs.py`, `rewards.py` | 행동 확장, 관측 `checkpoints`, 체크포인트 재시작, 다음 목표를 신호 그래프로 계산 | R6, R19, R21 |
| `mujoco_puzzle_gym/curriculum.py` | 새로. forward/reverse/none, 구간 표집, 기록 | R21 |
| `mujoco_puzzle_gym/{train,evaluate}.py`, `cli.py` | `--curriculum`, `--advance-at`, `--replay`, `eval --full/--segments`, `mpg parts` | R21, R22, R3 |
| `mujoco_puzzle_gym/registry.py` | `register_logic` 추가 | R7 |
| `examples/rooms/*.yaml`, `examples/broken/*.yaml` | 예제 6개 이상, 망가진 방 4개 | R20, R24 |
| `docs/parts-guide.md` | 새로 | R23 |
| `AGENTS.md`, `skills/mujoco-puzzle-gym/SKILL.md` | 가이드 연결, 프리셋 조립 순서 | R25 |
| `tests/test_*.py` | Proof 표의 테스트 | 각 R |

## Order of work
PR E1 `feat/m2-signals` (신호와 입력·논리, 흐름 부품)
1. `schema.py`와 `Part` 기반 확장. M1 부품 3종에 스키마를 단다. M1 테스트가 녹색인지 확인한다(R2).
1b. 몸 능력 태그(R2b): `BodyAdapter.capabilities` → 마이크로덕 기본 능력 → 부품 `requires_body` 스키마 → `mpg check` 정적 단계에서 빠진 능력 거절. 이후 단계의 부품은 필요한 능력을 이 자리에 선언한다.
2. `signals.py`를 만들고 `links:` 변환을 넣는다. `test_signals.py`.
   - 구현 기록 (2026-11-11, 커밋 f02e924·eef4d5c·fc0e88a, 100 passed incl. slow, 예제 방 solver 20/20):
     - `Capabilities`는 `bodies/base.py`에 두었다. 마이크로덕 기본 능력은 `walk`, `beak_grab`, `max_lift_mass` 0.05.
     - 문 `open_dir`은 1 또는 -1만 받는다.
     - 상태 없는 순환은 `Room.load`에서 `RoomError`로 거절한다. `mpg check`는 `ok: false`와 `error`에 순환 경로를 낸다. 정적 단계(`failed` 목록)로 옮기는 일은 6단계에서 한다.
     - `logic.py` 이름 바꾸기는 미뤘다. 부품 갱신 순서는 입력 → 논리 → 장치(R1).
     - 남은 일: `check_order`는 문을 여는 출처를 채널에 직접 쓰는 부품으로만 찾는다. 3단계에서 논리 부품을 거쳐 거슬러 올라가게 고치고 시험을 더한다(게이트를 거쳐 열리는 문이 있는 방이 order 단계를 통과).
     - 가이드의 "점검 3단계" 설명과 R2b "M2 예제 방 통과" 시험은 8단계에서 한다.
3. `expr.py`와 게이트, 카운터(R8b), 트랜시버. `test_logic.py`(거절 사례 포함), `test_counter.py`.
4. 버튼 3종(바닥형만. 벽형은 E3에서), 센서(압력판은 E3의 큐브 뒤에 시험), 체크포인트와 `snapshot.py`, 어댑터의 `teleport`와 이력 저장·복원. 소켓은 `receptacle` 부품(R5)으로 여기서 만들고, E1 시험은 시험 전용 자유 상자로 한다. 물고 있는 물체 제외 시험은 15단계에서 한다. `snapshot.py`는 처음부터 `mjSTATE_INTEGRATION`을 쓴다. 같은 단계에서 `BodyAdapter.step(..., on_substep=None)` 콜백과 `env.advance`의 부품 `substep` 묶음을 넣는다(압력판은 쓰지 않음).
   - 구현 기록 3·4단계 (2026-11-11, 커밋 caf9a19…1c277ad, 빠른 229 passed·전체 240 passed, 예제 방 solver 20/20):
     - 바닥 버튼·구역 센서는 위치로 판정한다(몸통 중심이 판 위, 물체는 중심이 `max_height` 아래). 접촉 판정 판은 발을 3.7 mm 들어 올렸다. 압력판만 4 mm 솟은 판의 접촉력을 쓴다.
     - 상자 `eject`는 물건을 벽 너머로 던진다(spec 문구 수정). 모양은 `box`·`plate`만, 걸이는 16b. `accept`의 마커 번호는 R26 뒤. `require`는 `{count: 2}` 형태도 받는다.
     - 버튼 `mount`는 E3 전까지 `floor`만. 버튼 기본 `trigger: body`.
     - 게이트는 채널 없는 `op` 형태(`{op: delay, seconds: 1}`)도 받는다. `xor`는 `or`와 같은 우선순위. 점 이름(`laps.count`)은 채널 이름이고 산술은 없다.
     - 체크포인트 복원 뒤 재생은 1초에 약 2 mm 어긋난다(이력 복원 없이는 4.3 cm). spec R6에 알려진 오차로 적음.
     - `respawn`은 task.yaml `env.respawn: start|checkpoint`. `checkpoint`면 `respawn_penalty`(-1)가 자동으로 붙는다.
     - 물체로만 켜지는 센서는 역할 `sensor`. 6단계 `demo: place` 전까지 그 뒤의 문은 순서 점검에서 실패한다. M1 `links:`는 이것에서 시작할 수 없고 `in:`을 쓴다.
     - `check_order`가 논리 부품을 거슬러 문 입력을 추적한다(c1df057, 2단계에서 넘어온 수정).
     - 한 번, 어느 테스트인지 모르는 빠른 테스트 실패 1건(이후 6회 재현 없음). 다시 나오면 테스트 이름을 남긴다.
5. 문 3종, 바닥 타일 부품(R9, R10), 워프(R14).
   - 타일 바닥에서 M1-R3 걷기 시험을 다시 한다.
   - 기울기·단차·마찰을 나눠 시험해 기본 허용 범위를 정한다(Open question 5).
   - 구현 기록 5–5d단계 (2026-11-11, HEAD 5fcf2ea, 빠른 317 passed(데스크 확인)·전체 334 passed(작업자 보고)):
     - 5단계 커밋 031e866: 문, 바닥 타일, 트랩도어, 워프 발판, `stages:`(R9, R10, R14). 예제 방 `switch_door` solver 20/20(작업자 보고). `examples/rooms/` 폴더를 이 단계에서 만들었다.
     - Open question 5 측정값과 결정은 spec에 적었다(`max_step` 0.01 m, `max_slope` 5°, 기울기 ±13°, 높이 ±0.1 m, 마찰 0.2 이상, 타일 geom `priority` 1).
     - 받아들인 차이: 여닫이 문은 벽(움직이는 판)으로 흉내 낸다. 문 속도 값은 선택이다. 워프는 최소 구현이다. 풀이 순서 격자는 트랩도어를 무시한다(6단계 할 일). 넘어짐 높이는 `BodyAdapter.state(data, floor_z=0.0)`와 `fall_height`로 잰다.
5d. 공과 골대(R38): Open question 10 측정(밀 수 있는 질량) → `ball` 스키마(탄성은 solref, 튀는 비율 시험) → `goal_net`(scored/count/accepts) → `demo:`의 `push` → 정적 점검(입구 크기, 닿지 않는 칸) → 관측 `balls`, 보상 `ball_progress` → 프리셋 4종 → 예제 `mini_soccer.yaml` → `test_ball.py`.
   - 구현 기록 (2026-11-11, 커밋 82b2232·4b68e46·0a47496, 병합 6ec9a4e, 뒤이어 5fcf2ea):
     - 공, 핀(`pin`), 골대와 부품 정적 점검 단계 `parts`(82b2232). solver 공 밀기, 관측 `balls`, 보상 `ball_progress`(4b68e46). `mini_soccer` 예제와 프리셋(0a47496). 5fcf2ea에서 공이 워프·구역·높은 바닥을 넘어 동작하게 하고 `ball_ramp`를 더했다.
     - solver: `mini_soccer` 19/20(시드 13 시간 초과, 데스크 확인). spec 기준 18/20 이상은 넘는다.
     - Open question 10 측정값과 공·골대 기본값은 spec에 적었다.
     - 받아들인 차이: `push` 명령 대신 `push.py` 자동 밀기를 쓴다(6단계 `push`가 대신함). solver는 앞에 벽이 있으면 돈다. 공 프리셋은 방 파일 통째다(8단계 프리셋 로더 전까지). 골대 벽은 튀지 않는 벽(반발 없음, 공보다 접촉 우선)이라 들어간 공이 나오지 않는다. `ball_ramp`는 R37 경사로(3.4°)로 만든다.
5c. 글자 지도(R37, spec 확정 뒤): `room.py`에 `map`/`cell`/`legend` 읽기 → 타일 바닥 생성(같은 높이 칸 합치기, 경사로) → 바닥 없음 낙하 판정 → 마이크로덕 `max_step`·`max_slope`(Open question 5 측정값) → `logic.py` 격자에 높이·경사 이동 조건 → 정적 점검의 칸 좌표 이유 → 스크립트 해답 경로 → 예제 `two_rooms_map.yaml`, `stairs_map.yaml` → `test_map.py`.
   - 구현 기록 (2026-11-11, 커밋 dba3ece):
     - solver: `two_rooms_map`, `stairs_map` 20/20(작업자 보고).
     - 받아들인 차이: `max_step`은 오를 때와 내려갈 때 모두 적용한다. 풀이 순서 격자 칸은 약 0.033 m다. 지도 점검(벽·바닥 없음 칸)은 `order` 단계에 있다. 방 값 `on_fall: reset`을 지원한다(기본 `fail`, 빠지면 에피소드를 끝내지 않고 되돌림). 지도 방과 `stages:`는 아직 함께 쓰지 않는다.
5b. 코인(R36, 사용자 2026-11-11 "2번으로"로 E3에서 E1로 당김): `coin` 부품과 묶음 채널 → `coin_lock` 프리셋(R8b 카운터 재사용) → 관측 `coins`, 보상 `coin_pick` → `check.py` 코인 순서와 닿지 않는 코인 거절 → R35 샘플러 연결 → 예제 `coin_room.yaml`.
   - 구현 기록 (2026-11-11, 커밋 52be311, 병합 16ceba8, 뒤이어 5fcf2ea):
     - 5fcf2ea에서 코인이 워프·구역·높은 바닥을 넘어 동작하게 했다.
     - solver: `coin_room` 20/20(데스크 확인).
     - 받아들인 차이: `coin_lock`은 8단계 프리셋 로더 전까지 부품 종류다. 묶음은 숨은 `coin_group` 부품이 맡는다. `.picked` 채널은 펄스 사이를 띄워 낸다. 관측 `coins`는 최대 3개(`COIN_OBS_K`). 코인 위치 샘플러는 16g로 미뤘다.
6. `check.py` 두 단계와 `solver.py`의 `demo:`(goto, press, wait). 망가진 방 4개.
   - 5–5d에서 넘어온 일: `demo:`의 `push` 명령(`push.py` 자동 밀기를 대신함). 풀이 순서 격자가 트랩도어와 구덩이를 피하게 한다. 공 밀기 여유를 늘린다(`mini_soccer` 19/20).
   - 구현 기록 (2026-11-11, 커밋 a87748d·816f1b4·60fce07, 병합 4990f57, 빠른 371 passed(병합 뒤, 데스크 확인)·전체 363/364 passed(브랜치별, 작업자 보고)):
     - solver: 예제 `switch_door`, `coin_room`, `mini_soccer`, `stairs_map`, `two_rooms_map` 모두 20/20(데스크 확인). 프리셋 `ball_ramp` 20/20, `color_sort` 19/20, `bowling` 18/20.
     - 받아들인 차이: 방을 읽을 때 나던 오류는 정적 단계 `values`, `channels`, `demo`가 되었다(출력에는 `error` 키를 남김). 정적 단계 `overlap`을 새로 넣었다(규칙은 spec R20). `demo:`는 중간에 멈출 수 있고 그 뒤는 격자 해답이 이어 간다. 골대·핀 목표는 `demo:` 없이 자동으로 민다. `push.py`는 이제 `push` 명령의 구현이고 골대에서 0.3 m보다 멀면 더 빨리 민다. `examples/switch_door.yaml`에는 `demo:`를 넣지 않았다(시험이 그 방을 복사해 씀). 일반 "바닥이 구멍" 훅을 두었고 지금은 트랩도어만 쓴다. 체크포인트 저장본은 `runs/check/<room>/checkpoints/`에 `.pkl`로 남는다.
     - 넘어가는 일: 8단계(`bowling` 핀 밀기, 가이드 점검 단계)와 12단계(구덩이 구멍 훅)에 적었다.
7. `curriculum.py`, 관측 `checkpoints`, `eval --full/--segments`. 가짜 정책 테스트.
   - 구현 기록 (2026-11-11, 커밋 4ef10fd·d927e81·cbfe027, 병합 2172f3d, 빠른 432 passed(병합 뒤, 데스크 확인)·전체 421 passed(브랜치, 작업자 보고)):
     - 짧은 실행(깃발 2개 임시 방): `--curriculum forward` 1분에 구간이 6초·9초에 열림, `eval --full --segments` 8회에서 전체 1.0, 구간 0.875/1.0/1.0, 이어 붙임 실패 0(작업자 보고).
     - 받아들인 차이: 체크포인트 깃발이 목표 순서에 들어간다(전에는 `progress` 보상이 깃발을 건너뛰어 첫 구간이 0에 머묾). 깃발이 있는 모든 방의 풀이 경로가 바뀌었고 예제 5개는 그대로 20/20(데스크 확인). 저장본이 없으면 `mpg check`를 먼저 돌리라는 오류를 낸다. `--full`은 지금 일반 평가와 같은 에피소드다. 깃발마다 저장본이 하나라 구간 에피소드 시작 상태가 같고 시드는 잡음만 바꾼다. 구간 성공은 `info["segment_success"]`, `info["success"]`는 여전히 골이다. `CurriculumVec`는 train.py에 둔다.
     - 넘어가는 일: 12단계 난이도, 16g `checkpoint_states:`, 9단계 앞 `mpg check`, guide.en.md·guide.ja.md 번역은 E1 PR.
7b. 마커(R26)와 시선 센서(R27). `view_frame`은 `head_camera`를 쓴다. 머리 숙이기 없이 몸을 돌려 보는 경우만 시험한다.
   - 구현 기록 (2026-11-11, 커밋 69b4d50·8fb10ae, 병합 ebe5c1f, 빠른 432 passed·1 skipped(병합 뒤, 데스크 확인)·전체 426 passed(브랜치, 작업자 보고)):
     - 예제 5개 solver 20/20(병합 뒤, 데스크 확인). 마커 PNG 50개는 `tools/make_aruco_markers.py`로 만들었고 OpenCV `generateImageMarker`와 같음(작업자가 시험 밖에서 확인).
     - OpenCV는 개발 의존성으로 넣지 않는다(사용자 2026-11-11 "응 승인"). 검출 시험은 cv2가 없으면 건너뛰고, 렌더한 마커 칸을 직접 읽는 시험이 대신 돈다.
     - 받아들인 차이: 마커 오류 3가지는 `values` 단계에서 거절한다. 디스펜서가 아직 없어 복제 예외는 `marker_clone_of` 속성으로 둔다. 마커 부품 9종(button, pressure, checkpoint, door, swing_door, trapdoor, receptacle, warp, floor_tile), M1 `switch`는 8단계 별칭 뒤, 큐브·플랫폼은 그 단계에서. `size`는 검은 사각형이고 흰 테두리 한 칸을 더한다. 문은 기본 `face: both`. 방 값 `gaze: {fov: 60, max_dist: 2.0, max_tilt: 70, dwell: 0.5}`(서 있는 몸이 바닥 마커를 보도록 70°). 표시된 부품 자신과 몸의 geom은 광선을 막지 않는다. `.seen`은 숨은 `_gaze` 부품이 쓰고 읽는 부품이 있을 때만 생긴다. `markers_seen` 슬롯은 (seen, x, y, nx, ny) 진행 방향 기준이고 센서 항목으로 센다.
     - 넘어가는 일: `look` 명령은 16b(사용자 2026-11-11 "응 승인"). README 문장은 E1 PR. guide.en.md·guide.ja.md 번역은 E1 PR.
7c. 실물 센서 관측(R39): (5b·5d에서 넘어온 일: task.yaml에서 관측 항목에 값을 주는 방법, 예: 코인 수, 공 수.) Open question 11 측정(카메라 렌더 비용) → `obs.py`에 `rangefinder`(어댑터가 머리 사이트에 센서 추가)와 `camera`(환경당 렌더러 1개, 결정 스텝에만 렌더) → 관측 공간 Box/Dict 전환 → `train.py`에서 Dict면 `MultiInputPolicy` → `critic_obs`용 비대칭 정책 클래스(가치망 입력만 따로) → R35 `body` 잡음 연결 → `mpg bench` 카메라 수치 → `test_sensor_obs.py`. README·가이드 문장은 구현 확인 뒤 같은 PR에서 고친다.
   - 구현 기록 (2026-11-11, 커밋 54db102·d628d19·5bbecc9, 병합 9e5e516, 빠른 371 passed(병합 뒤, 데스크 확인)·전체 363/364 passed(브랜치별, 작업자 보고)):
     - Open question 11 측정값과 결정(64×64, 10 Hz, 그림자·반사 끔)은 spec과 `docs/bench.md` 3차 측정에 적었다.
     - 짧은 실행: `mpg train --minutes 1`을 센서만 설정과 `critic_obs` 설정으로 각각 돌려 정책을 저장했고, `mpg eval`이 돌았다(성공 0, 예상대로).
     - 받아들인 차이: spec R39 "구현에서 정한 것"에 적었다(카메라 렌즈 축, 수평 광선, uint8 영상, 카메라 항목 하나, sac 거절, `noise`와 `randomize: {body: k}`).
     - 7b가 다시 쓰도록 `BodyAdapter.view_camera`, `view_frame`, `add_rangefinders`를 더했다.
     - 남은 일: README 문장 고침은 E1 PR에서 한다.
8. 프리셋 폴더, `mpg parts`, 가이드 1차, 예제 방(버튼·논리, 문, 체크포인트 4개). M1 `switch` → `oneshot` 별칭(spec R2).
   - 5b·5d에서 넘어온 일: `coin_lock`과 공 프리셋을 프리셋 로더로 잇는다. 문서에 `ball_ramp`를 더한다.
   - 6단계에서 넘어온 일: 프리셋을 예제 방으로 옮길 때 `bowling` 핀 밀기를 다듬는다(18/20이 R20 기준선). `docs/guide.md`에 새 점검 단계를 적는다. `overlap` 단계가 문(`role == "gate"`)을 모든 벽에서 빼는 것을 자기 벽만 빼도록 좁힌다(사용자 2026-11-11 "응").
   - 구현 기록 (2026-11-11, 커밋 3b80bfe·7fbc715·5fe807b, 병합 358ac50, 빠른 499 passed·1 skipped(병합 뒤, 데스크 확인)·전체 529 passed(브랜치, 작업자 보고)):
     - solver: 새 예제 `logic_lock`, `doors`, `four_flags` 20/20, 프리셋 `bowling` 20/20(병합 뒤, 데스크 확인). 기존 예제 5개·`ball_ramp` 20/20, `color_sort` 19/20(작업자 보고). `bowling`은 핀 0.3 m 너머를 겨누게 해 18/20 → 20/20(평균 21.3 s → 24.0 s).
     - 받아들인 차이: 조립 프리셋은 방 맨 위 `preset: puzzles/<topic>/<name>`(이름 하나나 목록)으로 읽고, 부품은 이름으로 합치며 `links`는 더한다. `coin_lock`은 `need: all` 코드가 필요해 부품 종류로 남기고 `puzzles/coins/coin_lock` 프리셋이 그것을 쓴다. 공 프리셋은 방 파일 그대로 `presets/puzzles/ball/`에 둔다. `overlap`은 문이 자기 줄의 벽(5° 안 평행, 중심선 위)에서만 빠진다. 이로 드러난 `ball_ramp` 문 1 cm 파고듦을 고쳤다(폭 0.5 → 0.48). `switch`는 버튼 하위 종류라 마커와 버튼 색을 갖는다. 바뀐 값은 task.resolved.yaml `room_parts`에 남긴다. `doors`는 oneshot 버튼만 쓴다. `mpg parts --plugin <module>`을 더했다.
     - 넘어가는 일: 13단계 `mpg demo` 풀이 MP4(R24), 12b단계 momentary 버튼 → oneshot 문 계획, 17단계 AGENTS.md·SKILL.md(R25), E1 PR에서 README·guide/parts en·ja 번역과 parts.md "다음 부품" 단락.
9. 데모 방 `four_flags`에서 forward 학습 1회. 목표는 Open question 3에서 정한 값이다. 학습 조정은 subagent에 맡긴다. (7단계에서 넘어온 일: 학습 전에 `mpg check`로 깃발 저장본을 먼저 만든다.)
   - 구현 기록 (2026-11-11, 커밋 bb6f0c6, 병합 e22d521): 학습 설정 `examples/task_four_flags.yaml`(PPO, 7 env, 에피소드 60 s, 관측에 `checkpoints`), `forward`·`none`을 같은 때 2시간씩(작업자 보고).
     - 결과(평가 100회, 결정적): `forward` 전체 1.00·구간 모두 1.00·이어 붙임 실패 0·평균 19.7 s, `none` 1.00·1.00·0·15.3 s. 목표 달성, 다시 돌리지 않음. 기록은 `docs/training-log.md`.
     - 해석: 깃발이 풀이 순서라 `target_pos`와 진행 보상이 길을 알려 주어 `none`도 약 1분에 80%를 넘었다. 1분 시험에서 `forward`는 결정적 평가 첫 구간이 0.02(학습 중 0.66–0.8)였고 원인은 보지 않았다.
     - 넘어가는 일: 단계별 학습 이득 비교는 12단계 난이도 방에서. 정책 파일은 저장소 `runs/ff-fwd-120`, `runs/ff-none-120`(git 밖).

PR E2는 셋으로 나눈다(2026-11-12): E2a `feat/m2-devices` 10·11단계와 그 예제 방, E2b `feat/m2-traps` 12단계와 함정 예제 방, E2c `feat/m2-npc` 12b·13단계.
10. 플랫폼·엘리베이터, 승강 기둥, 회전판(관절과 위치·속도 액추에이터, R11·R12). 컨베이어는 띠 위의 몸과 물체 위치를 옮긴다(OQ1 (c)). 몸이 띠와 플랫폼 위에서 3초 넘어지지 않는지는 slow 시험으로 기록한다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-devices`, 빠른 571 passed·1 skipped(작업자 보고)):
     - 플랫폼 `path`는 `pos`에서 잰 `[dx, dy, z]`다(z는 바닥 위 윗면 높이). 프리셋(엘리베이터)이 경로를 가질 수 있고 `random`이 경로째 옮긴다.
     - 미끄럼 관절은 경로가 움직이는 축에만 둔다(수직 엘리베이터는 z 하나). 움직이지 않는 축은 관절 없이 고정이다.
     - 구동기 목표를 몸 스텝(20 ms)마다 한 번 옮기면 판이 덜컥거려 0.3 m/s, 3초에 큐브가 3.1 cm 미끄러졌다. 물리 스텝마다 목표를 이어 옮기도록(`substep`) 바꿔 0.0 cm가 되었다(`tests/test_platform.py`).
     - `run: on_signal`은 상승 순간마다 한 번 운행한다(once: 끝까지, pingpong: 반대 끝까지, loop: 한 바퀴). 엘리베이터 프리셋은 `pingpong` + `on_signal`이다.
     - 컨베이어 띠와 스프링 패드는 바닥에 그린 것이고 충돌하지 않는다. 옮기기·쏘기는 `World.requests`의 `move`·`kick`으로 env가 몸 스텝 뒤에 한다. 몸을 옮기려고 `BodyAdapter.shift`도 더했다.
     - 계획(풀이 순서, 해답)은 플랫폼이 쓸고 가는 칸과 회전판 칸, 솟을 기둥 칸을 막힌 칸으로 본다.
     - slow 기록(통과 기준 아님): 플랫폼 위 몸 0.1 m/s 3초 넘어지지 않음(판 0.296 m, 몸 0.295 m 이동), 0.3 m/s 넘어지지 않음(판 0.888 m, 몸 0.800 m). 띠 위 몸 0.1 m/s 3초 넘어지지 않음(0.300 m), 0.2 m/s 넘어지지 않음(0.600 m).
11. 스프링과 어댑터의 `kick`, 설정 변경기.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-devices`):
     - 실행 중 변경 값은 부품의 `live`에 두고 부품이 매 스텝 거기서 읽는다. `set_value`로 바꾸고, `reset`이 방 값으로 되돌리며, 저장본(`Snapshot.values`)이 담는다.
     - `bind`는 설정 변경기만의 값이 아니라 모든 부품의 공통 값이다(`bind: {<값>: {from, map}}`, spec R13 예시 형태). 버스가 그 부품의 `update` 직전에 적용한다. 수 값만 묶을 수 있다(스프링 `velocity` 같은 벡터는 거절).
     - 설정 변경기는 자기 채널(적용 중이면 1)을 내고 대상이 그 채널을 읽게 해서, 같은 몸 스텝에 대상보다 먼저 갱신된다. 순서를 두지 않았을 때 스프링이 바뀌기 전 속도로 쏘았다.
     - 실행 중 변경이 아닌 값(문 `width`), 범위 밖 값, `bind`와 설정 변경기가 함께 바꾸는 값은 방을 읽을 때 `values` 단계에서 거절한다(별도 정적 단계를 두지 않음). `add_room_parts` 오류의 단계를 `values`로 정했다.
     - 바닥 타일의 `tilt`·`height`를 설정 변경기나 `bind`가 바꾸면 그 관절을 만든다(`claim_value`). 압력판 `min_mass`도 `live`에서 읽는다.
     - R13 Check의 "바람 세기"는 바람(R16)이 E2b에서 들어올 때 시험한다.
     - 스프링 측정: 큐브 발사 속도는 설정값과 같고(±5% 안), 착지 거리는 탄도 계산의 1.002배·1.009배(속도 [1, 0, 2], [0.6, 0.4, 1.5]).
     - 기본 `spring`은 `trigger: any`, 프리셋 `device/spring_pad`는 `trigger: object`다(몸 발사는 학습 분포 밖).
     - 예제 `examples/rooms/lift_spring.yaml`(버튼 → 문, 엘리베이터, 설정 변경기로 스프링 켜기) `mpg check` solver 20/20.
   - 리뷰 반영 (2026-11-12, claude-mac-reviewer, 빠른 575 passed·1 skipped(데스크 확인)):
     - 받음: 높은 칸 위 플랫폼이 reset마다 z를 잃던 것(`place`가 x, y만 씀), R13 전에 저장한 저장본에 `values`가 없어 복원이 깨지던 것(방 값으로 되돌림), 타일 `on_tilt`·`on_height`의 `bind`가 관절을 만들지 않던 것, 복원 뒤 계획이 지난 `place`의 이동량을 쓰던 것(`xy - base_xy`로 계산). 각각 시험을 더했다.
     - 버림: 설정 변경기가 자기 입력 판의 `min_mass`를 바꾸면 loop 오류가 나는 것(드문 설계이고 안전하게 막힌다), 입력 단계 대상이 한 몸 스텝 늦게 바뀌는 것(20 ms, 고장 아님).
     - spec R11 문구를 코드에 맞췄다(움직이는 축에만 관절, `path`는 `pos` 기준 오프셋; 사용자 "승인").
12. 물체 보관 선반 코드(`pool.py`, 대포·디스펜서 공용)를 먼저 만든다. 함정 6종(대포 포함). 탄에 맞을 때 넘어지는 세기를 재서 대포 기본값을 정한다. `curriculum.py`에 난이도 단계(R21 `difficulty`)를 더한다. (6단계에서 넘어온 일: 구덩이가 "바닥이 구멍" 훅을 써서 풀이 경로가 피하게 한다.) `cannon_turret`, R13 `bind`, 과녁(R5 `target`)과 탄도 정적 점검(조준 채널 범위 기준)도 여기서 만든다. 레버 칸 기준 점검의 시험은 16b에서 한다. (7단계에서 넘어온 일: 난이도는 `Curriculum.levels`에 `{"segment": i, "difficulty": k}`로 붙인다. `eval --full`이 가장 어려운 난이도를 고르게 한다.)
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-traps`, 빠른 620 passed·1 skipped(작업자 보고), slow 넘어짐 기록 5개·`trap_run` 점검 통과):
     - `parts/pool.py`: 미리 만든 공 `pool`개를 방 아래(z −5 m)에 세워 두고 돌려 쓴다. 세워 둘 때는 충돌 비트 0과 `body_gravcomp` 1, 쏠 때 둘 다 켠다. 둘 다 `snapshot.MODEL_FIELDS`에 있고, 어느 공이 언제 나갔는지는 부품 `save_state`에 담는다. 공 geom은 충돌하는 상태로 짓고 `reset`에서 세운다. 짓자마자 비트를 0으로 두면 걷는 몸 컨트롤러가 처음 한 번 부르는 `mj_setConst`가 몸체 충돌 비트를 0으로 굳혀 공이 바닥을 뚫었다.
     - 함정 6종(`wind`, `pusher`, `spinner`, `pendulum`, `pit`, `cannon`)과 프리셋 `trap/crosswind`, `pusher`, `spinning_bar`, `pendulum`, `pit`, `cannon`, `cannon_turret`. 공통 값 `enabled`(실행 중 변경)와 `power`. 구덩이, 회전 막대·진자가 쓸고 가는 자리, 밀대가 나오는 자리는 "바닥이 구멍" 훅으로 계획이 피한다. 미끄러운·끈적한 바닥 프리셋은 전부터 있었다.
     - 탄은 `World.kinds`에서 `shell`이라 버튼·압력판·구역의 `trigger`에 들지 않는다. 과녁과 R34용 훅은 `pool.Impacts`(탄이 닿기 시작한 물리 스텝과 그 직전 속도)다.
     - 넘어짐 기록(서 있는 몸, 0.4 m, 옆/앞 각 3번, 넘어짐/맞음, slow `test_knockdown_record`):

       | 탄 kg | 3 m/s | 4 m/s | 5 m/s | 6 m/s | 8 m/s |
       |---|---|---|---|---|---|
       | 0.02 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 |
       | 0.05 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 |
       | 0.1 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 3/3 | 3/3 3/3 |
       | 0.2 | 0/3 0/3 | 1/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 |
       | 0.4 | 0/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 |

       (리뷰 반영 때 겨눔을 포구 끝에서 계산하도록 고친 뒤 다시 잰 표. 고치기 전에는 0.2 kg 4 m/s가 6/6이었다.) 0.5 N·s 이하는 한 번도 넘어지지 않았고, 0.6 N·s(0.1 kg, 6 m/s, 앞)부터 넘어지기 시작한다. 대포 기본값은 0.2 kg, 5 m/s(1.0 N·s, 6/6 넘어짐). 걷는 중에는 더 약하다: 0.4 N·s 탄을 2.5초마다 쏜 예제 방 초안은 solver 14/20(6번 넘어짐)이라 예제에서 대포를 뺐다.
     - 과녁 `target`(R5): 탄만 세고 `min_speed` 이상일 때 `<이름>.hits`를 올린다. `mode: latch`(기본) 또는 `pulse`. 포탑 정적 점검은 `parts` 단계에서 `aim: signal` 대포마다 `bind` 출력 범위의 겨눔 격자(0.25° 간격, 원천이 `channel_cells`를 주면 그 칸)로 포물선을 계산하고, `spread` > 0이면 원뿔 가장자리 8방향 모두 판에 맞아야 한다. 시험: 맞음, 뒤에 있음, 너무 멂, 원뿔 전체, 점검이 맞다는 겨눔이 시뮬레이션에서도 맞음.
     - 난이도 단계(R21): task.yaml `difficulty:` 행은 `steps: n` 또는 `at_step: k`. 실행 중 변경이나 에피소드 시작 값만, `bind` 대상은 거절. 매 `reset`이 단계 값을 넣고 시작·체크포인트 저장본에도 같은 값을 넣는다. `Curriculum.levels`가 `{"segment", "difficulty"}` 쌍이고 수준 이름은 `<구간>@d<단계>`. `curriculum.order` 기본 `difficulty_first`. 뽑기 없이(평가, 직접 `reset`) 마지막 단계, `mpg eval --difficulty k`로 고른다(`--onnx`와는 함께 못 씀). `info["difficulty"]`와 에피소드 기록에 남는다.
     - 스키마에 `Field(episode=True)`(에피소드 시작 때만 변경)를 더했다. 지금 이 표시를 쓰는 기본 부품 값은 없다.
     - 예제 `examples/rooms/trap_run.yaml`(구덩이, 회전 막대, 약한 옆바람) `mpg check` solver 20/20.
     - 계획과 다른 점:
       - 대포 세기는 `power`(발사 속도, R16)다. R21 예시의 `cannon_a.speed`는 `cannon_a.power`로 쓴다. `enabled`는 모든 함정의 값이다.
       - `shell_mass`는 지을 때만 정한다(실행 중·에피소드 시작 변경 없음). MuJoCo가 자유 물체의 질량 행렬을 캐시해서, 실행 중에 바꾸면 떨어지는 가속도가 39 m/s²로 틀렸다.
       - `in`이 없는 함정은 늘 작동한다. 구덩이에는 `power`가 없고, `in`이 있으면 꺼진 동안 뚜껑이 덮인다.
       - `at_body`·`lead`는 닿지 않는 거리면 45°로 쏜다. 포신 그림은 겨눔을 따라 돌지 않는다.
       - 레버 칸 기준 점검은 `channel_cells` 훅만 두고 시험은 16b에서 한다(계획대로).
       - R21 `param: randomize.<층>`(무작위화 세기 단계)은 R35를 기다린다.
   - 리뷰 반영 (2026-11-12, 사용자 승인 4건):
     - 난이도 단계 값은 `_reset_sim`에서 방 값을 되돌린 직후, 부품 `reset`과 계획 전에 넣는다. 타일 마찰이 첫 스텝부터 단계 값이고, 진자 계획이 단계 `power`로 쓸고 간다. 관절이 없는 타일의 `tilt`·`height` 단계는 task를 읽을 때 거절한다(`in`, 설정 변경기, `bind`를 주라는 안내). 깃발 저장본에서 시작하는 구간은 복원 뒤 단계 값을 다시 넣는다(타일 마찰은 저장본 값에서 이어 바뀐다).
     - `in`이 없는 구덩이는 늘 열려 있어서 `enable`, `enabled: 0`, `enabled`의 설정 변경기·`bind`·난이도 단계를 거절한다.
     - 시험 추가: `mpg eval --difficulty`와 `--full`의 마지막 단계, `--onnx`와 `--difficulty` 거절, `CurriculumVec`의 `@d<k>` 기록, `mode: none` + 난이도, 움직이는 표적에 `aim: lead`.
     - `lead` 시험에서 드러난 것: 겨눔을 포구 중심에서 계산해 탄이 실제로 나가는 포구 끝(0.13 m 앞, 0.09 m 위)과 어긋났다. 사거리 끝 근처(4 m/s, 1.5 m)에서 0.28 m 넘게 빗나갔다. 포구 끝을 고정점 반복으로 넣어 1 cm 안으로 맞췄다. 그래서 넘어짐 표를 다시 쟀고 기본 속도를 4 → 5 m/s로 바꿨다.
12b. NPC 로봇(R29): 몸체와 행동 8종, `demo:`의 `herd`, 예제 `herd.yaml`. `lure`와 `shy_gaze`는 E3의 잡기와 시선이 필요하므로 E3에서 시험한다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-npc`, 빠른 666 passed·1 skipped, slow `herd`·`warp_hop` 3 passed(작업자 보고)):
     - `parts/npc/rover.py`(부품 `npc`, 종류 `npc`)와 `behaviors.py`(행동 규칙 8개). 뿌리 몸체에 x·y 미끄럼 관절과 z 경첩, 속도 구동기 3개. 몸통 상자는 바닥 위 6 mm에 떠 있고 z 관절이 없다(바퀴·코는 그림만). 질량이 있어 다른 몸·물체와 부딪힌다. 마커(R26)는 `/chassis` 윗면.
     - 값: `behavior`, `size`, `mass`, `speed`(기본 0.08 m/s), `accel`, `sense`, `leash`, `keep`, `reach`, `path`, `button`, `bait`, `awake`, `asleep`, `invert`. 행동마다 필요한 값(`path`, `button`, `bait`)이 없으면 방을 읽을 때 거절한다.
     - 벽 피하기는 앞 0°, ±35°, ±70° 광선 5개(1 m). 앞이 막히면 여유가 큰 쪽부터 고른다. `leash`는 미끄럼 관절 범위(딱딱한 `solref_limit`)라 차거나 밀어도 벗어나지 않는다(2 m/s 차기에도 5 mm 안).
     - 신호 `<이름>.moving`, `<이름>.on`(버튼 위), 관측 `npcs`(로봇마다 상대 x·y, 몸 기준 상대 속도 x·y, moving, acting).
     - `demo:`의 `herd <npc> <부품 | x y>`와 `herd.py`: flee는 감지 원을 돌아 뒤쪽 선 위로 간 뒤 천천히 밀고, follow는 목표 너머 `keep`만큼 앞에서 이끈다(lure는 E3부터). 감지 범위 밖으로 처지면 기다린다.
     - 시험 `tests/test_npc.py` 25개(그중 slow 1개: `herd.yaml` 점검). lure는 E3 잡기 뒤에 시험한다.
     - 예제 `examples/rooms/herd.yaml`(도망 로봇을 구석 버튼으로 몰아 문 열기) `mpg check` solver 20/20, 풀이 25.5–30.9 s(평균 27.5 s, 에피소드 40 s).
     - 계획과 다른 점:
       - spec의 "최고 속도"와 공통 값 `speed`는 같은 값 하나다(`speed` = 최고 속도). `accel`이 그 속도까지 오르는 빠르기다.
       - `shy_gaze`는 E3를 기다리지 않는다. R27 시선 센서(`<이름>.seen`, 머무름 시간)가 이미 있어 그것을 읽고 지금 시험한다. 보지 않을 때 몸 쪽으로 `keep`까지 다가온다.
       - `lure`는 E3 잡기가 채울 `World.held`를 읽는다. 지금은 늘 비어 있어 제자리에 있다.
       - `seek_button`은 `in`이 없으면 늘 버튼으로 가고, `in`이 꺼지면 그 자리에 선다(집으로 돌아가지 않음).
       - `leash`는 로봇 중심의 범위다.
       - 압력판(`pressure`)은 `trigger`가 npc를 세면 판 안 로봇의 질량을 더한다.
       - `snapshot.MODEL_FIELDS`에 `jnt_range`를 더했다(배치 `random`이 `leash` 관절 범위를 옮긴다).
       - 워프·컨베이어·스프링은 로봇의 미끄럼 관절 값을 옮긴다(`shift`·`kick`·`teleport`).
       - 몰 수 있는 로봇(flee, follow)의 `leash` 안(없으면 어디든)에 있고 npc를 세는 입력(`objects` 목록 없음)은 계획에서 `switch`로 본다. 그래야 풀이 순서가 그 입력을 `herd`로 누를 수 있다.
       - `seek_button` 로봇의 `button`은 계획에서 로봇의 `in`이 켜지면 켜진 것으로 보고(`plan_driver`), 몸을 그 위로 보내지 않는다(`sensor`).
       - 정적 점검 `parts` 단계에 경고(실패 아님)를 더했다: `speed` 0.16 m/s 이상(몸 걷기 속도).
   - 리뷰 반영 (2026-11-12, 사용자 승인 6건, 빠른 675 passed·1 skipped, slow `herd`·`warp_hop` 3 passed):
     - `seek_button`의 협동 문: 로봇의 `button`이 어떤 경우에도 `switch`가 되지 않아 "스위치 A → 로봇(`in: [A]`) → 판 → 문" 방이 `order` 단계에서 실패했다. 입력 부품에 `plan_driver`를 두어, 계획에서 로봇의 `in`이 켜지면(`in` 없으면 늘) 그 입력도 켜진 것으로 본다. 로봇의 `in`이 순서상 뒤에 계산되는 채널일 수 있어, 그런 방에서만 `plan_channels`가 값이 멈출 때까지 다시 돈다. 몸이 누를 수 있는 버튼(`trigger: any`)이 목표여도 `sensor`로 내려 몸을 그 위로 보내지 않는다(몸과 로봇이 버튼 가운데서 부딪히지 않게).
     - 로봇이 켤 수 없는 입력(`objects` 목록이 있는 구역)은 몰기용 `switch`로 올리지 않는다.
     - `lure`를 몰 수 있는 행동(`HERDABLE`)에서 뺐다. `World.held`가 E3까지 비어 있어 `herd <lure 로봇>`이 끝나지 않는다. spec R29의 "`follow`/`lure`는 끌고 간다"는 E3에서 다시 넣는다(R 줄은 그대로).
     - 시험 추가: 컨베이어(`push` move → `Npc.shift`), 워프 판(`trigger: npc` → `Npc.teleport`).
     - 높은 바닥: `floor_tile`의 `height` 위에 둔 로봇이 바닥 0에 지어져 타일에 34 mm 묻히고 순찰을 거의 못 했다(점검 단계는 통과시켰다). 글자 지도 칸은 방을 지을 때 이미 올려 두어 문제가 없었다. 공처럼 env가 넣는 `floor_z`를 받아 `reset`에서 몸체 높이(`body_pos` z)로 쓰고, 벽 피하기 광선 높이도 그만큼 올린다. 위아래 관절이 없어서 계단은 로봇에게 벽이다.
13. 예제 방(플랫폼·워프, 함정, 몰기), 가이드 갱신, `mpg bench`를 다시 잰다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-npc`):
     - 예제: 플랫폼은 `lift_spring.yaml`, 함정은 `trap_run.yaml`(E2a·E2b), 몰기 `herd.yaml`, 워프 `warp_hop.yaml`(막힌 벽 너머로 워프, solver 20/20, 평균 9.53 s)을 새로 더했다. 워프 예제에서 되돌림 판(`reset_pad`)은 뺐다. 계획이 되돌림 판을 피하지 않아 그 위를 걸어 8/20이었다.
     - 가이드: `guide*.md` 예제 목록, `parts-guide*.md` 값 표(카탈로그에서 다시 만듦), NPC 로봇 절, `herd` 하위 목표, `parts` 단계 경고. README 마일스톤 2 줄을 완료로 표시했다.
     - `mpg bench` 전후(`switch_door`, 환경 스텝/초, 두 번씩): 전 `35466cd` 단일 572·562, 병렬 12 4,546·4,499 → 후 단일 569·557, 병렬 12 4,515·4,523. 잡음 안이라 새 부품이 없는 방은 느려지지 않았다. 표는 `docs/bench.md` 4차 측정.

PR E3 `feat/m2-grab` (큐브, 잡기, 부리 누르기)
14. Open question 2를 확인한다: 상위 학습 설정의 머리 명령 범위, 머리를 숙이고 걷는 시험.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): 범위는 넓고(neck·head pitch ±1.10 rad) 머리를 숙여 걷기·돌기·벽에 붙기 모두 넘어짐 0/10. 측정값과 결정(`head` 행동 → 숙임 정도 t, 명령 t × (−1.1, +1.1) rad)은 spec Open question 2에 적었다. Risks의 가슴 누름 벽 버튼 대안은 필요 없다.
   - 잡기 위치 측정(15·16단계 설계용): 바닥 물체는 닿지 않는다. 높이 0.17–0.19 m 받침대 앞 모서리에 둔 4 cm 큐브(옆면 손잡이 1.5 cm, 오리 쪽)는 머리를 든 채 걸어 들어가면 부리 끝이 손잡이 4 cm 안에 8/8 들어오고 큐브는 거의 밀리지 않는다(0–1.6 cm). 받침대 0.15 m는 몸이 큐브를 밀어 4 cm 안에 1/8. 그래서 `grab_radius` 기본값을 0.04 m로 두고, 예제 방은 글자 지도의 높은 바닥 칸을 받침대로 쓴다.
15. 큐브, 압력판·소켓 시험, 부리 끝 사이트, weld 잡기. 잡은 채 저장한 체크포인트를 복원하면 계속 잡고 있는지 시험한다(`eq_active`).
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): `parts/object/cube.py`, `grab.py`(숨은 입력 부품 `_beak`), `BodyAdapter.grab_site()`(마이크로덕 `mouth_tip`). 시험은 `tests/test_grab.py` 11개(질량·손잡이 면, 압력판 0.02/0.04 kg 대 `min_mass` 0.03, 놓은 뒤에만 세는 소켓, 0.3 m 넘게 옮기는 동안 2 cm 안·놓으면 떨어짐, 반경 밖, 너무 무거움, 힘 한도로 놓침과 다시 잡기, 잡은 채 저장본 복원, 관측 `beak`).
   - 계획과 다른 점:
     - spec R17 Check는 `tests/test_cube.py`인데, 큐브 시험을 잡기 시험과 한 파일(`tests/test_grab.py`)에 두었다. 큐브만 따로 볼 시험이 적고 도우미(`to_beak`)를 같이 쓰기 때문이다.
     - 머리와 큐브의 접촉을 뺐다(`spec.add_exclude`). 손잡이가 `jaw_soft`에 파고든 채 weld를 켜면 구속 힘이 약 26 N으로 튀어 바로 놓쳤다. 뺀 뒤 들고 걷는 동안 부리와의 거리 변화 0.5 mm 아래, 구속 힘 약 0.35 N.
     - 큐브 손잡이 기본은 옆면 넷(`+x`, `-x`, `+y`, `-y`)이다. 윗면 손잡이는 부리가 닿지 않는다(14단계 측정).
     - `_beak`은 방 순서 맨 앞의 입력 부품이다. 같은 몸 단계에서 센서가 이번 `World.held`를 보게 하려는 것이다.
     - 방에 큐브가 있으면, 큐브가 켤 수 있는 입력 센서(압력판, 소켓, 물체 버튼·구역)를 풀이 순서에서 발판으로 친다(R20 순서 계산이 큐브 방을 풀 수 있게).
16. 행동 확장(`head`, `grab`), 벽 버튼, `demo:`의 grab과 place.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): task `actions: [move]` + `head`·`grab`(이 순서, 각 1칸), 벽 버튼(`mount: wall`, `height`, `facing`), `demo:` `grab <큐브>`·`place <큐브> <부품>`(`carry.py`), 관측 `beak`. 시험 `tests/test_actions.py`(행동 크기 4가지, `check_env`, 값 전달, 잘못된 `actions` 거절), `tests/test_buttons.py` 벽형 종류 3 × enable 2와 높이·옆 어긋남, R19 slow 시험(판 중심 0.15 m, 해답이 머리를 숙여 누름, 시드 0–5 6/6). 예제 `examples/rooms/cube_plate.yaml`(받침대 큐브 → 압력판 → 문): `mpg check` 해답 20/20, 평균 26.4 s.
   - 계획과 다른 점:
     - 벽 버튼 누름 거리 `PRESS_DEPTH`를 6 cm로 두었다. 벽에 붙어 머리를 숙이면 가슴이 먼저 닿아 부리 끝이 판에서 4–5 cm 떨어진다(14단계 측정 3.7–5.4 cm). spec R4의 "닿으면 눌림"을 이 거리 안으로 읽었다.
     - task에 `head`·`grab` 행동이 없을 때 스크립트 해답은 머리 숙임과 잡기 명령을 env에 직접 넣는다(`env.scripted_action`). 그래서 기본 `actions: [move]` 방도 `mpg check` 해답 단계가 큐브와 벽 버튼을 다룬다.
     - `place`는 큐브가 목표 가운데 3 cm 안에 오면 서서 흔들림이 멎기를 기다린 뒤 놓고(걷는 중에 놓으면 머리 흔들림으로 큐브가 십여 cm 튐), 놓은 뒤 몸을 돌려 큐브에서 0.35 m 물러난다(다음 걷기가 큐브를 차서 판에서 밀어냄). 벽 버튼도 누른 뒤 0.3 m 물러난다(벽에 붙은 채로는 길찾기가 시작점을 못 찾음).
     - 예제 방은 글자 지도의 높은 바닥 칸(0.18 m)을 받침대로 쓴다(14단계 측정).
   - `lure` 되살림(12b단계에서 넘어온 일): 잡기가 `World.held`를 채우므로 `lure`를 `HERDABLE`에 다시 넣었다(spec R29 "`follow`/`lure`는 끌고 간다"). 시험: 미끼를 물고 있는 동안만 다가와 미끼에서 `keep`만큼 떨어져 서고, 놓으면 멈춤(fast). `grab bait` → `herd bot pad` 방이 시드 3개 모두 성공(slow, 같은 방 `mpg check` 20/20). 가이드 3개 언어의 lure 줄과 몰기 설명, 값 표, README 진행 줄(시뮬레이션 단순화 문구)을 고쳤다.
   - `lure` 계획과 다른 점:
     - 미끼가 잡을 수 있는 물체(손잡이가 있는 부품)인 `lure`만 몰 수 있다(`behaviors.can_herd`). 아니면 `herd`를 거절하고 로봇용 입력도 발판으로 올리지 않는다.
     - 몰기에서 `lure`는 몸이 아니라 문 미끼가 이끄는 점으로 가고, 기다릴 때 로봇 쪽으로 돌지 않고 선다(돌면 미끼가 휘둘림). `follow`·`lure` 모두 로봇이 `sense` 밖으로 처지면 그쪽으로 걸어 돌아간다(전에는 서서 기다려 끝나지 않았다).
16b. 부리 기믹 프리셋(R28): 집을 물체 6종, 부리 장치 8종, `demo:`에 turn·pull 하위 목표 추가. 뒤로 걷지 않는 제약에 맞춰 당기기 방향을 정한다. `lever`와 `receptacle`의 걸이·바구니 프리셋도 여기서 만든다. 예제 `turret.yaml`. (7b단계에서 넘어온 일: `demo:`에 `look` 명령을 더해 `.seen` 문이 있는 방도 풀이가 통과하게 한다. 지금은 그런 방이 `order` 단계에서 실패한다.)
16c. 애드온 장착대(R30): 먼저 공식 바퀴 모델과 합친 모델에서 `roller.onnx`를 나란히 돌려 OQ7을 정한다(자산 추가 받기는 사용자 승인 후). 그다음 장착대, 모드 전환, `crouch` 행동, 예제 `roller_rink.yaml`. `set_addon` 순서와 리셋·복원 경로, `crouch` 최소 속도, 바퀴 퍼즐 조립 프리셋 9종, 예제 `roller_route.yaml`.
16d. 견인 끈(R31): 먼저 OQ9를 잰다(빈 방에서 썰매 질량·마찰을 올려 가며 velstand 넘어짐과 속도, 장력 기록). 그다음 `rope.py`(매듭, tendon, `connect` 구속, 채널), 부리 묶기·풀기, 끊어짐, `hitch_station`, 저장본·리셋 경로, 정적 점검 경고(문 폭, 꺾임, 질량), `tow`/`untow` 하위 목표, 프리셋 8종, 예제 `tow_yard.yaml`. 구속은 사이트 두 개 `connect`, 붙이기는 매듭 이동·속도 0 → `eq_active` 순서. 워프는 끈 묶음 전체, 고정 끝 끈은 먼저 풂. 매듭 고리 손잡이. NPC 목줄은 구동 이득 0. 매듭 20 g 이상·바닥만 충돌·트리거 제외, `reel_speed`, 순간이동·`set_addon` 전 끈 처리, `rope_door`.  NPC 끈 워프는 평면 관절 값으로, 이득·활동 영역은 끈 상태에서 다시 계산. `attach_failed`.
16e. 주인과 동행(R32): `zone.attach_to` → `owner.py`(경로, 기다리기, 명령, `lost`) → 저장본 → `follow` 하위 목표 → 프리셋 8종 → 예제 `walkies.yaml`. `leash_walk`·`roller_walk`는 R31·R30 뒤. `stay` 동안 `near_ratio`·`lost`·`waits` 멈춤. `receptacle.attach_to`(NPC 자식 몸체). `attach_to` 부품은 상대 `pos`, `place()` 건너뜀, 위치는 `xpos`.
16f. 신호기와 파괴(R33, R34): 논리 부품 몸체(`body: true`) → `source` → `wire`(연결 끊김) → `breakable`(원인 4가지, 고치기) → `relay` → `destroyer` → 회로 그래프 정적 점검 → `break`/`repair` 하위 목표 → 프리셋 8종 → 예제 `power_cut.yaml`. 전선 `carries`/`to` 간선과 맞는 부피, 꼬리표 마커. `substep` 훅으로 탄·눌림 판정(한 접촉 한 번). 부서짐 모양은 복원 뒤 상태에서 다시 입힘. R7 순서 잠금 `.wrong`. `break ... via` 하위 목표. 움직이는 중계기 `hitch`. `crush`는 물체만. 움직이는 부품에 전선 연결 금지(정적 점검).
16g. 에피소드 무작위화(R35): (5b에서 넘어온 일: 코인 위치 샘플러.) 샘플러 5종(`range`, `region`, `candidates`, `shuffle`, `rooms`) → 층별 세기(task.yaml) → 다시 뽑기와 `mpg check` 비율 점검 → 연결 단서 자동 표시 → 결정성·기록·저장본 → `rooms`(stages 구역 재사용, 관측 마스크) → `mpg eval --randomize` 3종 → 예제 `shuffle_switches.yaml`과 비교 학습 기록. M1 `random:`은 `range`의 줄임으로 그대로 읽는다. 다시 뽑기 점검은 그래프 목표 순서 + 뽑힌 값이 입력인 정적 점검(탄도, 끈 길이, 질량). `holdout` 후보는 학습 모델에 미리 구역/선반으로 만들고 학습 뽑기에서 뺌. 순서 추론 시험: `rewards.py`에 `progress_free` → Open question 12로 기준 확정 → `shuffle_switches` 단서 켬·끔 두 정책 학습 → `wide` 비교를 학습 기록에. (7단계에서 넘어온 일: 저장본이 없을 때 쓰는 YAML 대체 `checkpoint_states:`.)
17. 예제 방(큐브·소켓, 벽 버튼, 빨래 `laundry.yaml`). 가이드 완성, AGENTS.md와 SKILL.md 연결.
18. 처음 보는 작업자 시험(R25).

## Risks
- 플랫폼·스프링·워프 위의 몸은 걷기 정책의 학습 분포 밖이라 넘어질 수 있다. 측정한 뒤 프리셋 기본값을 낮추고, 가이드에 안전 범위를 적는다.
- 머리 명령 범위가 좁거나, 머리를 숙인 채 걸으면 넘어진다면 벽 버튼과 잡기가 막힌다. 대안은 벽 버튼의 높이를 낮춰 가슴으로 누르는 프리셋이다. 이 경우 spec R4·R18·R19의 변경을 사용자에게 다시 묻는다.
- 바닥 타일이 걷기를 망칠 수 있다(타일 사이 틈과 접촉). 타일 사이에 틈이 없도록 붙이고, R10 시험으로 확인한다.
- 움직이는 타일(기울기, 높이)은 경첩·슬라이드 관절이 생겨 스텝이 느려진다. 움직이지 않는 타일은 관절 없이 고정 몸체로 만든다.
- 신호 평가와 부품 수 증가로 속도가 떨어질 수 있다. 평가는 매 스텝 파이썬 루프라 부품 50개 이하를 가정한다. bench로 확인한다.
- 체크포인트 저장본 복원 후 걷기 정책의 이력이 어긋나면 첫 몇 스텝이 흔들린다. R6 테스트에 복원 직후 3초 동안 넘어지지 않는지를 넣는다.
- PR E3는 부리 기믹이 늘어 E1만큼 커졌다. 16b를 별도 PR(E4 `feat/m2-beak`)로 나눌 수 있다.
- `max_lift_mass` 기본값 0.05 kg은 실측 전 값이다. 실물 시연 영상의 물체 무게를 모르므로, 사용자가 실물 값을 알게 되면 바꾼다.
- PR E1만으로도 크다. 1–8단계와 9단계(학습)를 나눠 9단계를 별도 PR로 낼 수 있다.

## Proof
| R | 증거 |
|---|---|
| R1 | `tests/test_signals.py` |
| R2 | M1 테스트 전체 통과, `mpg check examples/switch_door.yaml --json` |
| R2b | `tests/test_body_caps.py` |
| R3 | `tests/test_presets.py`, `mpg parts --json` |
| R4 | `tests/test_buttons.py` (벽형은 E3) |
| R5 | `tests/test_sensors.py` |
| R6 | `tests/test_checkpoint.py`; `mjSTATE_INTEGRATION`으로 잡기 용접·끈 연결이 복원 뒤 유지됨 |
| R7 | `tests/test_logic.py` |
| R8 | `tests/test_logic.py::test_transceiver_delay` |
| R8b | `tests/test_counter.py` |
| R9 | `tests/test_doors.py` |
| R10 | 타일 바닥 방의 걷기 시험(slow), `tests/test_floor.py` |
| R11 | `tests/test_platform.py` |
| R12 | `tests/test_devices.py` |
| R13 | `tests/test_setter.py` |
| R14 | `tests/test_warp.py` |
| R15 | `tests/test_spring.py` |
| R16 | `tests/test_traps.py` |
| R17 | `tests/test_cube.py` |
| R18 | `tests/test_grab.py` |
| R29 | `tests/test_npc.py`, `herd.yaml` check |
| R30 | `tests/test_addon.py`, `roller_rink.yaml` check, OQ7 비교 기록 |
| R31 | `tests/test_rope.py`(재부착 위치, `reel_speed`, 워프·회수·`set_addon`, 매듭 트리거 제외 포함), `puzzles/tow/` 정적 check, `tow_yard.yaml` check 18/20(slow), OQ9 측정 기록 |
| R32 | `tests/test_owner.py`, `puzzles/pet/` 정적 check, `walkies.yaml` check 18/20(slow) |
| R33 | `tests/test_source.py` |
| R34 | `tests/test_breakable.py`, `puzzles/circuit/` 정적 check, `power_cut.yaml` check 18/20(slow) |
| R35 | `tests/test_randomize.py`, `mpg check` 다시 뽑기 비율, `mpg eval --randomize nominal/train/wide`, `shuffle_switches.yaml` 비교 기록(slow) |
| R36 | `tests/test_coin.py`, `mpg check examples/rooms/coin_room.yaml --json` |
| R28 | `tests/test_beak_presets.py`, `laundry.yaml` 데모 영상 |
| R19 | `tests/test_actions.py`, 벽 버튼 데모 영상 |
| R20 | `tests/test_check2.py` |
| R21 | `tests/test_curriculum.py`, `runs/<id>/curriculum.json` |
| R22 | `tests/test_eval_full.py`, four_flags의 `mpg eval --full --segments` JSON |
| R23 | `tests/test_guide.py` |
| R24 | `tests/test_examples.py`(slow), MP4 8개(`turret`, `roller_route` 포함) |
| R26 | `tests/test_markers.py` |
| R27 | `tests/test_gaze.py` |
| R25 | 처음 보는 작업자 시험의 작업 기록 |
