# 쌓인 큐브를 집어 옮기기 — 막힌 것과 그 증거

Status: draft (R134에서 측정, 수정은 별도 작업)

## 왜 이 폴더가 있나

도전 방 하나(「젱가 들기」: 2단 탑에서 **맨 아래** 큐브를 상자에 넣고, 집기 쉬운 미끼 큐브는 상자가
거부)를 만들다가 **공용 운반의 버그 두 개**를 재서 멈췄다. 방 초안과 task 초안은 이 폴더에 있고,
프리셋 폴더에는 두지 않는다 — 그 폴더의 모든 방은 `mpg check`를 통과해야 하고(느린 스위트가
`test_room_presets_pass_check`로 전수 검사한다), 이 방은 통과하지 못한다.

## 측정된 버그 둘

1. **탑에서 집어 올린 큐브를 옮기면 운반이 즉시 손을 놓는다.** `cube.rest_on`으로 다른 큐브 위에
   놓인 큐브는 z 0.06에서 시작한다. `grab` 다음 스텝에서 `place`/`drop`이 `grab = -1`을 보내
   놓아 버리므로 물체가 이동하지 않는다. 바닥에서 집은 큐브는 같은 경로가 정상이다
   (측정: 실제 운반 7.9초 대 깨진 쪽 3.1초, 같은 방·같은 목표). 의심 지점은 `carry.py`의 높이
   논리(`_lower_in_act` / `_carry_z` / `_let_go`와 `_pick_z`)가 "물체가 이미 운반 높이보다 위"를
   "목표 위에 도달했다, 놓아라"로 읽는 것.
2. **그 거짓을 `ScriptedSolver._unfinished`가 구역 목표에서 못 잡는다.** `drop top shelf`에서
   `shelf`가 `input/object_zone`일 때, 큐브가 0.33 m 떨어져 있고 구역 신호가 0인데도 하위 목표가
   "완료"로 보고됐다. R78이 그 후조건을 넣은 이유가 "시간 초과가 진전처럼 보인다"인데, 상자
   (receptacle) 경우는 잡히고 구역 경우가 빠져 있다.

## 재현

```bash
mpg check docs/specs/2026-11-18-stack-pick/room-draft.yaml --task docs/specs/2026-11-18-stack-pick/task-draft.yaml
```

0/20이 나오고, 실패 이유에 `place base box`가 나온다. 한 판을 추적하면 `grab` 직후 스텝에서
`carrier.held()`가 None이 되는 것이 보인다.

## 고칠 때의 기준 (먼저 적어 둔다)

- 팔 방 **열 개**가 각자의 task로 20/20을 유지하고 **대본 시간이 그대로**여야 한다
  (0.6 · 1.3 · 8.2 · 11.5 · 7.6 · 13.1 · 18.8 · 10.6 · 7.8 · 9.6초).
- `tools/grab_grid.py`의 집기 격자가 후퇴하지 않아야 한다.
- 빠른·느린 스위트 통과.
- 공용 경로에 조건을 더하는 수정은 이 저장소에서 **네 번 연속 다른 방을 깼다**
  (`docs/training-log.md` R132–R133). 먼저 재고, 깨지면 되돌린다.

고친 뒤에는 방 초안을 `presets/puzzles/tabletop/jenga_lift.yaml`로 올리고,
`tests/test_examples.py`의 `PRESET_TASKS`에 등록하고, `tools/refresh_parts_tables.py`로 3개 언어
표를 다시 만든다.
