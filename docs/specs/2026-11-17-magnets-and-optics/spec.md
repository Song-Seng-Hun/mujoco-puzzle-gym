# Spec — 자석과 빛 (magnets and optics)

Status: approved (2026-11-17, 사용자 `/goal 예정된 퍼즐부품들 모두 추가`)
Intent: [intent.md](intent.md)
Baseline: `f554736` (`measure/budget-and-balance`, PR #55)

부품 다섯과 방 셋. 번호는 R97 다음부터 이어 붙인다. R98과 R99~R101은 서로 독립이므로 따로 싣는다.

```mermaid
flowchart LR
  magnet[magnet: 이름으로 끌어와 붙잡기] --> arm[닿지 않던 물건이 닿는 곳으로]
  emitter[emitter: 빛 + 경로] --> mirror[mirror: 꺾기]
  mirror --> cell[photocell: 도착과 세기]
  emitter --> splitter[splitter: 나누기]
  splitter --> cell
  filter[filter: 깎기] --> cell
```

## Requirements

### R98 `magnet` — 이름으로 끌어와 붙잡는 비접촉 손

`pos`의 기둥이 반경 `radius` 안의 **받는 물체**(`accept`, 비면 모든 물체)를 중심 쪽으로 `power` [N]로
끈다. `in`/`enable`이 꺼지면 힘만 사라진다 — 움직이던 물체는 관성으로 조금 더 가 마찰에 멈춘다(측정:
0.472 m에서 끊으면 0.370 m, 0.359 m에서 끊으면 0.076 m).

- 값: `pos`, `radius` [m], `power` [N, 실행 중 변경], `accept`(물체 부품 이름 목록), `hold` [m],
  `height` [m, 이보다 높은 중심은 무시].
- 채널: `<이름>` — 받는 물체가 `hold` 안에 붙어 있으면 1.
- `wind`와 다른 것 셋(탐침에서 측정): ① 힘의 방향이 **물체 위치에 따라** 정해진다(바람의 `yaw`는 상수),
  ② `accept`로 **고른다**(바람의 `trigger`는 종류뿐), ③ `hold` 안에서 힘을 끊어 **세운다**(일정한 힘은
  벽까지 민다 — 측정: x 1.1 → −1.07 m).
- `power`는 힘이므로 질량에 대한 비율로 뜻이 달라진다. `wind.check_warnings`와 같은 형식으로 경고한다.
- 검사: `tests/test_magnet.py` — 끌어온다 / 끄면 도착하지 않는다 / `accept` 밖은 안 움직인다(변위 1 mm 미만) /
  `hold` 안에서 채널 1 / 체크포인트.

### R99 빛의 경로 — `emitter`, `photocell`, `mirror`

`laser`(R5, 끊기 센서)는 **건드리지 않는다**. 새 가족은 "빛이 도착했는가"를 묻는다.

- `emitter`(device): `pos`, `yaw` [deg, 실행 중 변경], `power` [-], `bounces`(최대 반사 수),
  `height` [m]. 매 몸 스텝 `mj_ray`로 경로를 따라가며 거울을 만나면 꺾고, 그 외의 것을 만나면 멈춘다.
  채널 `<이름>` — 경로가 하나라도 `photocell`에 닿으면 1.
- `photocell`(input): `pos`, `radius`, `min_power`. 채널 `<이름>`(세기 `min_power` 이상이면 1),
  `<이름>.power`(실수).
- `mirror`(device): `pos`, `yaw` [실행 중 변경], `width`, `loss`. 입사 광선을 거울 법선에 대해 반사하고
  세기에 `(1 − loss)`를 곱한다. 몸과 물체에 **닿는다**(벽처럼).
- 선 수 상한: 경로 전체에서 `MAX_SEGMENTS`를 넘으면 그 가지를 버린다(Codex #43: 깊이만으로는 부족).
- 왜 부품인가(탐침): 벽이 광선을 가리면 `laser`는 0을 내는데, 그 0은 "아무도 끊지 않았다"의 0과
  **같은 값**이다. 도착을 묻는 채널이 레포에 없고, 반사 코드도 없다.
- 검사: `tests/test_optics.py` — 직선으로 닿는다 / 벽이 끊는다 / 거울이 꺾어 닿게 한다 /
  `yaw`를 틀리게 두면 안 닿는다 / `loss`가 세기를 깎는다 / `bounces` 상한에서 멈춘다.

### R100 `splitter` — 하나의 빛을 나눠 쓰기

반투명 판. 입사 세기의 `split`은 그대로 지나가고 `(1 − split)(1 − loss)`는 반사된다.

- 값: `pos`, `yaw` [실행 중 변경], `width`, `split` [0..1, 실행 중 변경], `loss`.
- 왜 부품인가: 게이트 식은 **채널 사이의 산술을 거절한다**("BinOp is not allowed"). 그래서 세기를
  비율로 나누는 것은 식으로 쓸 수 없다. `budget`(R91)이 메운 빈칸과 같은 종류다.
- **측정으로 판정한다**: 방에서 `split`을 바꾸는 것이 실제로 선택(한쪽만 열린다)이 되는지. 되지 않으면
  R89·R92처럼 숫자와 함께 철회한다.
- 검사: 세기 합이 보존된다(손실 제외) / `split` 0과 1의 끝값 / 두 `photocell`이 동시에 켜지지 않는 설정.

### R101 `filter` — 지나가는 빛을 깎는 판

- 값: `pos`, `yaw`, `width`, `transmission` [0..1, 실행 중 변경]. 지나간 세기에 곱한다. 반사는 없다.
- **스칼라 하나로 줄인다**(Codex #42는 3밴드): 밴드별 관측 배관이 없다. 직전 스펙이 계측기 전체를
  같은 이유로 뺐고, 같은 기준을 여기에 적용한다.
- 왜 문이 아닌가(탐침): 벽·문은 빛을 **끊는다**(0/1). 필터는 깎는다 — "짧지만 어두운 길과 돌아가지만
  밝은 길" 중에서 고르게 만드는 것이 이 부품의 유일한 축이다. 그 선택이 방에서 성립하지 않으면 철회한다.
- 검사: 투과율이 곱으로 쌓인다 / `min_power` 아래로 떨어지면 `photocell`이 꺼진다.

### R102 방 셋

- `puzzles/tabletop/fetch_the_steel` (R98): 팔의 도달(0.48 m) 밖에 큐브 둘. 팔이 접시에 추를 올리면
  자석이 켜지고, `accept`에 적힌 쪽만 닿는 곳으로 온다. 미끼는 제자리(변위를 적는다).
  대조: demo를 `goto goal` 하나로 줄이면 0/5, 자석을 끄면 아무것도 오지 않는다.
- `puzzles/light/aim_the_mirror` (R99): 빛이 벽에 막혀 있다. 바닥 판이 거울의 `yaw`를 바꾸고(setter),
  닿으면 문이 열린다. 대조: 판을 밟지 않으면 0/5.
- `puzzles/light/long_way_round` (R100+R101): 짧은 길에는 필터가 있어 세기가 `min_power` 아래다.
  거울로 돌아가는 길이 답. 스플리터가 들어가면 둘 중 하나만 열린다.
- 방마다 머리글에 측정값과 **실패한 시도**를 적는다(R70·R74·R88·R93 형식).

### R103 문서와 표

- `docs/parts-guide{,.en,.ja}.md`: `tools/refresh_parts_tables.py`로 표를 다시 만들고(세 언어 한 번에),
  방 셋을 더한다. `docs/bodies{,.en,.ja}.md`의 `requires_body` 감사표에 새 부품 행을 더한다.
- `examples/task_*.yaml`과 `tests/test_examples.py`의 `PRESET_TASKS`에 방 셋을 등록한다.

### R104 조합 가능성 잠금 (탐침 결과를 테스트로)

- `tests/test_optics.py::test_a_wall_across_the_beam_stops_it` — 문·벽이 이미 셔터다(필터가 0/1용이
  아니라는 근거).
- `tests/test_magnet.py::test_a_wind_cannot_hold_an_object_in_place` — 일정한 힘은 벽까지 민다(자석의
  `hold`가 메우는 자리).
- `tests/test_signals.py::test_a_gate_refuses_arithmetic_between_channels`(이미 있음)가 스플리터의 근거다.

## Out of scope

- 3밴드 스펙트럼(Codex #42), 광학 솔버, 렌더러 반사.
- 레이저 출력을 소비하는 예산 결합(R91 + R100): 먼저 스플리터가 선택이 되는지부터.
- 자기 **반발**(polarity −1): 밀기는 `wind`·`pusher`로 이미 된다. 끌기만 들인다.
- 자석으로 몸을 끄는 것: 몸의 질량 대비 힘이 schema 상한에서도 걷기를 못 흔든다(`wind`의 R46 감사와
  같은 계산). 물체만 받는다.

## Sources

- Codex-mac #30 `MAGNETIC_PUCK`, #41 `MIRROR_ROUTE`, #42 `SPECTRAL_FILTER_TILE`, #43 `BEAM_SPLITTER`
  (`/Users/songseunghun/Documents/Codex/2026-09-27-window-pc-pc/outputs/puzzle-parts-ideas-20261117/IDEAS.md`)
- 직전 스펙의 유보 결정: `docs/specs/2026-11-17-measure-and-budget/spec.md` 「Out of scope」
- 탐침 측정: intent.md 표 (2026-11-17, 이 스펙 작성 전)
