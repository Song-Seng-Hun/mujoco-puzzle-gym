# 학습 기록 (R10, plan 17단계)

기계: Mac Studio M4 Max (성능 12 + 효율 4 코어), 2026-11-11. 평가는 `mpg eval runs/<id> --episodes 100 --json`(학습에 쓰지 않은 시드 1,000,000–1,000,099, 결정적 정책).

## 시도

| # | run id | 설정 | 학습 시간 | 환경 스텝 | eval 성공률 | 평균 소요 | 비고 |
|---|---|---|---:|---:|---:|---:|---|
| 1 | `a1-base15` | `examples/task_ppo.yaml` 그대로: PPO, lr 3e-4, n_steps 256 × 12 env, batch 1024, gamma 0.99, net [128, 128], 보상 progress 10 / stage 2 / success 10 / fall −5 | 15분 | 2,956,692 (3,285/s) | **0.99** (99/100) | 8.3초 | 목표(0.8) 첫 시도에서 넘음. 조정 없음 |

- 같은 시드의 무작위 정책: 성공률 0.00, 평균 보상 −0.83 (학습 정책 30.0).
- 학습 곡선(`runs/a1-base15/curves.png`): 처음 500 에피소드 성공 52%, 마지막 500 에피소드 성공 99.2%, 넘어짐 0%, 평균 길이 8.6초.
- `mpg diagnose runs/a1-base15`: verdict `ok`(롤아웃 20개 모두 성공, 실패 영상 없음).
- 성공 영상: `mpg render runs/a1-base15` → `runs/a1-base15/success.mp4` (시드 1,000,000, 7.9초).
- 실패 1개(시드 1,000,018)는 따로 보지 않았다.

## 결론

조정할 필요가 없어 `examples/task_ppo.yaml`은 바꾸지 않았다. 2시간 한도의 1/8(15분)로 R10을 넘는다.

## 진단 dogfooding (망가진 설정, `examples/broken/task_*.yaml`)

| 설정 | 학습 | verdict | 근거 |
|---|---:|---|---|
| `task_progress_hack.yaml` | 1.5분 | reward_hacking | 보상 36.5 → 156.8, 성공 0 → 0 |
| `task_far_switch.yaml` | 0.5분 | exploration | 발판 밟은 비율 학습 0.0, 롤아웃 0.0 |
| `task_wild_commands.yaml` | 0.3분 | body_failure | 넘어짐 마지막 구간 1.0, 롤아웃 0.35 |

## four_flags 단계별 학습 (R21, R22, plan 9단계)

목표(Open question 3): 2시간 안에 `mpg eval --full` 성공률 0.7 이상, 같은 시간의 `--curriculum none`과 비교. 2026-11-11, 같은 Mac Studio.

- 준비: `mpg check --seeds 20 examples/rooms/four_flags.yaml` → solver 20/20(평균 21.8초), 깃발 저장본 4개.
- 설정: `examples/task_four_flags.yaml`. PPO, lr 3e-4, n_steps 256 × 7 env, batch 896, gamma 0.99, net [128, 128], 에피소드 60초, 관측 `gravity, ang_vel, lin_vel, part_pos, part_state, target_pos, checkpoints`, 보상 progress 10 / stage 2 / success 10 / fall −5, 시드 0. forward는 `advance_at` 0.8, `replay` 0.2, `window` 50.
- 두 학습을 동시에 돌렸다(각 7 env, 16코어 나눔). 평가는 `mpg eval runs/<id> --full --segments --episodes 100`(시드 1,000,000–1,000,099, 결정적 정책).

| run id | curriculum | 학습 시간 | 환경 스텝 | full 성공률 | 구간 성공률 (5구간) | chain_gap_rate | 평균 소요 |
|---|---|---:|---:|---:|---|---:|---:|
| `ff-fwd-120` | forward | 120분 | 10,988,362 | **1.00** | 모두 1.00 | 0.00 | 19.7초 |
| `ff-none-120` | none | 120분 | 13,656,328 | **1.00** | 모두 1.00 | 0.00 | 15.3초 |
| `smoke-fwd` | forward | 1분 | 116,543 | 0.02 | 0.02 / 1.0 / 1.0 / 1.0 / 1.0 | 0.00 | — |
| `smoke-none` | none | 1분 | 121,891 | 1.00 | 모두 1.00 | 0.00 | 18.2초 |

(1분 행은 오류 확인용 짧은 실행이고 평가 50회다.)

- forward 구간 열림(`curriculum.json`): 13.8초, 20.6초, 31.3초, 41.0초에 다음 구간이 열려 41초 만에 5구간이 모두 열렸다. 끝에는 다섯 구간 모두 최근 50회 성공률 1.0.
- none은 학습 61.7초(287 에피소드)에 최근 200회 성공률 0.8을 넘었다. 20분째 두 학습 모두 학습 중 성공률 1.0.
- 목표 0.7은 두 쪽 다 넘었다. 다시 학습하지 않았다.

해석:

- 이 방은 단계별 학습 없이도 1분 안에 풀린다. 깃발이 풀이 순서에 들어가 `progress` 보상과 `target_pos`가 깃발을 차례로 이끌기 때문이다. 그래서 이 비교로는 forward의 이점이 보이지 않는다.
- forward는 오히려 느렸다. 1분째 full 0.02: 첫 구간(시작→깃발 1)이 학습 중 확률 정책으로는 0.66–0.8이지만 결정적 정책으로는 0.02였다. 마지막 구간이 열린 뒤에는 에피소드의 80%가 마지막 구간에서 시작해(2시간 동안 마지막 구간 244,413회, 첫 구간 15,569회) 앞 구간 연습이 적다. 2시간 뒤에는 차이가 없다.
- forward 정책이 4.4초 느리다(19.7초 대 15.3초). 구간 끝 깃발에 닿으면 끝나는 짧은 에피소드로 배워서일 수 있다. 확인하지 않았다.
- 구간 1–4의 시작 상태는 깃발마다 저장본 하나뿐인데도 처음 보는 배치의 full에서 이어 붙이기 실패(chain_gap)가 없었다.
- forward의 이점을 보이려면 progress 보상이 길을 잘 알려 주지 않는 방(먼 스위치, 함정, 난이도 단계)이 필요하다. 12단계 `difficulty` 뒤의 일로 남긴다.
