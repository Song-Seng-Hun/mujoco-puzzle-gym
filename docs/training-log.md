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
