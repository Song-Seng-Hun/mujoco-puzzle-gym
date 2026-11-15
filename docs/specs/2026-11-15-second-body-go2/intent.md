# Intent: M3 두 번째 몸 (Unitree Go2)
Status: approved 2026-11-15 (user: "응" — Go2 전용 프리셋으로 가고 기존 방은 건드리지 않기로 함)
View (read-only copy): (아직 없음)

## Problem
- M2까지 이 환경의 몸은 마이크로덕 하나다. `bodies/base.py`의 `BodyAdapter` 경계(R2)는 처음부터 "두 번째 몸이 올 수 있다"를 전제로 설계됐고 `tests/test_boundary.py`가 퍼즐·환경·학습 코드가 몸 이름을 부르지 못하게 막고 있다. 그런데 두 번째 몸이 한 번도 꽂힌 적이 없다. **어댑터가 하나뿐인 추상화는 추상화가 아니라 추측이다.** 경계가 실제로 맞는지, 무엇이 빠졌는지 아직 모른다.
- 능력 시스템(R2b: `Capabilities.names`/`values`, 부품의 `requires_body`, `mpg check`의 비교)도 같은 문제를 안고 있다. 몸이 하나면 `missing()`이 항상 빈 리스트라서, 부품이 선언한 `requires_body`가 정직한지 아무도 검증하지 않았다. 카탈로그는 "이 부품은 `beak_grab`이 필요하다"고 적지만 그게 사실인지, 빠진 선언은 없는지 시험된 적이 없다. 즉 **카탈로그의 능력 표기는 현재 미검증 주장이다.**
- 사용자가 원하는 그림: 다른 로봇을 가진 사람이 이 퍼즐을 자기 로봇으로 돌려 보는 것. 지금은 마이크로덕이 없으면 아무것도 돌려 볼 수 없다.

## Desired outcome
- Unitree Go2가 기존 퍼즐(발판 → 문 → 골)을 걸어서 푸는 영상과 성공률 숫자가 나온다.
- 몸을 바꾸는 일이 **플래그 하나**다(`--body go2`). 퍼즐 YAML은 한 글자도 고치지 않는다.
- `mpg check`가 "이 방은 Go2로 풀 수 없다, 부품 X가 능력 `beak_grab`을 요구하는데 이 몸에 없다"를 정확히 말한다. 지금처럼 항상 통과하지 않는다.
- 카탈로그의 `requires_body` 선언이 **실측으로 교정된다.** 두 몸으로 기존 R-item을 돌려서, 선언과 실제가 어긋나는 부품을 찾아 고친다.
- 두 번째 몸을 꽂아 보며 드러난 `BodyAdapter` 경계의 구멍(마이크로덕 전제가 새어 나온 곳)이 문서로 남는다.

## Scope
범위 안
- `bodies/go2.py`: `UnitreeGo2Adapter`. `make_body("go2")` 등록. 기존 어댑터와 같은 생명주기(`spawn` → `reset` → `step` → `state`)와 같은 명령 규약(`command_from_action`의 전진/횡/회전 부호).
- 자산 핀(R1 패턴 그대로): Menagerie `unitree_go2`의 MJCF와 메시, Hugging Face 정책 `policy.onnx` + `policy.onnx.data`, `params/deploy.yaml`, `params/env.yaml`. 모두 sha256으로 고정하고 라이선스·출처를 안내한다.
- `assets.py`: 외부 가중치가 분리된 ONNX(본체 + `.onnx.data` 짝 파일)를 함께 받고 함께 검증하는 핀 지원.
- 정책 계약 테스트: `deploy.yaml`이 말하는 관측 순서·차원·action scale·관절 매핑이 우리가 넣는 값과 맞는지 검사한다. 어긋나면 침묵하지 말고 터진다.
- 능력 정직화: 두 몸으로 기존 R-item과 퍼즐을 돌려 능력표를 실측하고, 부품의 `requires_body`를 교정하고, `mpg check`의 거부 메시지를 손본다.
- CLI에 몸 선택, 그리고 Go2 기준 퍼즐 프리셋(방 크기 문제, 아래 Open questions 참조).
- 문서: 두 자산의 라이선스와 출처, 그리고 **이 정책은 Unitree 공식 배포가 아니라 커뮤니티 체크포인트**라는 사실.

범위 밖 (나중 후보)
- Unitree G1 휴머노이드. 균형 난이도와 공개 정책 사정이 Go2보다 나쁘다. (단서: lerobot이 Hugging Face에 `lerobot/unitree-g1-mujoco`를 올려 뒀다.)
- lerobot의 공개 로봇들(SO-100/SO-101, Koch, ALOHA, Moss). 전부 고정 베이스 팔이라 평면 보행 명령 `(vx, vy, wz)` 하나로 추상화된 현재 경계와 맞지 않는다. 하려면 `BodyAdapter`가 아닌 별도 베이스 클래스와 다른 퍼즐 전제가 필요하다 — M3가 아니라 별개 마일스톤이다.
- `alpha_ground_pick.onnx`(바닥 집기) 채택 → M4. 몸 추상화를 건드리는 변경과 마이크로덕 전용 새 능력을 같이 넣으면 원인 분리가 어려워진다.
- Go2 전용 신규 부품(높은 턱 넘기, 두 발로 누르기 등), 보행 정책 재학습, 거친 지형·계단, sim2real.

## Affected users and systems
- 코드: `bodies/`(신규 어댑터), `assets.py`와 `upstream.lock.json`, `catalog.py`·`check.py`(능력 비교와 메시지), `cli.py`(몸 선택), 퍼즐 프리셋, `tests/`, README.
- 디스크: 캐시(`~/.cache/mujoco-puzzle-gym/`)가 커진다. `policy.onnx.data`는 가중치를 따로 담는 파일이라 크기를 재야 한다.
- **CI 메모리**: #45에서 fast suite를 2샤드로 나눈 상태다. 두 번째 몸의 테스트가 들어오면 다시 올라간다. M3의 가장 현실적인 위험은 알고리즘이 아니라 이쪽이다.
- 사용자(포트폴리오): "두 로봇이 같은 퍼즐을 푼다"는 그림은 "오리 하나가 푼다"보다 설명력이 크다.

## Constraints
- Menagerie 모델과 Hugging Face 정책 둘 다 BSD-3-Clause다. 출처와 라이선스를 밝히고, 파일은 저장소에 넣지 않고 기존 R1 방식대로 받아서 해시로 검증한다.
- 이 정책은 **공식 동결 정책이 아니다.** 문서에 "Unitree 공식"이라고 쓰지 않는다. 모델 카드가 "`unitree_rl_mjlab`의 기본 `deploy.yaml`과 다를 수 있으니 이 저장소의 `params/deploy.yaml`을 쓰라"고 명시하므로, 그 파일을 같이 핀한다.
- Go2는 마이크로덕보다 몸집이 크다. 기존 방 크기·문 폭·발판 크기가 그대로는 안 맞을 수 있다(`footprint_radius`, `fall_height`가 전부 여기에 매달려 있다).
- CI 메모리는 현재 2샤드 예산 안에서 끝나야 한다. 샤드를 더 쪼개는 것은 마지막 수단이다.
- `rm` 쓰지 않는다. 병합과 브랜치 삭제는 사용자가 한다.
- 범위를 작게 유지한다. M3는 "몸 하나 + 능력 정직화"까지다.

## Known facts (2026-11-15 확인, 출처 명시)
- Menagerie `unitree_go2`: BSD-3-Clause. 디렉터리에 `go2.xml`, `scene.xml`, `go2_mjx.xml`, `scene_mjx.xml`, `assets/`, `LICENSE`가 있다.
- Hugging Face `diasAiMaster/unitree-go2-velocity-flat`: 모델 카드 메타데이터 라이선스 `bsd-3-clause`. 파일은 `policy.onnx` + `policy.onnx.data`(배포용 ONNX), `model_500.pt`(최종 체크포인트), `model_0.pt`~`model_400.pt`, `params/deploy.yaml`(관측 순서, action scale, 관절 매핑), `params/env.yaml`, `params/agent.yaml`, TensorBoard 로그.
- 그 정책의 학습 조건: `gait_phase` **없이** 학습, **action scale 0.5**, 과제 이름 `Mjlab-Velocity-Flat-Unitree-Go2`(평지).
- 관측 차원·순서와 속도 명령 범위는 **모델 카드에 없다.** `params/deploy.yaml`과 `params/env.yaml`을 받아서 읽어야 한다.
- Unitree의 공식 동결 보행 ONNX는 공개 조사에서 찾지 못했다. Unitree가 공개한 것은 학습 코드(`unitree_rl_gym`, `unitree_rl_mjlab`)다. 그래서 커뮤니티 체크포인트를 쓴다(사용자 결정, 2026-11-15).
- 현재 몸 경계(`bodies/base.py`): 필수 구현은 `spawn`, `reset`, `step`, `state`, `teleport`, `root_body_id`, `history_state`, `load_history`. 선택 구현은 `kick`(스프링 R15), `shift`(컨베이어 R12), 애드온(R30), `tow_site`(R31), `grab_site`·`set_head`(부리 R18/R19), `view_camera`·`add_rangefinders`(센서 R39/R27). Go2는 부리와 애드온이 없으므로 선택 구현 상당수를 기본값(없음)으로 둔다 — 능력 게이트가 실제로 작동하는지 보는 첫 시험대가 바로 이 지점이다.
- lerobot의 공개 로봇은 전부 고정 베이스 팔이라 현재 경계와 맞지 않는다(사용자 확인 후 범위 밖, 2026-11-15).

## Open questions
- [x] 정책의 관측 차원·순서, action 차원, 속도 명령 범위 → **해결(2026-11-15)**: 관측 45차원, action 12차원, 50 Hz, 명령 `vx [-1.0, 2.0]`·`vy [-1.0, 1.0]`·`wz [-1.0, 1.0]`. 자세한 내용은 spec.md의 사실 출처와 R41·R42·R44.
- [x] `policy.onnx.data` 크기와 캐시 디스크 예산 → **해결**: `policy.onnx` 14,196 B + `policy.onnx.data` 757,248 B = 771,444 B. 예산 문제 아님.
- [x] 방 스케일 → **결정(2026-11-15, 사용자 "응")**: Go2 전용 프리셋을 새로 두고 기존 마이크로덕 방은 건드리지 않는다. M2 숫자의 재현성을 보존한다. spec.md R47.
- [ ] Go2의 실측 `cmd_limits`, `fall_height`, `footprint_radius`. (Claude, 구현 중)
- [ ] 두 번째 몸의 테스트를 fast suite에 넣을지, 별도 잡으로 뺄지(CI 메모리). (spec에서 제안, 사용자 확정)
- [ ] Go2의 성공률 목표치와 평가 조건. (spec에서 제안, 사용자 확정)
