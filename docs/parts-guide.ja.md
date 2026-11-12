# 部品とプリセットの案内

[使い方ガイドに戻る](guide.ja.md)

パズル部屋は部品を組み立てて作ります。この文書では、部品が信号でつながるしくみ、プリセットで部屋を組む手順、点検と段階的な学習、そしてすべての部品の値の表を扱います。値の表は`mpg parts --markdown`が出力したもので、それ以外は人が書いた説明です。

## 部品、プリセット、組み立てプリセット

- **部品**は`room.yaml`の`parts:`の一行です。種類(`type`)と値でできています。種類ごとに受け付ける値と単位、既定値、許される範囲が決まっていて(値のスキーマ)、範囲外の値は`mpg build`と`mpg check`が理由と範囲を示して拒否します。
- **部品プリセット**は、部品の種類一つと既定値のセットです。コードはありません。`src/mujoco_puzzle_gym/presets/parts/<分類>/<名前>.yaml`にあり、分類は`input`(ボタン・センサー)、`logic`(ゲート・カウンター・トランシーバー)、`device`(ドア・ワープ・ゴールネット・プラットフォーム・柱・回転台・コンベヤー・スプリング・設定変更器)、`floor`、`trap`、`object`、`beak`、`npc`です。`beak`と`npc`はまだ空です。
- **組み立てプリセット**は、複数の部品をまとめたもの、または部屋一つ全体です。`presets/puzzles/<テーマ>/<名前>.yaml`にあります。

`mpg parts --json`は、登録された部品と値のスキーマ、プリセットの一覧を出します。自作の部品まで見るには`--plugin my_parts`を付けます。

## プリセットで部屋を組む

部品の行に`preset:`を書くと、プリセットの種類と値を取り込み、その行に書いた値がプリセットの値を上書きします。

```yaml
parts:
  - {name: button_a, preset: input/floor_oneshot, pos: [-0.35, 0.55]}
  - {name: door, preset: device/sliding_door, pos: [0.4, 0.0], width: 0.4, in: [button_a]}
```

部屋のいちばん上に`preset:`(名前一つ、またはリスト)を書くと、組み立てプリセットを敷いて、その上に部屋の値を重ねます。

```yaml
preset: puzzles/coins/coin_lock     # コイン3枚、コインロックlock、ドアlock_door
size: [2.4, 1.8]
spawn: {pos: [-0.9, -0.7]}
parts:
  - {name: lock, need: 2}                         # 同じ名前: 値だけ変える
  - {name: goal, type: goal, pos: [0.85, 0.0]}    # 新しい名前: 足す
```

合わせ方の規則は次のとおりです。リストの順に合わせ、部屋のファイルが最後です。

- いちばん上の値(`size`、`spawn`、`demo`など)は、後のものが置き換えます。プリセットの`name`は引き継ぎません。
- `parts`は`name`で突き合わせます。同じ名前の行に`type`か`preset`があれば部品をまるごと置き換え、なければ書いた値だけを変えます。新しい名前は後ろに足します。
- `links`はつなげます。

上書きした値を含む最終的な値は、学習のときに`runs/<id>/task.resolved.yaml`の`room_parts`に残ります。

部屋を組む手順:

1. `mpg parts --markdown`か下の表から、使うプリセットを選びます。
2. 壁と出発点を決め、部品の行ごとに`preset`と`pos`、変える値を書きます。
3. 信号をつなぎます。受け取る部品の`in:`に、送る部品のチャンネル名を書きます。
4. `mpg build`で画像を見て、`mpg check`で点検します。

## 信号バス

部品は名前の付いたチャンネルで信号をやりとりします。

- チャンネルの値は実数です。オン・オフは1と0で、0.5より大きければオンと読みます。カウンターは数えた数をそのまま載せます。
- 部品は`in`のチャンネルを読み、`out`のチャンネルに書きます。`out`を書かなければ、ボタンやゲートのように信号を出す部品は自分の名前のチャンネルに書きます。カウンターは`<名前>.count`、`<名前>.reached`のような、点の付いたチャンネルも出します。
- `enable: <チャンネル>`がオフのとき、その部品の出力は止まります(電源)。
- 体のステップごとに、入力部品 → 論理部品 → 装置部品の順に一度ずつ更新するので、ボタンからドアまで同じステップで伝わります。
- 状態を持たない部品だけで回るループ(NOT → NOT)は拒否します。遅延やラッチのように状態を持つ部品をはさんだループは使えます。
- M1の`links: [{from, to}]`は、受け取る部品の`in`に送る部品のチャンネルを足すのと同じです。

## 式ゲートの文法

`type: gate`は、次のどれか一つで出力を計算します。

- `expr`: チャンネルを読む式。例: `a and not b`、`delay(btn, 1.5)`、`latch(pulse(a), reset_btn)`、`count(lap) >= 3 or timer(plate) > 2`。
- `op`と`in`: チャンネル名を式に書かない形なので、プリセットで使います(`logic/and`など)。`and`、`or`、`xor`はすべての入力、`not`、`delay`、`hold`、`pulse`、`timer`、`count`は入力一つ、`latch`は`[set, reset]`です。
- `sequence: [a, b, c]`: 順序ロック。順序を間違えると`<名前>.wrong`が1ステップだけオンになります。
- `logic: <名前>`: `register_logic`で登録したPython関数。

式で使えるもの: チャンネル名(点の入った`laps.count`もチャンネル一つ)、数値、`and or not xor`、比較(`< <= > >= == !=`)、かっこ、関数`delay(x, s)`、`hold(x, s)`、`pulse(x)`、`latch(set, reset)`、`count(x)`、`timer(x)`。算術、属性アクセス、ほかの関数の呼び出し、文字列は、部屋を読み込むときに拒否します。式は`ast`で読んで許可リストと照らし合わせ、このパッケージが自分で計算するので、他人が作った部屋のファイルを開いてもコードは実行されません。

## 動く装置と設定変更器

- プラットフォーム(`platform`、プリセット`device/moving_platform`、`device/elevator`): `pos`から測った`path: [[dx, dy, z], …]`(zは上面の高さ)を`speed`でたどります。`mode`は`loop`、`pingpong`、`once`、`run`は`always`、`while_signal`(信号がオンの間)、`on_signal`(オンになるたびに一回運行)です。スライド関節と位置アクチュエーターで動くので、上に置いた物体も一緒に運びます。
- 昇降柱(`pillar`)、回転台(`turntable`)、コンベヤー(`conveyor`): 信号がオンの間、せり上がり、回り、ベルトの上の体と物体をベルトの速さで運びます。コンベヤーのベルトは床に描いたものなので、摩擦は床と同じです。
- スプリング(`spring`、プリセット`device/spring_pad`): パッドの上の対象に速度`velocity` [vx, vy, vz]を与えます。プリセットは物体だけを飛ばします(`trigger: object`)。
- 設定変更器(`setter`): 信号がオンになった瞬間に`target: <部品>.<値>`を`value`に変え、`revert: true`ならオフになったときに元に戻します。下の値の表で「実行中の変更」が「可」の値だけが対象になります。ドアの幅のような大きさの値は、部屋を読み込むときに拒否します。
- 値の連結`bind`: すべての部品が受け取る値です。`bind: {speed: {from: lever.angle, map: [-1, 1, 0, 0.3]}}`のように、実行中に変えられる数値をチャンネルに直線でつなぎ、体のステップごとに値の範囲で切って適用します。同じ値を`bind`と設定変更器の両方が変える場合は拒否します。
- 解の順序と解答は、プラットフォームと回転台のマス、せり上がる柱のマスを避けて進みます。体が乗ったまま耐えられるかは歩行ポリシーが学んだ範囲の外なので、記録だけを取ります。2026-11-12の記録: プラットフォーム0.1・0.3 m/s、ベルト0.1・0.2 m/sで3秒間転びませんでした。

## 組み立ての例

どれも`mpg check`の解答段階を20回中18回以上通ります。

| 部屋 | すること | 使うプリセット |
|---|---|---|
| [`examples/rooms/logic_lock.yaml`](../examples/rooms/logic_lock.yaml) | ボタン二つを両方踏むと、ANDゲートがドアを開けます | `input/floor_oneshot`, `logic/and`, `device/sliding_door` |
| [`examples/rooms/doors.yaml`](../examples/rooms/doors.yaml) | 引き戸、開き戸、同じ信号で開く落とし戸(解答は穴を回り込む) | `device/sliding_door`, `device/swing_door`, `device/trapdoor` |
| [`examples/rooms/four_flags.yaml`](../examples/rooms/four_flags.yaml) | 旗1→2→3→4を順に通ってゴールへ。段階的な学習のデモ部屋 | `input/checkpoint_flag` |
| [`examples/rooms/coin_room.yaml`](../examples/rooms/coin_room.yaml) | コイン3枚をすべて拾うとドアが開きます。同じ組み立ては`puzzles/coins/coin_lock` | `coin`, `coin_lock` |
| [`examples/rooms/mini_soccer.yaml`](../examples/rooms/mini_soccer.yaml) | ボールを体で押してゴールネットに入れるとドアが開きます | `ball`, `goal_net` |
| `puzzles/ball/ball_ramp` | 高い台の上のボールを坂(3.4°)で押し下ろし、ゴールネットに入れます。文字マップの坂なので、歩く体の坂の上限5°に収まります | 文字マップ |
| `puzzles/ball/bowling` | ボールでピン三本のどれか一本を倒すと(`<ピン>.down`)ドアが開きます | `ball`, `pin` |
| `puzzles/ball/color_sort` | 色別のゴールネット二つにボールを分けて入れ、カウンターでドアを開けます | `ball`, `goal_net`, `counter` |
| [`examples/rooms/lift_spring.yaml`](../examples/rooms/lift_spring.yaml) | ボタン一つでドアが開き、エレベーターが上がり、設定変更器でスプリングがオンになってボールが跳ね始めます | `device/elevator`, `device/spring_pad`, `device/setter`, `device/sliding_door` |
| [`examples/rooms/two_rooms_map.yaml`](../examples/rooms/two_rooms_map.yaml), [`stairs_map.yaml`](../examples/rooms/stairs_map.yaml) | 文字マップで描いた二つの部屋、階段と坂 | `button`, `door` |

ボールの組み立てプリセットは部屋一つ全体なので、そのまま`mpg check`に渡してもよく、`preset: puzzles/ball/bowling`で敷いて値を変えて使ってもかまいません。

## 点検の二段階

`mpg check room.yaml`は、静的段階(シミュレーションなし)と動的段階に分かれます。結果の`static.failed`、`dynamic.failed`が、失敗した段階を順に出します。

静的段階:

| 段階 | 見るもの |
|---|---|
| `values` | 値がスキーマの範囲内か、プリセット名が存在するか、マーカー番号 |
| `channels` | 誰も書かないチャンネルを読んでいないか、一つのチャンネルに書く部品が二つないか、状態を持たないループ、存在しない部品を指すリンク |
| `demo` | `demo:`のサブゴールの命令と対象 |
| `body` | 部品の`requires_body`の能力が体にあるか |
| `parts` | 部品ごとの点検(ゴールネットの入口より大きいボール、届かないボール・コイン) |
| `overlap` | 既定の配置で、部品の硬い本体が壁やほかの部品に5 mmを超えてめり込んでいないか。床パッド・旗・ゴール領域は除き、ドアは自分の壁(ドアがふさぐすき間の壁、同じ線上の壁)とだけは重なってもかまいません |
| `order` | 信号と壁から見て、ゴールまでの解く順序があるか。文字マップの部屋では、部品が床のマスの上にあるかと、行く手をふさぐ段差・坂も |

最初の三段階(`values`、`channels`、`demo`)は部屋を読み込む途中で失敗するので、そのときの結果には失敗した段階だけがあり、解答段階は飛ばします。静的段階だけを見るには`--static`を付けます。

動的段階:

| 段階 | 見るもの |
|---|---|
| `stability` | 指令0で10秒間、転んだり計算が発散したりしないか |
| `solver` | 答えを知っているスクリプトが、ランダム配置20個のうち18個以上を解けるか。部屋に`demo:`(`goto`、`press`、`wait`、`push <ボール> to <ゴールネット>`)があればその順序に先に従い、残りは格子の解答が引き継ぎます |

解答段階は、通った旗ごとにスナップショットを`runs/check/<部屋>/checkpoints/<旗>.pkl`に残します。

## チェックポイントと段階的な学習

旗(`input/checkpoint_flag`、`order: 1, 2, …`)がある部屋は区間に分かれます。まず`mpg check`で旗のスナップショットを作り、`mpg train --curriculum forward`で区間を一つずつ開きます。詳しい使い方は[使い方ガイドの段階的な学習](guide.ja.md#段階的な学習と区間の評価)にあります。

```bash
mpg check examples/rooms/four_flags.yaml
mpg train task.yaml --minutes 60 --curriculum forward
mpg eval runs/<id> --full --segments
```

## つかむ動作はまだありません

キューブ、くちばしでつかむ動作、壁のボタン、くちばしの装置(`beak`分類)は次のまとまりで入ります。そのときのつかむ動作は単純化したものです。くちばしの先が物に触れた状態でつかむ命令を出すと、物をくちばしの先にくっつけ(溶接拘束)、指の接触や滑りは再現しません。

## Pythonプラグイン

新しい部品は`Part`を継承し、`register_part`で登録します。値を`fields`に`Field`として書けば、範囲の検査と`mpg parts`の表が自動でついてきます。ゲートの関数は`register_logic`で登録し、部屋では`logic: <名前>`で呼びます。

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

`task.yaml`に`plugins: [my_parts]`と書くと、学習のワーカープロセスまで含めて、このモジュールを最初に読み込みます。プリセットのフォルダーはこのパッケージの中にしかありません。自作の部品の既定値のセットは、部屋のファイルに値として書きます。

## 値の表

以下は`mpg parts --markdown`の出力です。部品を直したら、この部分をその出力に置き換えます(`tests/test_guide.py`が確認します)。

<!-- mpg parts --markdown: begin -->
### すべての部品に共通の値

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | 必須 |  |  | position [x, y] on the floor |
| `random` | vec | m | `[0.0, 0.0]` | [0, 5] |  | uniform +- range added to pos on every reset |
| `require` | enum |  | `all` | `all`, `any` |  | gates: open when all / any of their inputs are on |
| `requires_body` | list |  | `[]` |  |  | body capabilities this part needs (R2b), checked by `mpg check` |
| `in` | list |  | `[]` |  |  | channels this part reads (R1) |
| `out` | list |  | — |  |  | channels this part writes (default: its name if it emits) |
| `enable` | str |  | — |  |  | power channel; off (<= 0.5) freezes the part's output |
| `marker` | any |  | — |  |  | ArUco marker (R26): `none` or {id, size, face}; default: a free id on parts that carry one |
| `bind` | any |  | — |  |  | R13: {<run-time value>: {from: <channel>, map: [in0, in1, out0, out1]}}, applied every body step, clipped to the value's range |

### `ball`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.05` | [0.02, 0.2] |  | OQ10: >= 0.05 m pushes without falling (<= 0.3 kg) |
| `mass` | float | kg | `0.05` | [0.005, 5] |  | OQ10: <= 0.3 kg pushes without falling (radius >= 0.05 m) |
| `elasticity` | float |  | `0.5` | [0, 0.95] |  | restitution (0 = no bounce), through contact solref |
| `friction` | float |  | `0.8` | [0, 2] |  | sliding friction |
| `roll_friction` | float | m | `0.005` | [0, 0.05] |  | rolling friction (0.005: a 0.5 m/s ball stops in about 0.2 m) |
| `roll_damping` | float | N m s/rad | `0.0` | [0, 1] |  | damping on the rotational dofs |
| `color` | enum |  | `white` | `white`, `red`, `blue`, `green`, `yellow`, `orange`, `purple`, `black` |  |  |

### `button`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `kind` | enum |  | `momentary` | `momentary`, `oneshot`, `toggle` |  |  |
| `mount` | enum |  | `floor` | `floor` |  | floor (wall buttons: E3) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  | plate size [x, y] |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |

### `checkpoint`

役割`checkpoint`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `order` | int |  | `1` | [1, 100] |  | flags turn on in this order |
| `radius` | float | m | `0.15` | [0.03, 1] |  | reach distance |
| `skip` | enum |  | `ignore` | `ignore`, `fail` |  | reaching this flag before its turn: ignore it, or fail the episode |

### `coin`

役割`prop`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.08` | [0.02, 0.5] |  | pick-up distance from the body centre |
| `group` | str |  | `coins` |  |  | coin group (channels <group>.count, <group>.total) |

### `coin_group`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the group (it has no body) |

### `coin_lock`

役割`prop`、更新段階`logic`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.8, 0.3]` | [0.1, 5] |  | belt size [x, y] |
| `axis` | enum |  | `x` | `x`, `y` |  | direction the belt runs along |
| `speed` | float | m/s | `0.1` | [-0.5, 0.5] | 可 | belt speed (+: toward +axis) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it carries |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `counter`

役割`prop`、更新段階`logic`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | where guides draw the counter (it has no body) |
| `up` | str |  | 必須 |  |  | channel whose turn-ons add 1 |
| `down` | str |  | — |  |  | channel whose turn-ons subtract 1 |
| `reset` | str |  | — |  |  | channel whose turn-on sets the count to start |
| `start` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `min` | int |  | `0` | [-1e+06, 1e+06] |  |  |
| `max` | int |  | `1000` | [-1e+06, 1e+06] |  |  |
| `wrap` | bool |  | `False` |  |  | past max -> min, below min -> max |
| `target` | int |  | `1` | [-1e+06, 1e+06] |  | `<name>.reached` is 1 at count >= target |
| `persist` | bool |  | `False` |  |  | keep the count when a snapshot is restored |

### `door`

役割`gate`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `width` | float | m | `0.5` | [0.1, 2] |  | opening the panel spans |
| `axis` | enum |  | `y` | `x`, `y` |  | direction the panel lies along |
| `thickness` | float | m | `0.04` | [0.01, 0.2] |  |  |
| `height` | float | m | `0.25` | [0.05, 1] |  |  |
| `open_dir` | enum |  | `1` | `1`, `-1` |  | slide toward +axis (1) or -axis (-1) |
| `speed` | float | m/s | — | [0.02, 5] |  | opening speed (default: as fast as the motor moves it) |

### `floor_tile`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `friction` | vec | - | `[1.0, 0.005, 0.0001]` | [0, 5] | 可 | slide, torsion, roll friction |
| `tilt` | float | deg | `0.0` | [-13, 13] | 可 | tilt about tilt_axis (+: raises the +y side for axis x, +x side for axis y) |
| `tilt_axis` | enum |  | `x` | `x`, `y` |  | hinge axis of the tilt |
| `height` | float | m | `0.0` | [-0.1, 0.1] | 可 | top face height above the room floor |
| `on_friction` | vec | - | — | [0, 5] | 可 | friction while the inputs are on |
| `on_tilt` | float | deg | — | [-13, 13] | 可 | tilt while the inputs are on |
| `on_height` | float | m | — | [-0.1, 0.1] | 可 | height while the inputs are on |
| `friction_rate` | float | 1/s | `5.0` | [0.01, 100] | 可 | friction change per second |
| `tilt_speed` | float | deg/s | `10.0` | [0.5, 90] | 可 |  |
| `height_speed` | float | m/s | `0.05` | [0.005, 0.5] | 可 |  |

### `gate`

役割`prop`、更新段階`logic`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

役割`goal`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.15` | [0.03, 1] |  | reach distance |

### `goal_net`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `width` | float | m | `0.36` | [0.08, 2] |  | mouth width (inside) |
| `depth` | float | m | `0.2` | [0.05, 1] |  | inside depth |
| `height` | float | m | `0.12` | [0.03, 0.5] |  | wall height |
| `facing` | float | deg | `180.0` | [-180, 180] |  | direction the open side faces |
| `thickness` | float | m | `0.02` | [0.005, 0.1] |  | wall thickness |
| `accepts` | list |  | `[]` |  |  | ball colours that count (empty: any) |
| `need` | int |  | `1` | [1, 100] |  | balls in for the main channel |

### `laser`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `to` | vec | m | 必須 |  |  | receiver position [x, y] |
| `height` | float | m | `0.05` | [0.01, 0.5] |  | beam height |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  |  |

### `pillar`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `size` | vec | m | `[0.1, 0.4]` | [0.03, 2] |  | footprint [x, y] |
| `height` | float | m | `0.25` | [0.05, 1] |  | top height when up |
| `speed` | float | m/s | `0.2` | [0.01, 2] | 可 | rising speed |

### `pin`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.02` | [0.01, 0.1] |  |  |
| `height` | float | m | `0.12` | [0.03, 0.5] |  |  |
| `mass` | float | kg | `0.03` | [0.005, 2] |  |  |
| `down_angle` | float | deg | `45.0` | [5, 90] |  | tilt from upright that counts as down |
| `color` | enum |  | `white` | `white`, `red`, `blue`, `green`, `yellow`, `orange`, `purple`, `black` |  |  |

### `platform`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `path` | any |  | 必須 |  |  | [[dx, dy, z], ...] from `pos`: points the top centre follows (2 or more) |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 2] |  | slab size [x, y] |
| `thickness` | float | m | `0.03` | [0.01, 0.2] |  |  |
| `speed` | float | m/s | `0.1` | [0.01, 1] | 可 | speed along the path |
| `mode` | enum |  | `pingpong` | `loop`, `pingpong`, `once` |  | loop: closed loop; pingpong: back and forth; once: to the end and stop |
| `run` | enum |  | `always` | `always`, `while_signal`, `on_signal` |  | always; while_signal: while `in` is on; on_signal: one trip per rising edge |

### `pressure`

役割`sensor`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  |  |
| `min_mass` | float | kg | `0.1` | [0.001, 100] | 可 | mass needed to turn on |
| `trigger` | enum |  | `object` | `body`, `object`, `npc`, `any` |  |  |

### `receiver`

役割`prop`、更新段階`logic`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `band` | str |  | 必須 |  |  | band name shared by a transmitter and its receivers |
| `delay` | float | s | `0.0` | [0, 3600] |  | hold the value back |

### `receptacle`

役割`sensor`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

### `setter`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | unused (a setter has no body) |
| `target` | str |  | 必須 |  |  | <part>.<value>: the run-time value to change |
| `value` | any |  | 必須 |  |  | new value (checked against the target value's schema) |
| `revert` | bool |  | `False` |  |  | put the old value back when the signal turns off |

### `spring`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.2, 0.2]` | [0.05, 2] |  | pad size [x, y] |
| `velocity` | vec | m/s | `[0.0, 0.0, 1.5]` | [-5, 5] | 可 | launch velocity [vx, vy, vz] (world frame) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it launches |
| `cooldown` | float | s | `1.0` | [0.1, 60] |  | time a launched body or object ignores this spring |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `swing_door`

役割`gate`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `kind` | enum |  | `momentary` | `momentary`, `oneshot`, `toggle` |  |  |
| `mount` | enum |  | `floor` | `floor` |  | floor (wall buttons: E3) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.05, 2] |  | plate size [x, y] |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |
| `latch` | bool |  | `True` |  |  | M1: stay on after the body leaves (oneshot); false: momentary |

### `transmitter`

役割`prop`、更新段階`logic`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `band` | str |  | 必須 |  |  | band name shared by a transmitter and its receivers |
| `delay` | float | s | `0.0` | [0, 3600] |  | hold the value back |

### `trapdoor`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mode` | enum |  | `follow` | `follow`, `oneshot` |  | follow: open while the signal is on; oneshot: stay open once opened |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `hinge_side` | enum |  | `-x` | `-x`, `+x`, `-y`, `+y` |  | edge the tile hangs from |
| `speed` | float | deg/s | `180.0` | [10, 2000] |  | opening speed |

### `turntable`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `radius` | float | m | `0.25` | [0.05, 1.5] |  |  |
| `height` | float | m | `0.008` | [0.004, 0.1] |  | top face above the floor |
| `speed` | float | deg/s | `45.0` | [-360, 360] | 可 | spin rate while on (+: counter-clockwise) |

### `warp`

役割`warp`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.25, 0.25]` | [0.1, 2] |  | pad size [x, y] |
| `trigger` | enum |  | `body` | `body`, `object`, `npc`, `any` |  | what it moves: body, object, npc or any |
| `to` | any |  | — |  |  | destination: {pos: [x, y], yaw}, reset or next_stage |
| `routes` | any |  | — |  |  | conditional destinations: [{when: expr, to: dest}, ..., {else: dest}] |
| `cooldown` | float | s | `1.0` | [0, 60] |  | time a warped body or object ignores warp pads |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `zone`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.4, 0.4]` | [0.05, 10] |  | box size [x, y] |
| `height` | float | m | `0.5` | [0.02, 3] |  | box height above the floor |
| `trigger` | enum |  | `body` | `body`, `object`, `npc`, `any` |  |  |
| `objects` | list |  | — |  |  | only these object parts count (overrides trigger) |
| `direction` | vec |  | — |  |  | count only entries moving this way |
| `rearm` | enum |  | `always` | `always`, `leave` |  |  |
| `leave_margin` | float | m | `0.05` | [0, 1] |  | rearm leave: how far outside the box a target must go |

### 部品プリセット

| プリセット | 部品の種類 | 値 |
|---|---|---|
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

### 組み立てプリセット

- `puzzles/ball/ball_ramp`
- `puzzles/ball/bowling`
- `puzzles/ball/color_sort`
- `puzzles/ball/mini_soccer`
- `puzzles/coins/coin_lock`
<!-- mpg parts --markdown: end -->
