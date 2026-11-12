# 部品とプリセットの案内

[使い方ガイドに戻る](guide.ja.md)

パズル部屋は部品を組み立てて作ります。この文書では、部品が信号でつながるしくみ、プリセットで部屋を組む手順、点検と段階的な学習、そしてすべての部品の値の表を扱います。値の表は`mpg parts --markdown`が出力したもので、それ以外は人が書いた説明です。

## 部品、プリセット、組み立てプリセット

- **部品**は`room.yaml`の`parts:`の一行です。種類(`type`)と値でできています。種類ごとに受け付ける値と単位、既定値、許される範囲が決まっていて(値のスキーマ)、範囲外の値は`mpg build`と`mpg check`が理由と範囲を示して拒否します。
- **部品プリセット**は、部品の種類一つと既定値のセットです。コードはありません。`src/mujoco_puzzle_gym/presets/parts/<分類>/<名前>.yaml`にあり、分類は`input`(ボタン・センサー)、`logic`(ゲート・カウンター・トランシーバー)、`device`(ドア・ワープ・ゴールネット・プラットフォーム・柱・回転台・コンベヤー・スプリング・設定変更器)、`floor`、`trap`、`object`、`beak`、`npc`(自分で動くロボット)です。
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

## 罠

罠(`trap`分類)は体を押したり、倒したり、落としたりします。どれも`in`の信号がオンの間だけ動き(`in`がなければ常に)、強さは`power`一つで決めます。`enabled: 0`で止まります(`in`のない穴は常に開いていて止められず、`enable`や`enabled`を書くと拒否します。切り替えるには`in`を与えます)。`power`と`enabled`は実行中に変えられるので、設定変更器、`bind`、難易度段階(後述)で調整します。

| 罠 | プリセット | `power` |
|---|---|---|
| 風 `wind` | `trap/crosswind` | 区域内の体と物体を押す力 [N]、向きは`yaw` |
| 押し板 `pusher` | `trap/pusher` | 板が箱から出て戻る速さ [m/s] |
| 回転バー `spinner` | `trap/spinning_bar` | 床の高さのバーの回転速度 [deg/s] |
| 振り子 `pendulum` | `trap/pendulum` | 振れ幅 [deg]、周期は`period` |
| 穴 `pit` | `trap/pit` | なし。落ちると`on_fall: fail`(エピソード失敗)か`reset`(出発点かチェックポイントへ)。`in`があるとオフの間はふたが閉じています |
| 大砲 `cannon` | `trap/cannon`, `trap/cannon_turret` | 砲弾を撃ち出す速さ [m/s] |
| 滑る床、粘つく床 | `trap/slippery_floor`, `trap/sticky_floor` | 床タイル(`floor_tile`)の摩擦 |

- 解く順序と解答スクリプトは、穴、回転バーと振り子が掃く範囲、押し板が出る範囲を穴とみなして回り込みます。
- 大砲は`period`秒ごとに`burst`発撃ちます(`fire: edge`なら信号がオンになるたびに)。狙い`aim`は`fixed`(`yaw` + `aim_yaw`、`aim_pitch`)、`at_body`(今の体の位置)、`lead`(体が向かう先)、`signal`(`aim_yaw`と`aim_pitch`を`bind`でチャンネルにつなぐ)です。`spread`は狙いの周りの円錐の半角です。`<名前>.fired`は撃った瞬間に1ステップだけオン、`<名前>.hits`は体に当たった数です。
- 砲弾は前もって作った`pool`個(既定10)を使い回します。`lifetime`秒たつか、全部出ていれば一番古いものを回収して撃ち直します。スナップショットには飛んでいる砲弾も入ります。
- 砲弾はボタン、圧力板、区域を押しません。数えるのは的(`input/target`)だけです。的は`min_speed`以上で当たると`<名前>.hits`を増やしてオンになります(`mode: latch`はオンのまま、`pulse`は1ステップ)。
- 砲台(`trap/cannon_turret`)はレバーのような信号で狙い、信号がオンになった瞬間に一発撃ちます。`mpg check`は`bind`が出せる狙いの範囲全体で放物線を計算し、一方向でも的の板に当たれば通します。`spread`があれば円錐全体が板に入る必要があります。
- 倒れる強さの記録(2026-11-12、立っている体、距離0.4 m、横と前から各3回):砲弾の質量 × 速さが0.5 N·s以下では一度も倒れず、0.6 N·s(0.1 kg、6 m/s、前)から倒れはじめ、0.8 N·s(0.2 kg、4 m/s)では6回中4回、1.0 N·s(0.2 kg、5 m/s)では6回とも倒れました。そのため既定値は0.2 kg、5 m/sです。歩いている体はもっと弱く、0.4 N·sの砲弾を2.5秒ごとに受けた解答スクリプトは20回中6回倒れました。
- 砲弾の質量(`shell_mass`)は部屋を組むときだけ決めます。MuJoCoが物体の質量行列を前もって計算しておくため、実行中に変えると動きが狂います。

## NPCロボット

`npc`分類は、自分で床を滑って動く小さなロボットです(`type: npc`)。平面の関節3つ(x、yのスライド、zの回転)と速度アクチュエーターで動き、質量があるので体・物・壁とぶつかります。床のボタンとゾーンは、ロボットの中心を物と同じように見ます(`trigger: npc`か`any`)。上下の関節がなく床に重さがかからないので、圧力板は中心が板の上にある間、ロボットの`mass`を数えます。上にマーカーが付きます。

| 行動`behavior` | プリセット | すること |
|---|---|---|
| `flee` | `npc/flee` | 体が`sense`の中に来ると反対側へ逃げます。ふさがれると壁に沿って滑ります |
| `follow` | `npc/follow` | `sense`の中の体のほうへ来て、`keep`だけ離れて止まります |
| `lure` | `npc/lure` | 体が`bait`の物(キューブ)をくわえている間、その物の方へ来て`keep`だけ離れて止まります |
| `patrol` | `npc/patrol` | `path`(`pos`から測った点、一周)を回ります。体が前をふさぐと止まって待ちます |
| `guard` | `npc/guard` | 持ち場(`pos`)を守り、体が`sense + reach`の中に来ると`reach`まで出てきて立ちふさがります |
| `shy_gaze` | `npc/shy_gaze` | 体がマーカーを見ている間(視線センサー、部屋の`gaze:`)は止まり、見ていないと体のほうへ近づきます。`invert: true`で逆になります |
| `seek_button` | `npc/seek_button` | `in`がオンになると`button`の部品の上へ行って止まります |
| `sleepy` | `npc/sleepy` | `awake`秒のあいだ`path`を回り、`asleep`秒のあいだ止まるのを繰り返します |

- 共通の値: `speed`(最高速度、既定0.08 m/s)、`accel`、`sense`(感知半径)、`size`、`mass`、`leash: [x0, y0, x1, y1]`(ロボットの中心が出ない箱、部屋の座標)。`enable`がオフになるとその場で止まります。`speed`、`accel`、`sense`、`keep`、`reach`、`awake`、`asleep`は実行中に変えられます。
- 壁の回避: 進みたい方向とその±35°、±70°に光線を5本撃ち、ふさがれていない最初の方向へ進みます。5本ともふさがれると止まります。規則に乱数がないので、同じシードなら同じ軌跡です。
- チャンネル: `<名前>.moving`(動いている間1)、`<名前>.on`(乗っているボタンのマーカー番号、なければ-1)。観測項目`npcs`は、ロボットごとの相対位置、速度、動いているか、反応中か(6個)です。
- `speed`が体の歩く速さ(約0.16 m/s)以上だと、`mpg check`の`parts`段階が警告します(失敗にはなりません)。体が追いつけないためです。
- `seek_button`のロボットの`button`(ロボットが押せるボタン、ゾーン、レーザー、圧力板)は、解く順序ではロボットの`in`がオンになればオンとして数えます(`in`がなければ最初から)。体はその上へは行かず、ロボットが行って押します。そのため「スイッチA → ロボット(`in: [A]`) → 板 → ドア」の部屋はAを押すだけで解けます。
- そのほかロボットだけが押す入力(`trigger: npc`、`objects`の指定なし)は、追い込めるロボット(`flee`、`follow`、えさがキューブの`lure`)の`leash`の中(`leash`がなければどこでも)にあれば、解く順序では踏み板として数えます。解答の`demo:`に`herd <ロボット> <部品>`(または`herd <ロボット> <x> <y>`)を書くと、`flee`のロボットは反対側から追い込み、`follow`のロボットは先に立って連れて行きます。`lure`のロボットは、先に`grab <えさ>`でえさをくわえてから`herd`を書くと、くわえたえさで連れて行きます。

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
| [`examples/rooms/trap_run.yaml`](../examples/rooms/trap_run.yaml) | 穴を回り込み、床で回るバーの横と弱い横風の中を通ってゴールへ | `trap/pit`, `trap/spinning_bar`, `trap/crosswind` |
| [`examples/rooms/herd.yaml`](../examples/rooms/herd.yaml) | 逃げるロボットを隅のボタンの上へ追い込むと、その間ドアが開いています | `npc/flee`, `input/floor_momentary`(`trigger: npc`), `device/sliding_door` |
| [`examples/rooms/warp_hop.yaml`](../examples/rooms/warp_hop.yaml) | ドアのない壁の向こうのゴールへ、ワープ板で越えます | `warp` |
| [`examples/rooms/cube_plate.yaml`](../examples/rooms/cube_plate.yaml) | 台の上のキューブをくちばしでつかんで圧力板に載せると、ドアが開きます | `cube`, `pressure`, `door` |
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
| `parts` | 部品ごとの点検(ゴールネットの入口より大きいボール、届かないボール・コイン)。体と同じくらい速いNPCロボットは警告(`warnings`)だけで、失敗にはなりません |
| `overlap` | 既定の配置で、部品の硬い本体が壁やほかの部品に5 mmを超えてめり込んでいないか。床パッド・旗・ゴール領域は除き、ドアは自分の壁(ドアがふさぐすき間の壁、同じ線上の壁)とだけは重なってもかまいません |
| `order` | 信号と壁から見て、ゴールまでの解く順序があるか。文字マップの部屋では、部品が床のマスの上にあるかと、行く手をふさぐ段差・坂も |
| `actions` | 解くのに要る行動がtaskの`actions`にあるか。物を運ぶ必要があれば(`demo:`の`grab`、または解く順序につかめる物だけが入れられる入力)`grab`、解答が押す壁のボタンの板の上端が頭を上げたくちばしの先(0.21 m)より低ければ`head`が要ります |

最初の三段階(`values`、`channels`、`demo`)は部屋を読み込む途中で失敗するので、そのときの結果には失敗した段階だけがあり、解答段階は飛ばします。静的段階だけを見るには`--static`を付けます。

動的段階:

| 段階 | 見るもの |
|---|---|
| `stability` | 指令0で10秒間、転んだり計算が発散したりしないか |
| `solver` | 答えを知っているスクリプトが、ランダム配置20個のうち18個以上を解けるか。部屋に`demo:`(`goto`、`press`、`wait`、`push <ボール> to <ゴールネット>`、`herd <ロボット> <部品>`)があればその順序に先に従い、残りは格子の解答が引き継ぎます |

解答段階は、通った旗ごとにスナップショットを`runs/check/<部屋>/checkpoints/<旗>.pkl`に残します。

## チェックポイントと段階的な学習

旗(`input/checkpoint_flag`、`order: 1, 2, …`)がある部屋は区間に分かれます。まず`mpg check`で旗のスナップショットを作り、`mpg train --curriculum forward`で区間を一つずつ開きます。詳しい使い方は[使い方ガイドの段階的な学習](guide.ja.md#段階的な学習と区間の評価)にあります。

```bash
mpg check examples/rooms/four_flags.yaml
mpg train task.yaml --minutes 60 --curriculum forward
mpg eval runs/<id> --full --segments
```

### 難易度段階

`task.yaml`の`difficulty:`は、罠の強さのような値を段階的に上げます。実行中に変えられる値(「可」)かエピソード開始時に変えられる値だけが対象です。

```yaml
difficulty:
  - {param: cannon_a.power, from: 2.0, to: 5.0, steps: 4}   # 段階0..3で2, 3, 4, 5
  - {param: wind_a.enabled, from: 0, to: 1, at_step: 2}      # 段階2からオン
curriculum: {mode: forward, order: difficulty_first}
```

- `steps: n`は`from`から`to`までをn段階に分け、`at_step: k`は段階kから`to`になります。行ごとに段階数が違えば一番長いものに合わせ、短い行は最後の値のままです。
- 段階的な学習は(区間、難易度)の組を一つずつ開きます。`order: difficulty_first`(既定)は一つの区間の難易度を上げきってから次の区間へ、`segment_first`は一つの難易度ですべての区間を開いてから次の難易度へ進みます。レベル名は`<区間>@d<段階>`で、`curriculum.json`とエピソード記録に残ります。
- `bind`がかかっている値、範囲外の値、大きさの値は部屋を読むときに拒否します。
- 抽選なしで動かすと(評価、ただの`reset`)最後の段階です。`mpg eval runs/<id> --difficulty 1`のように段階を選べます。

## キューブとくちばしでつかむ動作

- `cube`は取っ手の付いた箱です(`size`、`mass`、`friction`、取っ手の面`handles`は`+x`、`-x`、`+y`、`-y`、`+z`から選択、既定は側面の4つ)。圧力板、ソケット、物で押すボタンと区域が数えます。部屋にキューブがあると、キューブで入る入力は解く順序で踏み板として数えます。
- くちばしでつかむ動作は、実物のアヒルには移せないシミュレーション上の単純化です。シミュレーションのモデルにはくちばしの関節がないので、くちばしの先(`mouth_tip`)が取っ手から`grab_radius`(既定4 cm)以内にあり、つかむ命令が0.5を超えると、頭とキューブの間の溶接拘束を入れます。頭とキューブはぶつかりません。0.5以下にすると放します。質量が体の`max_lift_mass`(既定0.05 kg)より大きいとつかめず、`beak.too_heavy`が1になります。保持に`max_hold_force`より大きな力が要ると落とし、`<キューブ>.dropped`がパルスで出ます。もう一度つかむには命令を一度下げてから上げます。
- くちばしの先は床から0.135–0.21 mの高さなので、床のキューブには届きません。高さ0.17–0.19 mの台の手前の縁に、取っ手を体の方へ向けて置くと、歩いて近づいてつかめます。文字地図では高い床のマス(`legend`の`{type: floor, height: 0.18}`)が台になります。
- チャンネル: `beak.holding`、`beak.held_id`(つかんだキューブのマーカー番号、なければ-1)、`beak.too_heavy`、キューブごとに`<キューブ>.held`、`<キューブ>.dropped`。つかまれたキューブは床のボタン、圧力板、区域、ソケットが数えません。保存データはつかんだ状態まで戻します。ワープは、くわえているキューブも頭に対する姿勢を保ったまま一緒に運びます。
- 観測項目`beak`: つかんでいるか、つかんだ物の種類(キューブ1、なし0、ほかの物は次の節)、くちばしの先から一番近い取っ手までの相対位置(体の向き基準のx、y、z)。5個。
- 壁のボタン: `button`に`mount: wall`を付けると、壁の面(`pos`)の`height`の高さに`facing`の向きを向いた板(`size`: [幅, 高さ])になります。くちばしの先が板の上にあり、板の前6 cm以内なら押されます(壁に寄って頭を下げると胸が先に当たり、くちばしの先は板から4–5 cm離れるためです)。板の後ろは板の厚さ + 1 cmまでしか数えないので、薄い壁の向こう側のアヒルは押せません。`trigger`は使いません。
- 上位の行動: `task.yaml`の`actions: [move]`(既定)に`head`(頭を下げる1個: -1でそのまま、1で一番低く)と`grab`(1個、0.5を超えるとつかむ)をこの順で足せます。行動の大きさは3に足した数を加えたものです。方策にできないことはスクリプト解答もしません。キューブを運ぶ部屋や低い壁のボタンを押す部屋は、taskに`grab`・`head`がないと`mpg check`の`actions`段階で失敗します。例: `mpg check examples/rooms/cube_plate.yaml --task examples/task_cube_plate.yaml`。
- 解答の`demo:`: `grab <キューブ>`(取っ手の前まで歩き、頭の高さを合わせてつかむ)、`place <キューブ> <部品>`(部品の中心の上へ運んで止まってから放し、離れる)。壁のボタンは`press <ボタン>`か解く順序で、頭を下げて押します。
- 例 [`examples/rooms/cube_plate.yaml`](../examples/rooms/cube_plate.yaml): 台の上のキューブをつかんで圧力板に載せ、ドアを開けます。

## くちばしで扱う物と受ける部品

`cube`のほかにも、取っ手のある物はすべて同じ方法でつかみます(つかむ半径、持てる質量、落とす力、`.held`・`.dropped`チャンネルが同じです)。プリセットは`beak`分類にあります。

| プリセット | 種類 | 既定値 | 用途 |
|---|---|---|---|
| `beak/cube_weight` | `cube` | 4 cm、40 g | はかり・圧力板に載せる重り。体の`max_lift_mass`(0.05 kg)より少し軽い |
| `beak/laundry` | `laundry` | 12 × 10 cm、厚さ6 mm、20 g、4辺の上に縁(`hem`) | 軽い洗濯物。板は硬く、しわは見た目だけです。上の縁をつかみます |
| `beak/key` | `key` | 20 g、頭4 cm、軸5 cm、`-x`端に輪 | 頭にマーカーが付きます。`accept: [<マーカー番号>]`を持つ受ける部品だけが受け取ります |
| `beak/handle_ball` | `handle_ball` | 半径3 cm、30 g、`-x`側に輪 | 放すと転がります。ボールの部品`ball`とは別です |
| `beak/egg` | `egg` | 25 g、半径2.2 cm、高さ6 cm、横に取っ手4つ、`break_height` 0.05 m、`on_break: fail` | 割れると`<卵>.broken`が1になります |
| `beak/stick` | `stick` | 長さ20 cm、太さ1.6 cm、20 g、`-x`端に取っ手 | くわえていると、遠い端が壁のボタンを押します |
| `beak/hook` | `receptacle` `shape: hook` | 8 × 8 cmの柱、高さ0.19 mに長さ6 cmのかけ棒、`tolerance` 0.04 m | かけ棒の先から4 cm以内で放した物が掛かります |
| `beak/basket` | `receptacle` `shape: box` | 30 × 30 cm、壁の高さ10 cm、`require: count`、`count: 1` | `<かご>.count`と`<かご>.mass`(kg)で中身を数えます |
| `beak/scale` | `pressure` `trigger: object` | 20 × 20 cm、`min_mass` 5 g | 数値チャンネル`<はかり>.mass`(kg、1 g刻み)。5 gから`<はかり>`が1 |
| `beak/wall_button_beak_low` | `button` `mount: wall` | 板12 × 6 cm、中心の高さ0.15 m、`oneshot` | 頭を下げないと(`head`行動)届きません |
| `beak/wall_button_beak_mid` | `button` `mount: wall` | 板12 × 6 cm、中心の高さ0.2 m、`oneshot` | 立ったままくちばしの先(0.21 m)が届きます |

- 卵: 接触が始まるときの法線方向の相対速度が、`break_height`から落ちたときの速さ(√(2·9.81·`break_height`)、既定0.99 m/s)以上なら割れます。既定の体で測った値(2026-11-12): くちばしから床へ放すと1.42 m/s(頭を一番低く)–1.86 m/s(頭を上げたまま)、台から取って床の板に置くと1.68–1.87 m/s、つかんで運ぶ間の衝突は0.24 m/s以下(台に押し付けると0.49 m/s)、着地後に触れるのは0.35 m/s以下です。そのため5 cmにすると放したときだけ割れます。部品の既定の`on_break`は`none`(チャンネルだけ)、プリセットは`fail`(割れるとエピソード失敗)です。割れた卵は色が変わり、保存データとリセットで戻ります。
- 鍵とソケット: 受ける部品の`accept`には名前と種類のほかにマーカー番号(整数)も書けます。ソケットが同じ番号のマーカーを付けてもよく、このときだけマーカー番号の重複が許されます(ほかの重複は今まで通り拒否)。向きはソケットの`angle_tolerance`で見ます。
- かけ棒: くわえていない物の取っ手がかけ棒の先から`tolerance`以内に来ると、物はかけ棒に固定され、入ったと数えます。もう一度くわえると外れます。かけ棒は見えるだけでぶつからない論理的なかけ棒です(ぶつかるのは柱だけで、柱と掛けられる物の間の接触は切ります)。`facing`がかけ棒の向きです。
- かご: 壁が10 cmより低いと、体が引っかかって転ぶか壁を乗り越えます(5 cmの壁で4回中3回転倒)。10 cmの壁越しには、頭を上げたくちばしの先が壁面から5.8–6.8 cm先まで届きます。
- 棒: くわえている棒の遠い端も、壁のボタンではくちばしの先と同じ窓(板の前6 cm、板の後ろは板の厚さ + 1 cm、板の幅・高さの内側)で数えます。測った値: スクリプトでゆっくり近づくと10回すべて、棒の端が板の面の前2.9–5.9 cm、高さの差2.7 cm以下で押し、押す前の溶接の力は0.89 N以下でした。壁にそのまま歩いて突っ込むと、6回中4回は力が`max_hold_force`(3 N)を超えて棒を落とします。頭を下げると棒の端は大きく下がります(頭1単位で約0.43 rad)。
- 観測`beak`の物の種類: `cube` 1、`laundry` 2、`key` 3、`handle_ball` 4、`egg` 5、`stick` 6、なし0。
- 解答の`demo:`に`drop <物> <部品>`が加わります。`place`と同じく運びますが、頭を上げたまま放します(卵を割る解答)。`place <物> <かけ棒>`は先頭の取っ手をかけ棒の先に合わせて掛けます。棒をくわえて`press <ボタン>`をすると、棒の端を板の高さに合わせて押します。
- 既知の限界: 割れた卵も受ける部品が数えます。かご・はかりに置くスクリプト解答は、部屋とシードによって10回中8–10回成功します。

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
| `mount` | enum |  | `floor` | `floor`, `wall` |  | floor: stepped on; wall: pressed with the beak tip |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.02, 2] |  | plate size [x, y]; wall: [width, height] |
| `height` | float | m | `0.2` | [0.02, 1] |  | wall: plate centre above the floor |
| `facing` | float | deg | `0.0` | [-180, 180] |  | wall: the direction the plate faces (0 = +x) |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |

### `cannon`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 turns the trap off (setters, difficulty steps) |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | heading the cannon faces |
| `height` | float | m | `0.15` | [0.05, 1.5] |  | muzzle height above the floor |
| `power` | float | m/s | `5.0` | [0.5, 15] | 可 | launch speed |
| `shell_mass` | float | kg | `0.2` | [0.005, 1] |  | shell mass |
| `shell_radius` | float | m | `0.025` | [0.01, 0.1] |  |  |
| `period` | float | s | `2.0` | [0.2, 60] | 可 | fire: auto: time between bursts |
| `burst` | int |  | `1` | [1, 10] | 可 | shells per burst |
| `spread` | float | deg | `0.0` | [0, 30] | 可 | half angle of the random cone around the aim |
| `aim` | enum |  | `fixed` | `fixed`, `at_body`, `lead`, `signal` |  | fixed: yaw + aim_yaw, aim_pitch; at_body / lead: at the body (now / ahead); signal: aim_yaw, aim_pitch from channels (bind) |
| `aim_yaw` | float | deg | `0.0` | [-180, 180] | 可 | aim left / right of `yaw` |
| `aim_pitch` | float | deg | `10.0` | [-30, 80] | 可 | aim elevation |
| `lifetime` | float | s | `3.0` | [0.2, 30] |  | a shell is parked after this |
| `pool` | int |  | `10` | [1, 50] |  | shells built in advance (reused) |
| `fire` | enum |  | `auto` | `auto`, `edge` |  | auto: every `period` while active; edge: one burst per rising edge of `in` |

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

### `cube`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.03` | [0.001, 10] |  | above the body's max_lift_mass the beak cannot lift it |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `size` | float | m | `0.04` | [0.02, 0.3] |  | edge length |
| `handles` | list |  | `['+x', '-x', '+y', '-y']` |  |  | faces with a beak handle: +x, -x, +y, -y, +z |
| `handle_size` | float | m | `0.015` | [0.005, 0.1] |  | how far a handle sticks out of its face |

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

### `egg`

役割`prop`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.025` | [0.005, 0.2] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `radius` | float | m | `0.022` | [0.01, 0.06] |  | egg radius (x, y) |
| `height` | float | m | `0.06` | [0.02, 0.15] |  | egg height (z) |
| `handles` | list |  | `['+x', '-x', '+y', '-y']` |  |  | faces with a knob: +x, -x, +y, -y |
| `handle_size` | float | m | `0.012` | [0.005, 0.05] |  |  |
| `break_height` | float | m | `0.05` | [0.005, 2] | 可 | a contact starting as fast as a drop from this height breaks it |
| `on_break` | enum |  | `none` | `none`, `fail` |  | fail: a broken egg fails the episode |

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

### `handle_ball`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.02` | [0.001, 1] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `head` | float | m | `0.04` | [0.02, 0.1] |  | side of the square head (the marker sits on it) |
| `shaft` | float | m | `0.05` | [0.01, 0.2] |  | shaft length along +x |
| `handle_size` | float | m | `0.015` | [0.005, 0.05] |  | how far the ring sticks out of the head's -x end |

### `laser`

役割`switch`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `to` | vec | m | 必須 |  |  | receiver position [x, y] |
| `height` | float | m | `0.05` | [0.01, 0.5] |  | beam height |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  |  |

### `laundry`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
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

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `behavior` | enum |  | `flee` | `flee`, `follow`, `lure`, `patrol`, `guard`, `shy_gaze`, `seek_button`, `sleepy` |  | flee, follow, lure, patrol, guard, shy_gaze, seek_button or sleepy |
| `size` | vec | m | `[0.12, 0.12, 0.07]` | [0.04, 0.5] |  | chassis [x, y, z] |
| `mass` | float | kg | `0.3` | [0.02, 5] |  |  |
| `speed` | float | m/s | `0.08` | [0, 1] | 可 | top speed; at or above the body's 0.16 m/s `mpg check` warns |
| `accel` | float | m/s^2 | `0.5` | [0.05, 10] | 可 |  |
| `sense` | float | m | `0.4` | [0.05, 5] | 可 | reacts to the body this close (flee, follow, lure, patrol, guard) |
| `leash` | vec | m | — | [-50, 50] |  | [x0, y0, x1, y1] box (world) its centre never leaves; default: none |
| `keep` | float | m | `0.3` | [0, 3] | 可 | follow, lure, shy_gaze: stops this far from the body / bait |
| `reach` | float | m | `0.25` | [0, 3] | 可 | guard: how far it steps out from its post |
| `path` | any |  | — |  |  | patrol, sleepy: [[dx, dy], ...] from `pos`, 2 or more points, a loop |
| `button` | str |  | — |  |  | seek_button: the part it goes onto |
| `bait` | str |  | — |  |  | lure: the object the body must hold |
| `awake` | float | s | `2.0` | [0.1, 60] | 可 | sleepy: seconds moving |
| `asleep` | float | s | `2.0` | [0.1, 60] | 可 | sleepy: seconds still |
| `invert` | bool |  | `False` |  |  | shy_gaze: move while looked at, stop otherwise |

### `pendulum`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 turns the trap off (setters, difficulty steps) |
| `height` | float | m | `0.6` | [0.1, 2] |  | pivot above the floor |
| `length` | float | m | `0.45` | [0.05, 1.9] |  | pivot to ball centre |
| `radius` | float | m | `0.06` | [0.01, 0.3] |  | ball radius |
| `mass` | float | kg | `0.5` | [0.01, 5] |  | ball mass |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | swing direction |
| `period` | float | s | `1.6` | [0.3, 10] | 可 | one full swing |
| `power` | float | deg | `40.0` | [0, 80] | 可 | swing amplitude while active |

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

### `pit`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 closes the lid; only a pit with `in` can be switched |
| `size` | vec | m | `[0.3, 0.3]` | [0.05, 5] |  | tile size [x, y] |
| `on_fall` | enum |  | `fail` | `fail`, `reset` |  | fail: the episode ends; reset: back to the last checkpoint or the start |

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

### `pusher`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 turns the trap off (setters, difficulty steps) |
| `size` | vec | m | `[0.3, 0.1]` | [0.03, 1.5] |  | paddle [width, height] |
| `stroke` | float | m | `0.3` | [0.05, 1.5] |  | how far the paddle runs out |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | direction it pushes (0: +x) |
| `power` | float | m/s | `0.3` | [0.02, 2] | 可 | paddle speed |

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

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `pos` | vec | m | `[0.0, 0.0]` |  |  | unused (a setter has no body) |
| `target` | str |  | 必須 |  |  | <part>.<value>: the run-time value to change |
| `value` | any |  | 必須 |  |  | new value (checked against the target value's schema) |
| `revert` | bool |  | `False` |  |  | put the old value back when the signal turns off |

### `spinner`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 turns the trap off (setters, difficulty steps) |
| `length` | float | m | `0.4` | [0.05, 1.5] |  | hub to each tip |
| `height` | float | m | `0.03` | [0.01, 0.3] |  | bar centre above the floor |
| `thickness` | float | m | `0.03` | [0.01, 0.1] |  |  |
| `power` | float | deg/s | `90.0` | [-720, 720] | 可 | turn rate while active (+: counter-clockwise) |

### `spring`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.2, 0.2]` | [0.05, 2] |  | pad size [x, y] |
| `velocity` | vec | m/s | `[0.0, 0.0, 1.5]` | [-5, 5] | 可 | launch velocity [vx, vy, vz] (world frame) |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it launches |
| `cooldown` | float | s | `1.0` | [0.1, 60] |  | time a launched body or object ignores this spring |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects count when their centre is lower than this |

### `stick`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `mass` | float | kg | `0.02` | [0.002, 0.5] |  |  |
| `friction` | float |  | `1.0` | [0, 2] |  | sliding friction |
| `grab_radius` | float | m | `0.04` | [0.005, 0.2] |  | the beak grabs a handle this close to its tip (R18) |
| `max_hold_force` | float | N | `3.0` | [0.01, 100] |  | the grab lets go when holding it takes more force |
| `length` | float | m | `0.2` | [0.08, 0.5] |  | rod length along +x |
| `width` | float | m | `0.016` | [0.006, 0.04] |  | square rod side |
| `handle_size` | float | m | `0.015` | [0.005, 0.05] |  | how far the handle sticks out of the -x end |

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
| `mount` | enum |  | `floor` | `floor`, `wall` |  | floor: stepped on; wall: pressed with the beak tip |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what presses it: body, object, npc or any (not shells or rope knots) |
| `size` | vec | m | `[0.25, 0.25]` | [0.02, 2] |  | plate size [x, y]; wall: [width, height] |
| `height` | float | m | `0.2` | [0.02, 1] |  | wall: plate centre above the floor |
| `facing` | float | deg | `0.0` | [-180, 180] |  | wall: the direction the plate faces (0 = +x) |
| `max_height` | float | m | `0.2` | [0.01, 1] |  | objects press it when their centre is lower than this |
| `latch` | bool |  | `True` |  |  | M1: stay on after the body leaves (oneshot); false: momentary |

### `target`

役割`sensor`、更新段階`input`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `size` | vec | m | `[0.2, 0.2]` | [0.03, 2] |  | board [width, height] |
| `height` | float | m | `0.2` | [0.05, 2] |  | board centre above the floor |
| `yaw` | float | deg | `0.0` | [-180, 180] |  | direction the face looks |
| `min_speed` | float | m/s | `1.0` | [0, 30] | 可 | slowest hit that counts |
| `mode` | enum |  | `latch` | `pulse`, `latch` |  | pulse: 1 for the body step of a hit; latch: stays 1 |
| `trigger` | enum |  | `shell` | `shell` |  | only cannon shells count |

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

### `wind`

役割`prop`、更新段階`device`。

| 値 | 種類 | 単位 | 既定値 | 範囲 | 実行中の変更 | 説明 |
|---|---|---|---|---|---|---|
| `enabled` | int |  | `1` | [0, 1] | 可 | 0 turns the trap off (setters, difficulty steps) |
| `size` | vec | m | `[0.6, 0.6]` | [0.05, 5] |  | zone size [x, y] |
| `height` | float | m | `0.4` | [0.05, 2] |  | zone height above the floor |
| `yaw` | float | deg | `0.0` | [-180, 180] | 可 | direction the wind blows (0: +x, 90: +y) |
| `power` | float | N | `1.0` | [0, 20] | 可 | force on each body or object in the zone |
| `trigger` | enum |  | `any` | `body`, `object`, `npc`, `any` |  | what it pushes |

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

### 組み立てプリセット

- `puzzles/ball/ball_ramp`
- `puzzles/ball/bowling`
- `puzzles/ball/color_sort`
- `puzzles/ball/mini_soccer`
- `puzzles/coins/coin_lock`
<!-- mpg parts --markdown: end -->
