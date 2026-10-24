# 直線・凹み進入・発進方向の修正版

- [修正版プログラムZIP](TOFsimu_straight_recess_fix.zip)
- [Python単体](sonic_drive_v23_log_on_stop.py)
- [使い方](sonic_drive_v23_log_on_stop_README.txt)
- [修正前後の軌跡](straight_recess_comparison.png)
- [現在の軌跡](start1_one_lap.png)
- [1周リプレイZIP](start1_one_lap_replay.zip)
- [シミュレーター更新パッチ](TOFsimu_start1_one_lap_patch.zip)

ZIPを解凍して `python3 sonic_drive_v23_log_on_stop.py` を実行します。停止状態で3回測距し、中央値と方向候補を表示します。Enter後に再測距し、最も見通せる方向へ発進します。距離差が小さい時は正面優先です。Ctrl+Cで停止し、作業フォルダにTXT/JSONLログを保存します。

直線の小さな測距差には反応せず、部分的に広がる右側の凹みでは左壁との間隔を維持します。壁が見えないだけで右へ切る判断も廃止し、壁端の連続観測と区別しました。GPIO/PWM初期設定・測距関数・出力換算・後退手順は前回と同じです。

旧幾何モデルではスタート1から50.16秒で1周、衝突0・後退0です。開始前の測距時間を含みます。最初の直線（x=331〜750cm、最初の8秒）の開始位置からの最大横ずれは約38.43cmから0.01cm未満になりました。これは同じ近似モデルでの比較で、実機や後期HC-SR04音響モデルは未検証です。

リプレイZIPは解凍後 `start1_one_lap.html` をブラウザで開いて再生してください。パッチは既存のTOFsimuフォルダに上書きしてください。コースや車体の設定は変えていません。
