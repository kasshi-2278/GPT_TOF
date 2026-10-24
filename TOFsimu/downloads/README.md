# v28 旋回方向確認・斜め前の復帰修正版

- [v28プログラムZIP](TOFsimu_v28_turn_diagonal_fix.zip)
- [v28 Python単体](sonic_drive_v28_confirmed_turn.py)
- [詳しい条件・使い方](sonic_drive_v23_log_on_stop_README.txt)
- [現在の軌跡](start1_one_lap.png)
- [1周リプレイZIP](start1_one_lap_replay.zip)
- [シミュレーター更新パッチ](TOFsimu_start1_one_lap_patch.zip)
- [検証結果](v28_validation.json)

ZIPを解凍して `python3 sonic_drive_v28_confirmed_turn.py` を実行してください。起動時と停止後のログにv28の版名・ファイル名・SHA256を表示/記録します。

正面接近と開口を連続確認し、斜め前の340cm単独で左旋回を選びません。斜め前25cm未満では同じ側へ後退し、その斜めセンサーの35cm以上への改善を確認して低速離脱します。壁が射線から外れた後も、過去の斜め実測と現在の側面実測を合わせて判定します。側面接触後の方向固定・舵保持も維持しています。

GPIO/PWM初期設定・測距関数・PWM換算・ESC出力手順は添付v27と同一です。正面の極小値が後退しても固定される場合は、測距か移動状態の確認のため停止します。

29件のテストを通過し、旧幾何モデルで57.94秒、接触0・後退21パルスで1周しました。左右の斜め前の壁からも、各14パルス後に通常走行へ復帰し12秒まで接触0でした。実機・後期HC-SR04音響モデルは未検証です。
