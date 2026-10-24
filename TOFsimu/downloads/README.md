# 近接時の左右後退を反映した修正版

- [最新プログラムZIP](TOFsimu_reverse_reference_fix.zip)
- [Python単体](sonic_drive_v23_log_on_stop.py)
- [詳しい使い方・後退条件](sonic_drive_v23_log_on_stop_README.txt)
- [現在の軌跡](start1_one_lap.png)
- [現在の1周リプレイZIP](start1_one_lap_replay.zip)
- [シミュレーター更新パッチ](TOFsimu_start1_one_lap_patch.zip)

開始前の3回測距、Enter後の再測距発進、Ctrl+C後のTXT/JSONL保存を維持しています。

正面35cm未満、または提示された斜め35cmと左右最短距離の比較で後退します。右側が近ければアクセル-50・ハンドル-40、左側なら-50・+40。同距離は右を選び、正面近接で両分岐が成立して上書きされる問題を修正しました。側面8cm未満の既存判定も維持しています。

0.12秒の短い後退と停止・再測距を繰り返し、距離に余裕があることを2回確認して前進へ復帰します。回数上限は60パルス、1秒の前進でリセットします。初期設定・測距関数・実機出力部分は前回配布版と完全一致しています。

旧幾何モデルで70.05秒、接触0・後退57パルスで1周しました。35cm条件では狭い箇所で後退が多くなります。実機や後期HC-SR04音響モデルは未検証です。

プログラムZIPを解凍し `python3 sonic_drive_v23_log_on_stop.py` で実行します。リプレイZIPは解凍後 `start1_one_lap.html` をブラウザで開きます。パッチは既存のTOFsimuフォルダに上書きしてください。
