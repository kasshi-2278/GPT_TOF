# 走行プログラム配布

## 停止後に実機ログを保存するv23

- [プログラムと使い方のZIP](TOFsimu_program_log_on_stop.zip)
- [Pythonプログラム単体](sonic_drive_v23_log_on_stop.py)

ラズパイでZIPを解凍し、プログラムのあるフォルダをターミナルで開いて `python3 sonic_drive_v23_log_on_stop.py` を実行します。Enterで走行開始、Ctrl+Cで停止します。停止後、走行時のTXTログが起動した作業フォルダに保存され、ファイルの絶対パスが端末に表示されます。そのTXTを `log_viewer.html` に読み込むと、センサー値と走行判断を図で確認できます。

この版は保存してある元v23の実機コードにログ保存だけを追加しています。GPIO/PWM、センサー、走行判断の初期設定は変更していません。実機での走行は未検証です。

SHA256:

- ZIP: `efc369ccffd6b4717eece3f1cb524fbf08a440ce3f0da00b3650f39345341a5c`
- Python: `1ccabdddf99810cbda0e5de1c43f60d61a18cd50670e0d0b1f602859e4158c8c`
