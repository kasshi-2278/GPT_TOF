# TOFsimu 配布パッケージ

## 図付きログに対応した更新版

- [シミュレータ更新版ZIP（約1.5MB）](https://github.com/kasshi-2278/GPT_TOF/raw/refs/heads/main/TOFsimu/downloads/TOFsimu_simulator_visual_logs.zip)
- [ログビューアーだけのZIP（約17KB）](https://github.com/kasshi-2278/GPT_TOF/raw/refs/heads/main/TOFsimu/downloads/TOFsimu_log_viewer.zip)

ZIPを解凍し、`log_viewer.html` をブラウザで開いてください。「ログを開く」で実機のTXT/LOGまたはシミュレータの `replay.json` を選びます。実機ログは測距・操舵・前後進・判断名を図示し、実際の位置・軌跡は推定しません。シミュレータ記録では、保存したコース・車体・軌跡と文字ログを時刻に合わせて表示します。「図付きHTMLを保存」で、記録入りの1ファイルとして保存できます。

更新版シミュレータは走行終了後に「図付きログHTML」を保存できます。保存先にも `visual_log.html` が追加されます。以前のTXT/CSV/JSON出力は維持しています。

走行プログラム・GPIO/PWM初期設定・コース・物理計算は変更していません。元に戻した実機用v23（アクセル60・後退上限999）を収録し、v24案は含めていません。送られた実機ログ自体はこの公開パッケージに含みません。

シミュレータ起動: Windowsは `START_WINDOWS.bat`、ラズパイ/macOS/LinuxはTOFsimuフォルダで `sh START_MAC_LINUX.sh`。Python 3.8以降が必要です。ログビューアー単独ではPython不要です。詳細はZIP内の `VISUAL_LOGS_JA.md` を参照してください。

検証: Python 83テスト・Node 17テスト、ブラウザでログ読込/記録移動/HTML保存と再読込/モバイル幅/実行記録との同期を確認。元の実行基盤と走行プログラムのハッシュ不変を確認しました。現在の幾何射線モデルによる互換実行であり、後期HC-SR04音響モデルは未復元です。

SHA256:

- シミュレータ更新版: `6aa84ae8b18c24bc918e772a72db60bf81accdf17d44f649aeabd197058ec0e8`
- ログビューアー: `71c4dce4e48e30a385edc7cfb32394e53f283bbb50bce67ee6f068d63ed8a88a`

## 以前の配布版

[図付きログ追加前のZIP](https://github.com/kasshi-2278/GPT_TOF/raw/refs/heads/main/TOFsimu/downloads/TOFsimu_simulator.zip)

旧ZIPのSHA256: `25be66975413a0c65e54005384f6bf3887ef9a43df22fc1fb3f311b5ab8dc1de`
