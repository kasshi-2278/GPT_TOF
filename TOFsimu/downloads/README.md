# v27 側面接触後の復帰修正版

- [v27プログラムZIP](TOFsimu_side_recovery_v27.zip)
- [v27 Python単体](sonic_drive_v27_side_recovery.py)
- [詳しい復帰条件・使い方](sonic_drive_v23_log_on_stop_README.txt)
- [現在の軌跡](start1_one_lap.png)
- [現在の1周リプレイZIP](start1_one_lap_replay.zip)
- [シミュレーター更新パッチ](TOFsimu_start1_one_lap_patch.zip)

ZIPを解凍して `python3 sonic_drive_v27_side_recovery.py` を実行してください。開始前の測距、Enter後の再測距発進、Ctrl+C後のTXT/JSONL保存を維持しています。

後退開始25cm・右後退-40/左後退+40は維持しています。一連の復帰中は後退方向を固定し、停止中も舵を保ちます。接触側の距離の改善を3回確認して低速離脱へ移り、通常走行へは側面25cm以上と接近解消の実測を確認して戻します。340cmへの跳躍だけでは復帰完了にしません。後退だけでは側面が離れない場合は、実測改善と進行側の余裕を確認してからアクセル25・操舵20で離れます。詳細条件は説明書を参照してください。

GPIO/PWM設定・測距関数・main/ESC出力部分は前回版と完全一致しています。JSONLには復帰段階・方向固定・確認回数を追加しました。

旧幾何モデルでスタート1から57.66秒、接触0・後退21パルスで1周しました。左右の壁に近づけた条件では後退5パルスで離脱し、15秒まで接触0です。実機の新しい軌跡やHC-SR04音響条件は未検証です。
