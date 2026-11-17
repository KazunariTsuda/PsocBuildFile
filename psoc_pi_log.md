# psoc_pi_log.py

PSoC の UART と、ラズパイ側の CAN・電源・カーネルを CSV に残す。走行アプリとは別プロセスで動かし、アプリが bus-off で止まっても記録は続ける。

ファームが出す UART の形は [log-uart.md](../psoc_doc/log-uart.md)。このスクリプトは、その短い行を受け取って列に分ける。

## 起動

ラズパイで、このファイルがあるディレクトリから実行する。

```bash
python3 psoc_pi_log.py
python3 psoc_pi_log.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 --dir log
python3 psoc_pi_log.py --self-test
```

`--port` を省略すると `/dev/serial/by-id` から CP210x を 1 つ探す。`serial0` や `ttyAMA0` はバッテリ監視に使うので開かない。UART は 115200 8N1。

接続できたら `LOG` と改行を送り、EEPROM の保存ログを受け取ってからライブ行に移る。保存ログが 15 秒以内に始まらなければ、ライブ行だけ残す。

止めるときは Ctrl+C、または SIGTERM。

| オプション | 既定 | 意味 |
|---|---|---|
| `--port` | CP210x を探す | UART の by-id パス |
| `--dir` | `log` | CSV を置くディレクトリ |
| `--interval` | 10 | 状態 CSV の間隔（秒）。1 以上 |
| `--can-ms` | 150 | `ip link` を見る間隔（ミリ秒）。50 から 500 |
| `--iface` | `can0` | CAN インターフェイス |
| `--app-unit` | 空。環境変数 `PSOC_PI_LOG_APP_UNIT` | 起動と終了を残す systemd unit。空なら残さない |
| `--keep-days` | 14 | UART と状態 CSV を残す日数 |
| `--can-keep-days` | 30 | CAN とカーネル CSV を残す日数 |

## ファイル

起動のたびに、その時刻のファイルを 4 つ作る。日付が変わっても、その起動のファイルに書き続ける。

```text
log/uart/psoc_uart_YYYYMMDD_HHMMSS.csv
log/status/pi_status_YYYYMMDD_HHMMSS.csv
log/can/can_event_YYYYMMDD_HHMMSS.csv
log/kernel/kern_YYYYMMDD_HHMMSS.csv
```

時刻は現地時刻。同じ秒にもう一度起動して、既存ファイルの見出しが今の列と違うときは、名前の末尾に `_v2` を付けたファイルへ書く。

`pi_time` は現地時刻の ISO 表記で、ミリ秒とオフセットまで入る。行を並べるときはこれを使う。

## UART の CSV

PSoC の 1 行を 1 行の CSV にする。`message` は時刻を除いた受信そのもの。分かった行は、ほかの列にも値を入れる。

| kind | いつ |
|---|---|
| `note` | このスクリプトが出した行。接続、保存ログの開始と終了 |
| `dump` | EEPROM ダンプの 1 件。`psoc_time` はダンプの時刻。`sts1` から `sts4` は行の状態バイト |
| `event` | `START 1` のような動作ログ。カンマで始まらない行 |
| `M` | モータ状態 |
| `CANOFF` | bus-off |
| `CANPASV` | error-active / warn / passive の変化 |
| `CANERR` | 1 秒のエラー集計 |
| `CANSTOP` | CAN を止めてからの秒 |

ダンプの 1 行は `LOGDUMP <when> <sts1>,<sts2>,<sts3>,<sts4> <text>` です。`sts` は 2 桁の 16 進で、CSV の `sts1` から `sts4` に入ります。`message` はテキストだけです。`START 1` の sts2 はリセット要因の生値、sts3 は分類です。`CANOFF` の sts3 は TEC、sts4 は REC です。古いファームの、sts が付いていない行もそのまま `message` に残します。

モータ状態の UART は次の並び。空欄は未受信。

```text
HH:MM:SS,M,sw1,trq1,din1,sw2,trq2,din2,無指令秒,衝突ms,M1無受信秒,M2無受信秒
```

CSV では次のように読む。`sw` は状態ワードの 16 進。判定に bit4 は使わない。

| 列 | 内容 |
|---|---|
| `m1_sw` / `m2_sw` | 状態ワード |
| `m1_state` / `m2_state` | 準備前、励磁不可、準備完了、スイッチオン、運転可、急停止、異常処理、異常、不明 |
| `m1_remote` / `m2_remote` | 遠隔、またはローカル |
| `m1_target` / `m2_target` | 到達、または未到達 |
| `m1_speed` / `m2_speed` | 速度0、または回転 |
| `m1_limit` / `m2_limit` | 内部制限。立っていなければ空 |
| `m1_warn` / `m2_warn` | 警告。立っていなければ空 |
| `m1_trq` / `m2_trq` | トルク。バイトが無ければ `不明` |
| `m1_hwto` / `m2_hwto` | 60FDh の bit3。入、切、または不明 |
| `m1_son` / `m2_son` | bit16 |
| `m1_stop` / `m2_stop` | bit21 |
| `nocmd_s` | 速度指令が無い秒 |
| `corr_ms` | 衝突タイマのミリ秒 |
| `m1_silent_s` / `m2_silent_s` | 状態フレームが無い秒 |

CAN の列は、その行の種類だけ埋まる。

| 列 | 入る行 |
|---|---|
| `can_tec` `can_rec` `can_es` | `CANOFF`。TEC、REC、エラー状態レジスタ |
| `can_class` | `CANPASV`。`active`、`warn`、`passive` |
| `can_ack` `can_bit` `can_stuff` `can_form` `can_crc` `can_arb` `can_lost` | `CANERR` のその秒の件数 |
| `can_txfail` | 送信に失敗した ID と回数。`181=3` のように空白区切り |
| `can_stop_s` | `CANSTOP`。止めてからの秒 |

## 状態の CSV

`--interval` 秒ごとに 1 行。既定は 10 秒。

`cpu_temp_c` はラズパイの CPU 温度で、`vcgencmd measure_temp` の値。コマンドが使えなければ `/sys/class/thermal/thermal_zone0/temp` を使う。`cpu_pct` は `/proc/stat` から計算した使用率、`load1` は 1 分ロード、`mem_used_pct` はメモリ使用率、`throttled` は `vcgencmd get_throttled` の値と、電圧低下や温度制限などの名前。

`amp`、`temp1`、`temp2`、`temp3` は CAN の 0x090 で、PSoC が送る電流と温度。これは CPU 温度ではない。

CAN の列は `ip -details -statistics link show can0` の値。`can_link` は UP、DOWN、なし。`can_state` は ERROR-ACTIVE など。`can_restarts` から `can_bus_off` までは統計カウンタ。`can_rx` と `can_tx` はパケット数、`can_rx_err` と `can_tx_err` はエラー数、`can_rx_drop` と `can_tx_drop` は取りこぼし。`clock`、`bitrate`、`sample_point`、`sjw`、`restart_ms`、`qlen` はビットタイミング。TEC と REC は記録しない。PSoC の CAN 状態は UART の `CANOFF`、`CANPASV`、`CANERR`、`CANSTOP` に残る。

`last_090` と `age_090_s` のように、次の ID を最後に見た時刻と、そこからの秒を付ける。0x090、0x0A0、0x0C0、0x181、0x182、0x381、0x382。フレームが無ければ空。

## CAN の CSV

150 ms ごとに `ip link` を見て、次のどれかが変わったときだけ 1 行書く。受信パケット数の増加だけでは書かない。

`can_link`、`can_state`、`can_restarts`、`can_bus_errors`、`can_arbit_lost`、`can_error_warn`、`can_error_pass`、`can_bus_off`、`can_rx_err`、`can_rx_drop`、`can_tx_err`、`can_tx_drop`。

| kind | いつ |
|---|---|
| `timing` | クロック、ビットレート、サンプル点、SJW、restart-ms、qlen が変わった |
| `state` | 上の状態かカウンタが変わった |
| `error` | エラーフレームを 1 つ受信した |
| `app` | `--app-unit` の ActiveState が変わった |

`error` の列は、フレームの内容を分けたもの。

| 列 | 内容 |
|---|---|
| `err_cause` | ACK、bit、stuff、form、CRC、bus-off などの短い原因 |
| `err_class` | フレームの分類。ACK、PROT、BUSOFF など |
| `err_prot` | プロトコル違反のビット |
| `err_loc` | エラーの位置。CRC_SEQ、ACK、DATA など |
| `err_ctrl` | error-warning、passive、overflow |
| `err_data` | データ 8 バイトの 16 進 |

`detail` には、ビットタイミングが PSoC の 16tq、サンプル 0.875、SJW 2 と同じかどうかと、`restart-ms` が 0 のときは bus-off 後に自動復帰しないことを書く。

## カーネルの CSV

`journalctl -k -f` から、次を含む行だけ残す。`mcp251`、`can`、`spi`、`overflow`、`bus-off`、Oops、panic、hung task、メモリ不足、ウォッチドッグ、低電圧、温度、mmc、ext4、USB 切断。`kern_time` は journal の時刻、`message` は本文。

起動時に、直前の起動のカーネルログ末尾 400 行から同じ条件の行をコピーする。`前回起動の該当行` の次に並ぶ。該当が無ければその旨を 1 行書く。電源断でジャーナルへ届かなかった行は、ここにも無い。

## 確認

ロジックだけの確認は次で、実機も CAN も要らない。

```bash
python3 psoc_pi_log.py --self-test
```

`self-test ok` と出れば終了コードは 0。
