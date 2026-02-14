# pacecat_driver README

## 概要

このパッケージは、LiDARデバイス（PACE-CAT等）からシリアル(UART)経由でデータを受信し、
ROS2の`sensor_msgs/msg/LaserScan`として/scanトピックにパブリッシュするドライバです。

## パケット仕様

- 1パケット: 98バイト
- ヘッダー: 0xCF 0xFA
- 構造:
  - [0-1]   ヘッダー (0xCF 0xFA)
  - [2-3]   点数 (uint16_t, little endian, 通常30)
  - [4-5]   開始角度 (uint16_t, 0.1度単位, little endian)
  - [6-7]   角度ステップ (uint16_t, 0.1度単位, little endian)
  - [8-97]  点データ (30点分, 1点3バイト)
    - 1点あたり: [Intensity(1B), Distance(2B, little endian)]

## ROS2ノードの使い方

### ビルド

```bash
colcon build --packages-select pacecat_driver
```

### 実行例

```bash
ros2 run pacecat_driver pacecat_driver_node
```

### 主なパラメータ
- `port` (string): シリアルポート名 (例: /dev/ttyAMA0, /dev/ttyUSB0)
- `baudrate` (int): ボーレート (例: 230400)
- `read_size` (int): 1回のreadで読む最大バイト数 (例: 1024)
- `timer_frequency_hz` (double): データ処理周期Hz (例: 10.0)


### 出力トピック
- `/scan` (sensor_msgs/msg/LaserScan)
  - 600点分（360度分）がたまるごとにpublish
  - 0度スタートのパケットから1周分を蓄積

## 注意事項
- パケットの先頭（0xCF 0xFA）と0度スタートを検出してから600点分を蓄積します。
- 1パケット30点、20パケットで1周（600点）となります。
- 距離はmm単位→m単位に変換してLaserScanに格納されます。
- IntensityもLaserScanのintensitiesに格納されます。

---

ご質問・不具合はリポジトリのIssueまたは開発者までご連絡ください。
