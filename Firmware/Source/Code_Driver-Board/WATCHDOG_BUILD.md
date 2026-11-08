# Driver Board 1.3.0-wd4（看门狗、自动续绕、料盘预设、无线刷机）

基于官方 1.3.0，加上 `fix-freeze-in-1.3.0` 分支的 LEDC 修复。App 里显示的版本号仍是 1.3.0，实际版本在状态里的 `FW_B`、串口 `#BOOT` 行和网页版信息卡里能看到。

## 改了什么

| 项目 | 内容 |
|---|---|
| LEDC 占空比更新 | 不再在关中断时死等 `duty_start`。硬件上一次还没更新完时，新值先排队，下一轮 `loop()` 再写。取代之前的二进制补丁。 |
| RTC 看门狗 | 用 RTC 慢时钟，不依赖 APB/PLL。`loop()` 健康时才喂狗，健康指：主循环在跑，`esp_timer` 和 RTC 时钟对得上，风扇、舵机、指示灯、STEP 的 LEDC 配置和引脚路由都正常且计数器在走。6 s 没喂 → 系统复位；再过 4 s 还卡着 → 整片 RTC 复位（相当于按 EN）。 |
| 自动续绕 | 绕线时每 250 ms 把进度和导丝舵机位置存进 RTC 内存。看门狗、崩溃、欠压复位后，先显示"暂停"（橙灯）4 s，然后自动接着绕，进度不丢。连续 3 次在 60 s 内又崩，就停在暂停状态等人处理。 |
| 上电 / 按 EN / 刷完固件 | 进度恢复成"暂停"，**电机不会自己转**。按一下按钮或在 App 里点继续；长按 1 s 取消。 |
| 欠压检测 | 门限从 2.43 V 提到 2.80 V（`BROWNOUT_LEVEL 7`）。3.3 V 掉下去时干净地复位再续绕，不会进入不确定状态。 |
| CPU / 无线电 | 恢复官方设置：240 MHz，蓝牙 +6 dBm，Wi-Fi 不限功率，绕线时 Wi-Fi 不关。之前降功率只是 5 V 偏弱时的临时措施，开关还在（`CPU_FREQ_MHZ`、`BLE_TX_POWER_DBM`、`WIFI_TX_LIMIT`、`WIFI_OFF_WHILE_WINDING`）。 |
| 导丝 | 舵机按 0.01° 级别平滑移动，不再每圈跳一个整数角度。`SV_MMD` 是每度对应的导丝毫米数，可以在网页版里用实测行程校准。 |
| 料盘 | `SV_W` 记录料盘内宽。可以存最多 6 个料盘预设（名字、左右角度、内宽），在网页版里一键切换。 |
| 无线刷机 | 两种方式，见下面。只在没在绕线时接受。 |
| 串口诊断 | 115200 波特率。开机打印复位原因和上一次运行到哪；之后每秒一行 `#HB`。 |
| 开机死循环 | 30 s 内异常复位 4 次 → 做一次整片 RTC 复位。 |

所有参数都在 `.ino` 开头的 `Robustness / power settings` 一段。

## 编译和烧录

第一次（或者板子里还是官方固件时）只能用 USB。**只插 USB，不接 12 V**：D1 短接以后，USB 5V 和 UBEC 5V 是连在一起的。

```sh
cd Firmware/Source/Code_Driver-Board
cp local_ota.example.ini local_ota.ini     # 改里面的 password（这个文件不进 git）
pio run
esptool --chip esp32 -p /dev/cu.usbserial-0001 -b 460800 write-flash \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/driver-board/firmware.bin
```

只写 `boot_app0` 和应用分区，板子里存的设置（速度、方向、Wi-Fi、料盘预设等）都保留。`boot_app0` 让板子从 app0 启动；如果之前做过 OTA，板子可能在跑 app1。

### 之后：无线刷机

板子要连上 Wi-Fi，并且没在绕线（空闲或暂停都行）。

**A. 从电脑直接推（ArduinoOTA）**

```sh
pio run -e ota -t upload        # 编译并推到 lts-respooler.local
```

密码和地址在 `local_ota.ini` 里。mDNS 找不到板子时，把 `host` 改成板子的 IP（串口 `#OTA push ready` 行里有）。

**B. 网页版里填地址**

```sh
python3 -m http.server 8765 -d .pio/build/driver-board
```

网页版信息卡 → 固件那一行点 Update → "Update from URL" 填 `http://<电脑 IP>:8765/firmware.bin` → Install。板子自己下载、校验、重启。刷完把 http.server 停掉。

**注意：** 官方 App 里的"更新"会装回官方固件，这些改动就没了，而且官方固件不支持上面两种无线刷机，只能再用 USB 刷回来。网页版在检测到自定义固件时不会显示官方更新按钮。

## 串口输出

```
#BOOT fw=1.3.0-wd4 reset=WDT raw=9 boots=12 abnormal=2 resumes=1
#PREV up=2710s state=R crumb=16 loopmax=8ms clk=1.000 fault=none heap=187000
#JOB interrupted at 2705s, servo 93.4 deg -> auto-resume
#HB up=5 st=P run=0 el=2705 iv=700/116 sv=93.4 lps=4100 lmax=6 clk=1.000 heap=186000 wifi=ok ble=0 f=none
#OTA push ready: lts-respooler.local (10.0.0.123) auth=yes
```

- `reset=`：这次为什么启动。`WDT` = 看门狗，`BROWNOUT` = 欠压，`PANIC` = 程序崩溃，`POWERON` = 上电或按 EN，`SW` = 软件重启（比如刷完固件）。
- `#PREV`：复位前最后一次心跳：运行了多久、状态、`crumb`（卡在 `loop()` 的哪一段）、时钟比、故障。
- `#FAULT clock` / `#FAULT LEDC`：健康检查发现问题，看门狗随后复位。
- `#OTA ...`：无线刷机的来源、HTTP 状态码、出错原因。

串口命令（一行一个）：`!help` `!diag` `!hb 0` / `!hb 1` `!wdt`（让 loop 卡死，测看门狗）`!fault`（停止喂狗）`!panic` `!reset`

## 怎么测

1. 开始绕线，等 10 s。
2. 串口发 `!wdt`。约 6 s 后板子复位，串口出现 `reset=WDT` 和 `auto-resume`，4 s 后电机接着转，App 里进度接着走。
3. 再试一次 `!panic`，应该一样能续上。
