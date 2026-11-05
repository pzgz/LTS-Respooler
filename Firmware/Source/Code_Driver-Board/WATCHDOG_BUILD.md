# Driver Board 1.3.0-wd1（看门狗 + 自动续绕版）

基于官方 1.3.0，加上 `fix-freeze-in-1.3.0` 分支的 LEDC 修复。App 里显示的版本号仍是 1.3.0。

## 改了什么

| 项目 | 内容 |
|---|---|
| LEDC 占空比更新 | 不再在关中断时死等 `duty_start`；硬件上一次还没更新完时，新值先排队，下一轮 `loop()` 再写。取代之前的二进制补丁。 |
| RTC 看门狗 | 用 RTC 慢时钟，不依赖 APB/PLL。`loop()` 健康时才喂狗：主循环在跑、`esp_timer` 和 RTC 时钟对得上、风扇/舵机/指示灯/STEP 的 LEDC 配置、引脚路由都正常且计数器在走。6 s 没喂 → 系统复位；再过 4 s 还卡着 → 整片 RTC 复位（相当于按 EN）。 |
| 自动续绕 | 绕线时每 250 ms 把进度、导丝舵机位置存进 RTC 内存。看门狗、崩溃、欠压复位后：先显示"暂停"（橙灯）4 s，然后自动接着绕，进度不丢。连续 3 次在 60 s 内又崩，就停在暂停状态等人处理。 |
| 上电 / 按 EN | 进度恢复成"暂停"，**电机不会自己转**，按一下按钮或在 App 里继续。长按 1 s 取消。 |
| 欠压检测 | 门限从 2.43 V 提到 2.80 V（`BROWNOUT_LEVEL 7`）。3.3 V 掉下去时干净地复位再续绕，而不是进入不确定状态。 |
| 省电 | CPU 240 → 160 MHz；蓝牙发射 +6 → 0 dBm；Wi-Fi 发射 19.5 → 11 dBm；**绕线时 Wi-Fi 整个关掉**，停下 5 s 后再连。 |
| 串口诊断 | 115200 波特率。开机打印复位原因和上一次运行到哪；之后每秒一行 `#HB`。 |
| 开机死循环 | 30 s 内异常复位 4 次 → 做一次整片 RTC 复位。 |

所有参数都在 `.ino` 开头的 `Robustness / power settings` 一段。

## 编译和烧录

```sh
cd Firmware/Source/Code_Driver-Board
pio run                                   # 第一次会下载 ESP32 工具链
esptool --chip esp32 -p /dev/cu.usbserial-0001 -b 460800 \
  write-flash 0x10000 .pio/build/driver-board/firmware.bin
```

只写 0x10000（应用分区），板子里存的设置（速度、方向、Wi-Fi 等）都保留。

**注意：** 在 App 里点 OTA 升级会装回官方固件，这些改动就没了。

## 串口输出

```
#BOOT fw=1.3.0-wd1 reset=WDT raw=9 boots=12 abnormal=2 resumes=1
#PREV up=2710s state=R crumb=16 loopmax=8ms clk=1.000 fault=none heap=187000
#JOB interrupted at 2705s, servo 93.4 deg -> auto-resume
#HB up=5 st=P run=0 el=2705 iv=700/116 sv=93.4 lps=4100 lmax=6 clk=1.000 heap=186000 wifi=off ble=0 f=none
```

- `reset=`：这次为什么启动。`WDT` = 看门狗，`BROWNOUT` = 欠压，`PANIC` = 程序崩溃，`POWERON` = 上电或按 EN。
- `#PREV`：复位前最后一次心跳：运行了多久、状态、`crumb`（卡在 `loop()` 的哪一段）、时钟比、故障。
- `#FAULT clock` / `#FAULT LEDC`：健康检查发现问题，看门狗随后复位。

串口命令（一行一个）：`!help` `!diag` `!hb 0` / `!hb 1` `!wdt`（让 loop 卡死，测看门狗）`!fault`（停止喂狗）`!panic` `!reset`

## 怎么测

1. 开始绕线，等 10 s。
2. 串口发 `!wdt`。约 6 s 后板子复位，串口出现 `reset=WDT` 和 `auto-resume`，4 s 后电机接着转，App 里进度接着走。
3. 再试一次 `!panic`，应该一样能续上。
