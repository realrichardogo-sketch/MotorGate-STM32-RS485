# M19：电脑串口控制网关（待现场验收）

本包含 `Gateway_G3_uart_control` 和原样保留的 `PPP03_M18_rs485_control`。
G3 在已通过带动力短转验收的 G2 上增加 USART1 电脑命令输入。网关仍以
USART2/RS485 与电机板通信，电机板固件保持 M18；无需重烧电机板。

## 控制链路与边界

电脑串口（115200 8N1）→网关 USART1 →既有 G2 单次命令处理→网关
USART2/RS485→电机板 M18→OLED/编码器自动停机。接收使用 USART1
中断与小缓冲区，避免网关轮询电机期间丢掉短命令。每行只接受大写英文
命令和 CR/LF，异常长度、错误字符、溢出均丢弃，绝不执行部分命令。

| 输入一行 | 意义 |
| --- | --- |
| `STATUS` | 网关输出上次轮询的版本、状态和帧计数；不会转动 |
| `ARM` | 远程预备，有效期约 5 秒 |
| `START` | 在已预备且有效时启动 |
| `PULSE` | 预备，收到 ARMED 状态后自动启动一次短转 |
| `STOP` | 请求正常停机，覆盖待处理命令 |

输入命令后看到 `SERIAL QUEUED` 仅表示网关接收，看到 `CMD ACK` 才表示
电机板返回 Modbus 命令回显。最终结果请看 `STATUS` 和电机板 OLED。
串口 STOP 受轮询与通信超时影响，现场停机用电机板 KEY3；若持续运动，
断开金属电机动力电源的交流插头。M18 保留约 1 秒的运行上限和编码器限位。

## 串口连接

现场已通过现有 COM7 串口连接收到网关 `G1 OK`，并发送 `STATUS`
收到 `G1 STATUS`，双向链路已验证。复用这套连接，无需增接转接器或
改动六芯排线。网关程序使用 USART1 的 PA9 (TX) / PA10 (RX)，
串口参数为 115200 8N1，发送大写命令并附加回车换行。

若在另一套硬件上重新布线，须先依据实物丝印或原理图确认 USART1
的 TX、RX、GND 引出位置和电平。外接 USB 转 TTL 时需 **3.3V 电平**，
RX→网关 TX、TX→网关 RX、GND→GND，转接器电源脚留空。USB 转 RS485
不能直接接 USART1 TTL 引脚。先断开金属电机电源和逻辑电源再改线；
六芯排线带逻辑供电，不能带电插拔。

## 首次烧录的复现步骤

1. 留存已经验收的 G2/M18 工程和 HEX。金属电机电源交流插头保持拔下；
   网关与电机板继续用六芯排线连接、只恢复逻辑供电。
2. 用 Keil 打开 `Gateway_G3_uart_control/MDK-ARM/gateway_32.uvprojx`，
   执行 Rebuild，要求 **0 Error(s)**；核对新 HEX 的完整路径和生成时间。
   项目沿用旧输出名 `gateway_G1_readonly.hex`，名字不能用来判断版本。
3. 只把新生成的网关 HEX 烧录到已确认的 **128 KB 网关板**，Verify OK。
   电机板继续 M18，不烧录；ST-LINK 接线若要换板，先完全断电。
4. 热插调试连接网关，只读取 G1_frames_ok 是否递增、G1_reg[0] 是否
   为 18。串口终端设 `115200 8N1`，发送 `STATUS` 和换行；检查收到
   `G1 STATUS` 且板间轮询继续。若日志乱码，先复核串口波特率和接线。
5. 仍不接电机动力电源，先发送 `ARM`，OLED 短暂 ARMED 后回待机；
   再发送 `PULSE`，OLED 应短暂 RUN、最后 NO ENC；`STOP` 可使 OLED STOP。
   预期命令计数增长且 failed 不增加。
6. 无动力验收通过后，才在电机空载且人员离开运动部位的情况下进行
   一次带动力 `PULSE`，验收短转、DONE、自停与计数。不要连续快速发送。

## 2026-09-27 现场验收记录

- 网关 Keil Rebuild：Code=8028、RO-data=312、RW-data=88、ZI-data=1408，
  **0 Error(s), 0 Warning(s)**；ST-LINK 烧录网关 HEX checksum `0x000BCF29`，
  `Verification...OK`。电机板保持已经验收的 M18。
- SSCOM COM7，115200 8N1：发送 `STATUS` 收到 `G1 STATUS`；持续
  `G1 OK`，版本 18、bad=0、timeout=0，网关→电脑、电脑→网关、板间
  RS485 通路均有证据。
- 不接电机动力电源：`ARM` 收到 `SERIAL QUEUED` 与 `CMD ACK`，
  OLED ARMED 后恢复 READY；`PULSE` 收到两个 `CMD ACK`，状态
  READY(0)→ARMED(1)→NO ENC(5)，OLED `M18 NO ENC`；`STOP`
  收到 `CMD ACK`，状态 STOP(3)，OLED `M18 STOP`。
- 接通原有动力电源后空载发送一次 `PULSE`：两次 `CMD ACK`，状态
  STOP(3)→ARMED(1)→RUN(2)→DONE(4)。电机约转 1/4 圈自行停机，
  OLED `M18 DONE T13 D059 S20 R000 A0954 C550 F057 K002`。

源文件离线语法及命令解析模拟通过；上述 Keil 编译与带电实测由用户在实板
执行。本 ZIP 只打包源码和说明，现场生成的 HEX 未收入包内。
