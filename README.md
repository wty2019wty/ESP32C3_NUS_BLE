# ESP32-C3 蓝牙串口调试器（BLE UART / NUS 桥接）

把一块 ESP32-C3 变成一个「无线 USB-TTL 串口线」：手机/电脑的浏览器通过
Web Bluetooth 连上它，就能像用串口助手一样收发被测设备（DUT）的串口数据。

```
Chrome/Edge 网页 (Web Bluetooth)
        │  BLE GATT：写 RX / 通知 TX（NUS 布局）
        ▼
   ESP32-C3  (ESP-IDF v6.1 + NimBLE)
        │  UART (3.3V TTL)
        ▼
     被测设备 (DUT)
```

- BLE 侧复用 **ESP-IDF 官方 `ble_uart_service` 例程的 `common/ble_uart` 组件**，
  它实现的正是事实标准 **NUS（Nordic UART Service）** 的 GATT 布局。
- 采用**明文、免配对**模式，网页端一键连接。
- 波特率可在**网页端动态修改**，并内置收发计数与环回自检，便于排障。


---

## 1. 功能特性

| 能力 | 说明 |
|---|---|
| 双向透传 | BLE 写入 ↔ UART，UART 接收 ↔ BLE 通知 |
| 标准 NUS | 任何支持 NUS 的 App/网页都能直接连 |
| 动态波特率 | 网页端发命令即可改，范围 300 ~ 5000000 |
| 自定义引脚 | 默认 TX=GPIO4 / RX=GPIO5，可在 menuconfig 改 |
| 大缓冲 | BLE↔UART 之间各有一路 4096 字节 StreamBuffer |
| 免转义分流 | 控制帧用魔术前缀识别，普通数据无需转义 |
| 诊断计数 | `B2U`/`U2B`/`DROP` 三个计数器，一眼定位断点 |
| 环回自检 | `SELFTEST` 命令，短接 TX-RX 即可自测收发通路 |

> 网页端额外提供：HEX 显示、自动滚动、本地回显、发送自动追加 CRLF。

---

## 2. GATT 布局

| 角色 | UUID | 属性 |
|---|---|---|
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | — |
| RX（网页 → 设备） | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Write / Write Without Response |
| TX（设备 → 网页） | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | Notify |

---

## 3. 硬件接线

| ESP32-C3 | 被测设备 (DUT) |
|---|---|
| GPIO4（`BRIDGE_UART_TX`） | DUT 的 **RX** |
| GPIO5（`BRIDGE_UART_RX`） | DUT 的 **TX** |
| GND | GND |


注意：

- C3 是 **3.3V TTL**。DUT 若是 **5V TTL 或 RS-232**，必须加电平转换，**禁止直连**。
- 默认避开 UART0 控制台，桥接用 **UART1**。端口与引脚可在
  `idf.py menuconfig → BLE <-> UART 桥接配置` 里改。

---

## 4. 编译与烧录

```powershell
# 1) 进入 IDF 环境（按你本机安装路径）
D:\esp\v6.1\esp-idf\export.ps1

cd G:\esp32s3\ESP32C3_NUS_BLE
idf.py build
idf.py flash monitor
```


---

## 5. 使用网页客户端

Web Bluetooth 要求**安全上下文**：通过 `https` 或 `http://localhost` 打开
`web/index.html`。

- 本地快速预览：在该目录起个静态服务器，例如
  ```powershell
  cd G:\esp32s3\ESP32C3_NUS_BLE\web
  python -m http.server 8000
  # 浏览器打开 http://localhost:8000
  ```
  也可直接部署到 GitHub Pages / 任意 HTTPS 站点。
- 浏览器需 **Chrome / Edge**（桌面或 Android），并支持 Web Bluetooth。

操作顺序：**连接 → 选择 `C3-UART-XXXX` → 订阅建立后网页显示
`[设备] READY BAUD 115200` → 收发。**

网页功能：连接/断开、波特率下拉+应用、读取状态、清空缓冲、UART 自检、
HEX 显示、自动滚动、本地回显、发送时自动追加 CRLF。

---

## 6. 控制协议

控制帧用一个**魔术前缀**与串口数据区分，避免污染透传通道：

- 前缀：`0x1B 0x42 0x4C`（即 ASCII 的 `ESC B L`）
- **一次 BLE 写入若以该前缀开头** → 视为控制命令，不转发给 DUT；
- 其余任何写入 → 原样透传到 UART。

### 6.1 网页 → 设备（写在 RX 特征）

| 命令（前缀之后） | 作用 |
|---|---|
| `BAUD=<n>` / `BAUD <n>` / `BAUD:<n>` | 切换 UART 波特率（300 ~ 5000000） |
| `POWER=<n>` / `POWER <n>` / `POWER:<n>` | 设置 BLE 发射功率（dBm，-24 ~ 上限，默认上限 +9，就近吸附到 3dB 档，**写入 NVS 掉电保存**） |
| `POWER` | 查询当前 BLE 发射功率，回 `OK POWER <dBm>` |
| `STATUS` | 查询当前波特率、射频功率、引脚、丢包计数 |
| `FLUSH` | 清空 UART 输入与收发缓冲，并把 `DROP` 清零 |
| `PING` | 连通性测试，回 `PONG` |
| `SELFTEST` | UART 环回自检，回 `PASS`/`FAIL`（需把 TX-RX 短接） |

命令前后允许有空格/Tab；未知命令回 `ERR UNKNOWN`。

### 6.2 设备 → 网页（TX 通知）

同样以 `ESC B L` 开头、`\n` 结尾的 ASCII 文本，网页会识别并渲染成橙色
`[设备]` 系统消息，不混入串口数据：

```
[设备] READY BAUD 115200 PWR 3
[设备] OK BAUD 921600
[设备] OK POWER 0
[设备] STATUS BAUD 921600 PWR 0 TX 4 RX 5 B2U 5 U2B 5 DROP 0
[设备] PONG
[设备] ERR BAUD invalid
[设备] ERR POWER range -24..9
[设备] ERR POWER invalid
[设备] OK SELFTEST started
[设备] SELFTEST sent=33/33 got=33 PASS
```

> 说明：用「写入起始处的魔术字」做分流，无需对用户数据转义；只有当 DUT 数据
> **恰好**以 `ESC B L` 开头的那一个写入才会被误判，概率极低。

### 6.3 `SELFTEST` 自检详解

自检会把固定图案 `SELFTEST:TX-RX:0123456789ABCDEF\r\n`（33 字节）写到 UART TX，
再把环回回来的字节数计入 `U2B`，最后回报：

```
SELFTEST sent=<写入>/<期望> got=<收回> PASS|FAIL
```

- `sent`：`uart_write_bytes()` 实际写入的字节数（正常应为 33）。
- `got`：自检期间 `U2B` 的增量。
- `sent==33 且 got>=33` → `PASS`，说明 UART TX→(短接)→RX 环回通路正常；
  `got=0` → `FAIL`。

用它体检接线的步骤：**拔掉 DUT**，把桥接的 TX-RX（默认 GPIO4-GPIO5）短接，
在网页点「自检」。自检期间环回的图案也会照常以**普通串口数据**回显到网页。

---

## 7. 配置项（menuconfig）

路径：`idf.py menuconfig → BLE <-> UART 桥接配置`。

| Kconfig | 默认值 | 说明 |
|---|---|---|
| `BRIDGE_UART_NUM` | `1` | DUT 所用 UART 端口（0/1），默认避开 UART0 控制台 |
| `BRIDGE_UART_TX` | `4` | C3 → DUT 的 TX 引脚 |
| `BRIDGE_UART_RX` | `5` | DUT → C3 的 RX 引脚 |
| `BRIDGE_BAUD` | `115200` | 上电默认波特率，可网页动态改 |
| `BRIDGE_BUF_SIZE` | `4096` | BLE↔UART StreamBuffer 大小（字节） |
| `BRIDGE_RX_DEBUG_LOG` | `n` | 打开后每收到一段 UART 数据就打印一行日志，排查接收问题用 |
| `BRIDGE_TX_POWER_MAX_DBM` | `9` | 运行期 `POWER` 命令允许的最大发射功率（dBm） |

> `BRIDGE_RX_DEBUG_LOG` 属于除错开关，量产/日常使用可关掉以减少日志刷屏。

**BLE 发射功率**（不在上面这个菜单里）：`idf.py menuconfig →
Component config → Bluetooth → Controller → BLE default Tx power level`，
或直接改 `sdkconfig.defaults` 里的
`CONFIG_BT_CTRL_DFT_TX_POWER_LEVEL_*`。本工程 3.0V 供电，默认取 **+3dBm**
（`..._P3=y`），兼顾通信距离与电源裕量；运行时可用 `POWER=<dBm>` 命令再调，
该命令会把档位写入 NVS，**掉电重启后仍按上次设置生效**（无 NVS 记录时才用
上面这个编译期默认值）。

> 由于 3.0V 下高功率会导致电源跌落、连接监督超时掉线（`reason 0x208`），
> 运行期 `POWER` 受 `BRIDGE_TX_POWER_MAX_DBM`（默认 `+9`）上限约束；且上电时
> 若发现 NVS 里存了超过上限的旧值会自动钳位到上限。

---

## 8. 排障指南

### 8.1 先看 `STATUS` 的计数

```
STATUS BAUD <波特率> PWR <dBm> TX <脚> RX <脚> B2U <n> U2B <n> DROP <n>
```

| 字段 | 含义 | 判读 |
|---|---|---|
| `PWR` | 当前 BLE 发射功率(dBm) | 可用 `POWER=<dBm>` 动态调整 |
| `B2U` | 网页 → 设备 已转发字节数 | 发数据后应增长 |
| `U2B` | 设备 → 网页 已收到字节数 | 设备回数据后应增长 |
| `DROP` | BLE 写入过快被丢弃的字节数 | 持续增长说明写得太快，可降速或加大缓冲 |

### 8.2 「能发不能收」（`B2U` 涨、`U2B` 恒为 0）

先点「自检」区分是 **UART 接收** 还是 **BLE 通知** 的问题：

1. 短接桥接的 TX-RX，点「自检」。
2. `got>=33` → UART 收发链路正常，问题在 BLE/网页侧。
3. `got=0` → UART 接收方向没通，排查：
   - TX/RX 是否接反（桥接 TX 要接 DUT 的 RX，反之亦然）；
   - RX 是否落在 strapping 脚（**GPIO9 = BOOT**，实测不可靠）——改用 GPIO4/5；
   - 用万用表/示波器确认 RX 线上确有电平翻转；
   - 打开 `CONFIG_BRIDGE_RX_DEBUG_LOG`，看串口日志有没有 `UART RX n bytes`。

### 8.3 其它常见问题

| 现象 | 排查方向 |
|---|---|
| 上电后不断复位 | GPIO4/5 接线短路，或与 DUT 电平不匹配 |
| 网页搜不到设备 | 非安全上下文（需 https / localhost）；或非 Chrome/Edge |
| 连上但收不到 `READY` | TX 通知未订阅；确认网页已 `startNotifications()` |
| 波特率不对/乱码 | 网页点「应用」；注意复位后回默认 115200 |
| 大包发送丢字 | 查看 `DROP`，降低发送速率或增大 `BRIDGE_BUF_SIZE` |

---

## 9. 目录结构

```
ESP32C3_NUS_BLE/
├─ CMakeLists.txt          # EXTRA_COMPONENT_DIRS 指向官方 common/ble_uart
├─ sdkconfig.defaults      # 引脚、NimBLE、MTU 等默认配置
├─ main/
│  ├─ CMakeLists.txt
│  ├─ Kconfig.projbuild    # UART 端口/引脚/波特率/缓冲大小
│  └─ main.c               # 桥接逻辑 + 控制协议 + 自检
├─ web/
│  └─ index.html           # 单文件 WebBLE 客户端
└─ README.md
```

---

## 许可

本项目使用 **GNU General Public License v3.0**，详见 [LICENSE](LICENSE)。
