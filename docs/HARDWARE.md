# 硬件架构与接线

```
                 真实鼠标                         真实键盘(可选)
                    │ USB                              │ USB
                    ▼                                  ▼
        ┌───────────────────────┐          ┌───────────────────────┐
        │   Teensy 4.1          │          │  Arduino Leonardo      │
        │   USB Host 口 读取鼠标 │          │  USB Host Shield 读键盘 │
        │                       │          │  (ENABLE_USB_HOST)     │
        │   全透传 → PC          │          │                        │
        │   侧键事件 → UART      │          │   注入组合键 → PC        │
        └─────┬──────────┬──────┘          └──────────┬─────────────┘
              │ USB Device│ Serial1                    │ USB Device(HID 键盘)
              ▼           │ (UART)                     ▼
            主控 PC ◄─────┼────── CP2102 ──────────► 主控 PC
                          │                           ▲
                          └──── 串口跳线(可选)────────┘
                               Teensy → Leonardo
```

## 关键点(合规设计)

- Teensy 的鼠标输出是**真实鼠标的完整透传**,侧键既透传给 PC、又额外发一条 UART
  事件给 PC 做无障碍动作 —— 按钮对 PC 可见,不是隐藏触发。
- 键盘注入设备以**独立、可识别的测试/无障碍 HID** 身份工作,不与真人输入混流。
- 两块板都用**自身合法 VID/PID**,不冒充他厂(见 `USB_DESCRIPTORS.md`)。

## 接线表

### Teensy 4.1 ↔ CP2102(到 PC 的 UART)

| Teensy | CP2102 |
|--------|--------|
| Pin 1 (TX1) | RXD |
| Pin 0 (RX1) | TXD |
| GND | GND |

注意 Teensy 4.1 是 3.3V IO —— 选用 **3.3V 版 CP2102 模块**,勿用 5V 逻辑直连。

### Teensy 4.1 USB Host(读真实鼠标)

Teensy 4.1 板载 5-pin USB Host 焊盘。焊上 USB-A 母座(D+/D-/5V/GND),接真实鼠标。

### Teensy ↔ Leonardo 串口跳线(可选,Teensy 直驱键盘网关)

| Teensy | Leonardo |
|--------|----------|
| Pin 1 (TX1) | Pin 0 (RX1) |
| Pin 0 (RX1) | Pin 1 (TX1) |
| GND | GND |

> Leonardo 是 5V IO,Teensy 是 3.3V。Teensy 的 RX 接 Leonardo 的 TX 时需经
> 电平转换(分压或专用电平转换芯片),避免 5V 打到 3.3V 引脚。

### Arduino Leonardo + USB Host Shield(可选,读真实键盘)

USB Host Shield 直接叠在 Leonardo 上(SPI)。默认 `ENABLE_USB_HOST 0`,未接屏蔽
时保持关闭即可。

## 物料

- Teensy 4.1 ×1,USB-A 母座(host)×1
- Arduino Leonardo ×1(可选 + USB Host Shield)
- CP2102 USB-UART 模块(3.3V)×1
- 杜邦线、电平转换模块(Teensy↔Leonardo 链路需要)
