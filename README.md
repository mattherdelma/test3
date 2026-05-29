# 无障碍辅助输入网关与自动化测试硬件(合规版)

Teensy 4.1(鼠标网关)+ Arduino Leonardo(键盘网关)+ CP2102(UART)构成的
无障碍辅助输入与自动化 UI 测试硬件。

## 设计原则(合规边界)

本项目实现以下能力,且**仅**实现以下能力:

- **鼠标全透传**:真实鼠标的所有动作/按键完整转发给 PC,不隐藏、不改写、不丢弃。
- **无障碍按钮事件**:侧键在透传的同时额外发一条 UART 事件,供 PC 触发无障碍动作
  —— 按钮对 PC 可见,不是隐藏触发。
- **自动化测试注入**:以**独立、可识别**的测试/无障碍 HID 身份注入按键/鼠标动作,
  供无人值守 UI 测试与无障碍宏使用;**不**与真人实时输入混流伪装来源。
- **自身合法 USB 身份**:用开发板自带或自有/开源分配的 VID/PID,**不**冒充罗技/
  微软等任何第三方厂商。

> 刻意未实现:把外部主机算出的移动量与真人手部输入实时叠加并隐藏触发来源、以及
> 冒用商用外设 VID/PID 伪装设备指纹。这两类设计是硬件自瞄/反检测器的核心特征,
> 与无障碍和自动化测试目标无关,故不提供。

## 目录结构

```
protocol/                   两端共用的纯 C UART 协议 + CRC
  crc.h / crc.c
  gateway_protocol.h / .c
teensy_mouse_gateway/
  teensy_mouse_gateway.ino  Teensy 4.1:全透传 + 侧键事件 + 测试注入
arduino_keyboard_gateway/
  timing_engine.h           微秒级时序执行引擎
  arduino_keyboard_gateway.ino  Leonardo:接收脚本并注入组合键
host/
  gateway_host.py           PC 端参考实现(发送序列/读事件)
docs/
  USB_DESCRIPTORS.md        合规 VID/PID 与描述符配置指南
  PROTOCOL.md               UART 二进制协议规格
  HARDWARE.md               硬件架构与接线
```

## 快速开始

1. **键盘网关**:Arduino IDE 打开 `arduino_keyboard_gateway/`,选 Leonardo,上传。
2. **鼠标网关**:安装 Teensyduino,设 `USB Type = Mouse`,打开
   `teensy_mouse_gateway/`,选 Teensy 4.1,上传。
3. **接线**:见 `docs/HARDWARE.md`(注意 Teensy 3.3V 与 Leonardo 5V 间需电平转换)。
4. **PC 驱动**:`pip install pyserial`,然后
   `python3 host/gateway_host.py --port <串口> --demo`。

## 协议自检

`host/gateway_host.py` 的 CRC 与帧格式同 `protocol/` 保持一致,可用作回归基准。
