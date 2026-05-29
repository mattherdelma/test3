# UART 网关二进制通信协议

稳定、低延迟、自同步的帧协议,带 CRC-16 完整性校验与序列号 + ACK/NAK 错误恢复。
两端共用 `protocol/` 下的纯 C 实现。

## 物理层

- 默认 **115200 8N1**(可按线缆质量上调到 1 Mbps,Teensy/32U4 均支持)。
- 接线见 `docs/HARDWARE.md`。CP2102 提供 PC↔Teensy 链路;Teensy↔Leonardo 可用
  串口跳线直连(共地)。

## 帧格式

```
+------+------+------+------+------+------------------+--------+--------+
| SOF  | VER  | TYPE | SEQ  | LEN  |  PAYLOAD[LEN]    | CRC_HI | CRC_LO |
+------+------+------+------+------+------------------+--------+--------+
 0xAA   0x01   1B     1B     1B      0..255 bytes      CRC-16/CCITT-FALSE
```

- **SOF** = `0xAA` 帧头,用于重同步。
- **VER** = `0x01` 协议版本。
- **TYPE** 消息类型(见下表)。
- **SEQ** 发送方递增序列号,用于 ACK/NAK 配对。
- **LEN** 负载长度 0..255。
- **CRC** 对 `VER..PAYLOAD`(不含 SOF、不含 CRC 本身)计算的
  CRC-16/CCITT-FALSE(poly `0x1021`, init `0xFFFF`)。

## 消息类型

| 值 | 名称 | 方向 | 负载 |
|----|------|------|------|
| 0x01 | PING | 双向 | 任意(回显用) |
| 0x02 | PONG | 双向 | 回显 PING 负载 |
| 0x03 | ACK | 双向 | `[seq]` 被确认的序列号 |
| 0x04 | NAK | 双向 | `[seq][reason]` |
| 0x05 | STATUS | 设备→PC | 设备状态 |
| 0x10 | BUTTON_EVENT | 设备→PC | `[button_mask][state]` 无障碍按钮事件 |
| 0x20 | KEY_SEQUENCE | PC→设备 | 见下 |
| 0x21 | MOUSE_TEST | PC→设备 | `[dxLo dxHi dyLo dyHi wheel buttons]` 测试夹具鼠标动作 |
| 0x7E | RESET | 双向 | 复位序列号 / 中止当前序列 |

### KEY_SEQUENCE 负载

```
[count] 后接 count 条 5 字节记录:
  byte0 action   : 1=PRESS 2=RELEASE 3=TAP 4=RELEASE_ALL
  byte1 keycode  : HID usage / Arduino Keyboard 键值
  byte2 modifier : 位图 LCTRL=1 LSHIFT=2 LALT=4 LGUI=8 RCTRL=16 ...
  byte3..4 delay_before_us : 执行本记录前等待的微秒数(小端)
```

单帧最多 50 条记录(`(255-1)/5`)。更长的序列拆成多帧,逐帧 ACK 后发下一帧。

## 错误恢复

1. **重同步**:解析器在任意状态遇到坏数据时回到扫描 SOF;CRC 兜底防止把噪声
   当成有效帧。
2. **CRC 失败**:接收方回 `NAK(reason=CRC)`,发送方重发。
3. **重传策略(PC 端建议)**:发送后等 ACK,超时(建议 50 ms)未到则重发,
   指数退避,最多 4 次;仍失败则上报链路错误。
4. **去重**:接收方可记录上次成功处理的 SEQ,忽略重复(重传导致)的相同 SEQ。
5. **失败安全**:键盘网关在任何序列出错或结束时调用 `releaseAll()`,绝不留下
   卡住的按键;鼠标网关出错时清空按钮状态。

## 延迟预算

- 串行传输:115200 下每字节约 87 us;一条 RELEASE_ALL 帧(7 字节)约 0.6 ms。
- 真正的下限是 **USB HID 轮询间隔**(标准全速设备 1 ms)。时序引擎按微秒调度
  状态翻转,但主机仍按 bInterval 采样端点 —— 见 `arduino_keyboard_gateway/
  timing_engine.h` 的说明,我们不假装能突破这个上限。
