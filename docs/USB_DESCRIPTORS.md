# USB 描述符配置指南(合规版)

本项目的设备以**自身真实身份**呈现为标准合规 HID 设备,不冒用任何第三方厂商
(罗技 / 微软等)的 VID/PID 或描述符。下面说明如何在两块板子上做到"任何系统、
旧 BIOS 即插即用"的通用兼容性 —— 这一点完全不需要伪装成别家产品。

## 为什么不用厂商伪装

操作系统和 BIOS 的即插即用兼容性来自:

1. 正确的 **设备类代码**(HID, `bInterfaceClass = 0x03`);
2. 标准的 **HID Report Descriptor**(boot mouse / boot keyboard 协议);
3. 合规的 **bcdUSB / 端点 / bInterval** 配置。

满足以上三点,用你**自己的** VID/PID 一样会被 Windows / macOS / Linux 以及
传统 BIOS 识别为标准 HID。复制特定厂商的 VID/PID 不会带来任何额外的兼容性,
只会带来"冒充该厂商身份"这一个效果,因此本项目不这么做。

## 获取合法的 VID/PID

按合规程度从高到低,任选其一:

| 方式 | 说明 |
|------|------|
| 自有 USB-IF Vendor ID | 向 usb.org 申请(适合产品化)。 |
| 复用开发板默认 VID/PID | Teensy / Arduino 出厂即带 PJRC / Arduino LLC 的合法分配,直接用即可。 |
| pid.codes 开源 PID | 面向开源硬件的免费 PID 分配(`0x1209`),适合个人/开源项目。 |

> 不要使用未分配给你的厂商 ID,也不要复制商用外设的 VID/PID。

## Teensy 4.1(鼠标网关)

最简单、最合规的做法:在 Arduino IDE 中选择
**Tools → USB Type → "Mouse"**(或 "Serial + Keyboard + Mouse + Joystick")。
Teensyduino 会用 PJRC 合法分配的 VID/PID 和标准 HID 鼠标描述符,开箱即合规。

如需自定义产品字符串 / PID(仍用你自己的标识),编辑 Teensyduino 核心中的
`usb_desc.h` / `usb_names.h`:

```c
/* usb_desc.h —— 仅修改为你自己的标识,切勿填入他厂 VID/PID */
#define VENDOR_ID        0x16C0   /* 例:PJRC 默认,或你自有/ pid.codes 分配 */
#define PRODUCT_ID       0x0489
#define MANUFACTURER_NAME {'M','y','L','a','b'}        /* 你的名字 */
#define PRODUCT_NAME      {'A','1','1','y',' ','M','o','u','s','e'}
```

标准 boot mouse report descriptor(Teensy 核心已自带,此处仅供参考):

```c
static const uint8_t mouse_report_desc[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Buttons)
  0x19, 0x01, 0x29, 0x05,  //   Usage Min..Max (1..5 buttons)
  0x15, 0x00, 0x25, 0x01,  //   Logical 0..1
  0x95, 0x05, 0x75, 0x01,  //   5 bits
  0x81, 0x02,        //     Input (Data,Var,Abs)
  0x95, 0x01, 0x75, 0x03,  //   3-bit padding
  0x81, 0x03,        //     Input (Const)
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x30, 0x09, 0x31, 0x09, 0x38,  // X, Y, Wheel
  0x15, 0x81, 0x25, 0x7F,  //   Logical -127..127
  0x75, 0x08, 0x95, 0x03,  //   3 bytes
  0x81, 0x06,        //     Input (Data,Var,Rel)
  0xC0, 0xC0
};
```

## Arduino Leonardo(键盘网关)

Leonardo 的 `Keyboard` 库默认就用 **Arduino LLC 合法分配的 VID/PID** 和标准
boot keyboard 描述符,直接即合规。若要自定义为你自己的标识,修改
`boards.txt` 中对应条目:

```
leonardo.build.vid=0x2341      # Arduino LLC(默认),或换成你自有/ pid.codes
leonardo.build.pid=0x8036
leonardo.build.usb_product="A11y Keyboard"
leonardo.build.usb_manufacturer="MyLab"
```

## 验收

- Windows:设备管理器中显示为 "HID Keyboard/Mouse Device",厂商为你填写的名称。
- Linux:`lsusb` 显示你自己的 VID:PID 与产品字符串;`usbhid` 正常绑定。
- BIOS:进入主板 Setup,键盘/鼠标可用(boot protocol 生效)。
