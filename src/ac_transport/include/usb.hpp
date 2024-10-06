#ifndef USB_HPP
#define USB_HPP

#include "sdk/include/usbcdc_transporter.hpp"

#define SEND_ID 0x2
#define RECEIVE_ID 0x1

namespace ac_transport
{
typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef signed short    int16_t;
typedef unsigned short  uint16_t;
typedef signed int      int32_t;
typedef unsigned int    uint32_t;

#pragma pack(push, 1)

enum class ObjectType
{
    RUBIKCUBE,  // 魔方 - 0
    BILLIARDS,  // 台球 - 1
};

typedef struct  // 接收包
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 数据
    uint8_t mode;
    // 包尾
    uint8_t _EOF;
}ReceivePackage;

typedef struct  // 发送包
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 数据
    uint8_t x[4];
    uint8_t y[4];
    // 包尾
    uint8_t _EOF;
}SendPackage;

#pragma pack(pop)

}

#endif // USB_HPP