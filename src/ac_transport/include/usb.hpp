#ifndef USB_HPP
#define USB_HPP

#include "sdk/include/usbcdc_transporter.hpp"

#define RECEIVE_ID 0x1
#define SEND_ID 0x2

namespace ac_transport
{
typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef signed short    int16_t;
typedef unsigned short  uint16_t;
typedef signed int      int32_t;
typedef unsigned int    uint32_t;

#pragma pack(push, 1)

typedef struct 
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 数据
    int32_t mode;
    // 包尾
    uint8_t _EOF;
}ReceivePackage;

typedef struct 
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 数据
    float x;
    float y;
    // 包尾
    uint8_t _EOF;
}SendPackage;

#pragma pack(pop)

}

#endif // USB_HPP