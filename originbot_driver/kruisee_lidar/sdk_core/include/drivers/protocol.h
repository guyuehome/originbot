//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#pragma once

#include <stdint.h>

#pragma pack(1)

// 含能量信息的报文结构

/*
struct Packet {
    uint8_t header;
    uint8_t angle;
    uint16_t speed;
    uint16_t data[16];
    uint8_t power[16];
    uint32_t stamp;
    uint16_t checksum;

    static bool IsHeader(uint8_t header);
    static bool IsValid(const Packet *packet, bool &energetic);
    static uint32_t length();
};
*/


struct Packet {
    uint8_t header;
    uint8_t angle;
    uint16_t speed;
    uint16_t data[16];
    uint32_t stamp;
    uint8_t power[16];
    uint32_t reserved;
    uint16_t checksum;

    static bool IsHeader(uint8_t header);
    static bool IsValid(const Packet *packet, bool &energetic);
    static uint32_t length();
};

#pragma pack()