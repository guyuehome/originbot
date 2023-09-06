//
// Copyright (c) 2021 ECOVACS
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
//

#include "drivers/protocol.h"

#include <iostream>

#pragma pack(1)
// 不含能量信息的报文结构
struct MiniPacket {
    uint8_t header;
    uint8_t angle;
    uint16_t speed;
    uint16_t data[16];
    uint32_t stamp;
    uint16_t checksum;
};
#pragma pack()

bool Packet::IsHeader(uint8_t header)
{
    return header == 0xFA;
}

uint32_t Packet::length() { return sizeof(MiniPacket); }

// 该函数用于对数据包有效性校验(同时支持含能量和不含能量信息两种报文的解析)
bool Packet::IsValid(const Packet *packet, bool &energetic)
{
    // checksum angle
    if (packet->angle < 0xA0 || packet->angle > 0xF9)
        return false;

    // checksum minimum packet
    const MiniPacket *minipacket = reinterpret_cast<const MiniPacket *>(packet);
    const uint8_t *data = reinterpret_cast<const uint8_t *>(minipacket);

    uint32_t sum = 0;
    for (uint32_t i = 0; i < sizeof(*minipacket) - sizeof(packet->checksum); i++)
        sum += *(data + i);

    if ((sum == minipacket->checksum) && Packet::IsHeader((minipacket + 1)->header)) {
        energetic = false;
        return true;
    }

    for (uint32_t i = sizeof(*minipacket) - sizeof(minipacket->checksum); i < sizeof(*packet) - sizeof(packet->checksum); i++)
        sum += *(data + i);

    if (sum == packet->checksum) {
        energetic = true;
        return true;
    }

    return false;
}