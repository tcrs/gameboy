#pragma once
/* Copyright (C) 2014-2018 Thomas Spurden <thomas@spurden.name>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NDEBUG
struct gameboy_tp {
    enum {
        GAMEBOY_TP_MEM_READ,
        GAMEBOY_TP_MEM_WRITE,
        GAMEBOY_TP_INSTR_START,
        GAMEBOY_TP_DMA_INIT,
        GAMEBOY_TP_DMA_START,
        GAMEBOY_TP_DMA,
    } point;
    union {
        struct {
            uint16_t addr;
        } mem_read;
        struct {
            uint16_t addr;
            uint8_t data;
        } mem_write;
        struct {
            uint8_t opcode;
        } instr_start;
        struct {
            uint16_t src;
        } dma;
    } u;
};
#endif

enum {
    Interrupt_VBlank = 0x01,
    Interrupt_LCDC = 0x02,
    Interrupt_TIMA = 0x04,
    Interrupt_Serial = 0x08,
    Interrupt_Joypad = 0x10,

    Interrupt_Mask = 0x1F,
};
enum {
    Cart_MaxROMSize = 4 * 1024 * 1024,
    Cart_MaxRAMSize = 32 * 1024,

    Cart_MBC_None = 0,

    /* MBC1 can operate in two modes, switchable at runtime */
    Cart_MBC1_16_8,
    Cart_MBC1_4_32,

    Cart_MBC2,
    Cart_MBC3,
    Cart_MBC5,

    Cart_MBC3_RTCBase = 0x08,
    Cart_MBC3_RTCLast = 0x0C,
};
enum {
    Button_Down = 0x80,
    Button_Up = 0x40,
    Button_Left = 0x20,
    Button_Right = 0x10,

    Button_Start = 0x08,
    Button_Select = 0x04,
    Button_B = 0x02,
    Button_A = 0x01
};

struct Gameboy
{
#ifndef NDEBUG
    uint32_t (*trace_fn)(struct Gameboy* gb, struct gameboy_tp* tp);
#endif
    struct GameboyCPU {
        uint8_t A, F;
        uint8_t B, C;
        uint8_t D, E;
        uint8_t H, L;
        uint16_t SP;
        uint16_t PC;
        bool InterruptsEnabled;
        bool InterruptEnablePending;
        bool Halted;
        bool HaltBug;
    } cpu;
    uint64_t TotalCycles;
    struct {
        uint16_t CycleCount;
        bool TimerOverflow;
        bool TimerLoading;
    } clock;
    struct {
        char Title[16];
        bool HasRTC;
        bool HasBattery;
        bool HasRumble;
    } info;
    struct {
        /* 0x8000 - 0x9FFF */
        uint8_t VideoRAM[8192];
        /* 0xC000 - 0xDFFF */
        uint8_t WorkRAM[8192];
        /* 0xFE00 - 0xFE9F */
        uint8_t OAM[160];
        /* 0xFF00 - 0xFF7F */
        uint8_t IO[128];
        /* 0xFF80 - 0xFFFE */
        uint8_t HighRAM[127];
        /* 0xFFFF */
        uint8_t InterruptEnable;
    
        /* The cartridge ROM & RAM is typically banked into the main address
         * space using a MBC chip */
        unsigned int CartROMSize;
        uint8_t CartROM[Cart_MaxROMSize];
    
        unsigned int CartRAMSize;
        uint8_t CartRAM[Cart_MaxRAMSize];
    
        /* If true then addresses 00-FF contain the boot ROM */
        bool BootROMEnabled;
    
        /* Cartridge RAM should be enabled before writing to it, and disabled when finished */
        bool CartRAMBankEnabled;
    
        /* Model and state of the MBC chip */
        int MBCModel;
        unsigned int MBCROMBank;
        unsigned int MBCRAMBank;
    } mem;
    struct {
        time_t BaseTime;
    
        /* seconds, minutes, hours, days, dayhi */
        uint8_t BaseReg[5];
        uint8_t LatchedReg[5];
        bool Latched;
    } rtc;
    struct {
        bool DelayStart;
        uint8_t PendingSource;
    
        /* Source for currently running DMA (0 if not active) */
        uint16_t Source;
    
        /* If OAM DMA is active on current cycle */
        bool Active;
    } dma;
    struct {
        /* Machine cycles through current frame */
        unsigned int FrameProgress;
    
        /* Bits 0-1 = colour (0 = darkest, 3 = lightest)
         * Other bits currently all zero.
         */
        uint8_t Buffer[160][144];
        /* Set to true whenever a complete new frame is available */
        /* Reset this when you read the frame */
        bool NewFrame;
    
        struct GameboySprite {
            uint8_t x;
            uint8_t pixels[2];
            uint8_t attrs;
        } ScanlineSprites[10];
        unsigned int NumSprites;
    
        unsigned int CurX;
    } lcd;
    struct {
        uint8_t Pressed;
    } buttons;
};

char const* gameboy_load(struct Gameboy*);
int gameboy_reset(struct Gameboy*, bool enableBootROM);
int gameboy_step(struct Gameboy*);
uint8_t gameboy_read(struct Gameboy* gb, uint16_t addr);
void gameboy_write(struct Gameboy* gb, uint16_t addr, uint8_t value);
void gameboy_setButtonState(struct Gameboy*, int button, bool down);

#ifdef __cplusplus
}
#endif
