/* Copyright 2023 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#ifndef MRI_PLATFORM_H_
#define MRI_PLATFORM_H_

#include <stdint.h>
#include <memory.h>
extern "C"
{
    #include <core/platforms.h>
}

// The pMemory array passed into initDebugger() should be this size.
#define DEBUGGER_MEMORY_SIZE (64 * 1024)

// If a breakpoint contains this special 32-bit address then it is free.
#define FREE_BREAKPOINT 0xFFFFFFFF

// If a watchpoint has this special 32-bit address in its startAddress field then it is free.
#define FREE_WATCHPOINT 0xFFFFFFFF

// Enumeration of Z80 registers found in Z80Registers::regs[].
enum Z80RegisterIndex
{
    AF = 0,
    BC = 1,
    DE = 2,
    HL = 3,
    SP = 4,
    PC = 5,
    IX = 6,
    IY = 7,
    _AF = 8,
    _BC = 9,
    _DE = 10,
    _HL = 11,
    IR = 12
};

// Z80 Context
union Z80Registers
{
    struct
    {
        uint16_t af;
        uint16_t bc;
        uint16_t de;
        uint16_t hl;
        uint16_t sp;
        uint16_t pc;
        uint16_t ix;
        uint16_t iy;
        uint16_t _af;
        uint16_t _bc;
        uint16_t _de;
        uint16_t _hl;
        uint16_t ir;
    };
    uint16_t regs[13];
};

// Watchpoint entry. An array of these is passed into initDebugger().
struct Z80Watchpoint
{
    // Start of address range for this watchpoint.
    // This address is inclusive.
    // If set to FREE_WATCHPOINT then this entry is free.
    uint32_t               startAddress;
    // End of address range for this watchpoint.
    // This address is EXCLUSIVE so the range actually ends at the byte before this address.
    // This field is 32-bit so that 0x10000 can be safely used to use the end of RAM as the end of a range.
    uint32_t               endAddress;
    // Is this watchpoint for read, writes, or both type of accesses.
    PlatformWatchpointType type;
};


bool initDebugger(uint8_t* pMemory, size_t memorySize,
                  uint32_t* pBreakpoints, size_t breakpointCount,
                  Z80Watchpoint* pWatchpoints, size_t watchpointCount);
void enterDebugger(Z80Registers* pRegisters, uint8_t signal, PlatformTrapReason* pReason);

#endif // MRI_PLATFORM_H_
