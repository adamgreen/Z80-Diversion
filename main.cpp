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
// Firmware for my Z80 Diversion.
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "Z80Bus.h"
#include "mri_platform.h"

// MRI C headers
extern "C"
{
    #include <core/platforms.h>
}


// Z80 bus pin definitions.
static const uint32_t Z80_MREQ_PIN = 2;
static const uint32_t Z80_IOREQ_PIN = 3;
static const uint32_t Z80_CLK_PIN = 4;
static const uint32_t Z80_RD_PIN = 7;
static const uint32_t Z80_WR_PIN = 8;
static const uint32_t Z80_D0_PIN = 9;
static const uint32_t Z80_D1_PIN = 10;
static const uint32_t Z80_D2_PIN = 11;
static const uint32_t Z80_D3_PIN = 12;
static const uint32_t Z80_D4_PIN = 13;
static const uint32_t Z80_D5_PIN = 14;
static const uint32_t Z80_D6_PIN = 15;
static const uint32_t Z80_D7_PIN = 16;
static const uint32_t Z80_M1_PIN = 17;
static const uint32_t Z80_WAIT_PIN = 18;
static const uint32_t Z80_RFSH_PIN = 19;
static const uint32_t Z80_HALT_PIN = 20;
static const uint32_t Z80_NMI_PIN = 21;
static const uint32_t Z80_INT_PIN = 22;
static const uint32_t Z80_RESET_PIN = 26;

// Shift register pin definitions.
static const uint32_t SHIFT_EVEN_PIN = 0;
static const uint32_t SHIFT_ODD_PIN = 1;
static const uint32_t SHIFT_CLOCK_PIN = 5;
static const uint32_t SHIFT_LATCH_PIN = 6;

// The frequency at which the Z80 should be clocked.
static const uint32_t Z80_FREQUENCY = 500000;

// The number of hardware breakpoints supported by this firmware.
static const size_t   Z80_HARDWARE_BREAKPOINTS = 4;

// The number of hardware watchpoints supported by this firmware.
static const size_t   Z80_HARDWARE_WATCHPOINTS = 2;


// Pick which code to run.
static const enum
{
    RUN_DEBUGGER,
    RUN_COUNT_TEST,
    RUN_NOP_TEST
} g_runMode = RUN_DEBUGGER;

// Global for Z80 Bus object.
static Z80Bus g_z80Bus;


// Function Prototypes
static void testIncrementingZ80Code(Z80Bus& z80Bus);
static void testNOPs(Z80Bus& z80Bus);
static void runDebugger();
static void core1Z80Runner();


int main()
{
    stdio_init_all();
    while (!stdio_usb_connected())
    {
      sleep_ms(250);
    }

    // Configure the PIO state machines to manage the Z80 bus and let the RP2040 core know when memory or I/O requests
    // have been made by the Z80 so that the RP2040 core can handle them.
    bool result = g_z80Bus.init(Z80_FREQUENCY, Z80_RESET_PIN, Z80_CLK_PIN,
                                Z80_MREQ_PIN, Z80_IOREQ_PIN, Z80_RD_PIN, Z80_WR_PIN,
                                Z80_D0_PIN, Z80_D1_PIN, Z80_D2_PIN, Z80_D3_PIN,
                                Z80_D4_PIN, Z80_D5_PIN, Z80_D6_PIN, Z80_D7_PIN,
                                Z80_M1_PIN, Z80_WAIT_PIN, Z80_RFSH_PIN, Z80_HALT_PIN,
                                SHIFT_CLOCK_PIN, SHIFT_LATCH_PIN, SHIFT_EVEN_PIN, SHIFT_ODD_PIN);
    assert ( result );
    (void)result;

    switch (g_runMode)
    {
        case RUN_COUNT_TEST:
            testIncrementingZ80Code(g_z80Bus);
            break;
        case RUN_NOP_TEST:
            testNOPs(g_z80Bus);
            break;
        case RUN_DEBUGGER:
        default:
            runDebugger();
            break;
    }

    return 0;
}

static void testIncrementingZ80Code(Z80Bus& z80Bus)
{
    // Memory to be used by Z80.
    uint8_t memory[] =
    {
        // Offset 0x0:
        // LD HL, (nn)
        0x2A, 0x0A, 0x00,

        // Offset 0x3:
        // INC HL - 00 xx0 011 -  00 100 011 - 0x23
        0x23,

        // Offset 0x4:
        // LD (nn), HL
        0x22, 0x0A, 0x00,

        // Offset 0x7:
        // JP 0x0000
        0xC3, 0x00, 0x00,

        // Offset 0xA:
        // Value to increment.
        0x00, 0x00
    };
    const size_t counterOffset = sizeof(memory) - 2;

    // Trace log of what should be expected for each Z80 read/write request when running the above program.
    struct Trace
    {
        uint16_t expectedAddress;
        bool expectedM1;
        bool expectedWrite;
        bool checkCounter;
    } trace[] =
    {
        // Address, M1,     WR,     Check Counter
        { 0x0000,   true,   false,  false },
        { 0x0001,   false,  false,  false },
        { 0x0002,   false,  false,  false },
        { 0x000A,   false,  false,  false },
        { 0x000B,   false,  false,  false },
        { 0x0003,   true,   false,  false },
        { 0x0004,   true,   false,  false },
        { 0x0005,   false,  false,  false },
        { 0x0006,   false,  false,  false },
        { 0x000A,   false,  true,   false },
        { 0x000B,   false,  true,   true },
        { 0x0007,   true,   false,  false },
        { 0x0008,   false,  false,  false },
        { 0x0009,   false,  false,  false }
    };

    volatile uint16_t expectedCount = 0x0000;
    volatile size_t traceIndex = 0;
    while (true)
    {
        // Fetch next read/write transfer request.
        volatile struct Z80Bus::Request req;
        z80Bus.getNextRequest((Z80Bus::Request*)&req);
        assert ( req.address < sizeof(memory) );
        assert ( req.isMReq );

        // Verify that this request matches the expected trace.
        volatile Trace* pTrace = &trace[traceIndex];
        assert ( pTrace->expectedAddress == req.address );
        assert ( pTrace->expectedM1 == req.isM1 );
        assert ( pTrace->expectedWrite == !req.isRead );

        // Perform the memory read or write from/to RAM and send response to Z80 via PIO.
        bool isRead = req.isRead;
        uint16_t address = req.address;
        if (isRead)
        {
            req.value = memory[address];
            if (Z80_FREQUENCY <= 1000)
            {
                printf("[%04X]->%02X %s\r\n", req.address, req.value, req.isM1 ? "M1" : "");
            }
        }
        else
        {
            memory[address] = req.value;
            if (Z80_FREQUENCY <= 1000)
            {
                printf("[%04X]<-%02X\r\n", req.address, req.value);
            }
        }
        z80Bus.completeRequest((Z80Bus::Request*)&req);

        // Check the 16-bit counter right after it has been written.
        if (pTrace->checkCounter)
        {
            uint16_t counter;
            expectedCount++;
            memcpy(&counter, &memory[counterOffset], sizeof(counter));
            assert ( counter == expectedCount );
        }
        traceIndex = (traceIndex + 1) % (sizeof(trace)/sizeof(trace[0]));
    }
}

static void testNOPs(Z80Bus& z80Bus)
{
    volatile uint16_t expectedAddress = 0x0000;
    while (true)
    {
        // Fetch next read/write transfer request.
        Z80Bus::Request req;
        z80Bus.getNextRequest(&req);

        // Every request should be a read from an ascending address.
        assert ( req.isMReq );
        assert ( req.isRead );
        assert ( req.address == expectedAddress++ );
        (void)expectedAddress;

        uint8_t NOP = 0x00;
        req.value = NOP;
        z80Bus.completeRequest(&req);

        // Display some output every once in awhile to let the user know it is still running.
        if ((req.address & 0x00FF) == 0x0000)
        {
            printf("%04X\r\n", req.address);
        }
    }
}




// Commands that can be sent to the Z80 control thread.
enum Z80Commands
{
    // Just hold Z80 in WAIT state.
    COMMAND_PAUSE,
    // Let the Z80 run normally.
    COMMAND_RUN,
    // Use "RST 8" to halt Z80 and grab context.
    COMMAND_HALT,
    // Resume after Z80 has been halted. Only sent when control thread is in STATE_HALTED state.
    COMMAND_HALT_RESUME,
    // Resume after Z80 has been halted and reset. Skip register popping. Only sent when control thread is in
    // STATE_HALTED state.
    COMMAND_HALT_RESET_RESUME,
    // Let one M1 through and then enter halt mode like COMMAND_HALT.
    COMMAND_SINGLE_STEP
};

// States that the Z80 control thread can be in.
enum Z80States
{
    // Most recent Z80 memory request is left in WAIT state. Stay in this state as long as g_command is COMMAND_PAUSE.
    STATE_PAUSED,
    // Z80 is allowed to run normally from the RAM contents in g_memory.
    STATE_RUNNING,
    // Start single stepping mode. Insert a "RST 8" after one M1' cycle has been received.
    STATE_SINGLE_STEP_START,
    // Wait for next instruction fetch and return "RST 8" in its place.
    STATE_HALT_START,
    // Waiting for 1st RST 8 vector instruction to be fetched. Also records the 2 most recent memory writes as they will
    // be the automatic push of the PC to the stack pointed to by SP. This is where the value of SP is discovered.
    STATE_HALT_WAIT_RST8_VECTOR,
    // Z80 is now given code which pushes all of the registers to the stack.
    STATE_HALT_PUSHING,
    // All registers have been pushed to the stack and then copied over to g_registers.
    // Waits for COMMAND_HALT_RESUME in g_command before continuing to next state.
    STATE_HALTED,
    // The contents of g_registers (may have been modified by GDB) are copied back onto the stack.
    // The Z80 is then given code which pops all of the registers from the stack.
    STATE_HALT_POPPING,
    // The Z80 has returned to start executing at PC but is held in WAIT state until g_command is set to something other
    // than COMMAND_HALT_RESUME.
    STATE_HALT_DONE
};

// Globals used for communication between Core0 and Core1 for Z80 control.
static volatile Z80Commands g_command = COMMAND_PAUSE;
static volatile Z80States   g_state = STATE_PAUSED;

// 64k of RAM used for satisfying Z80 memory requests.
static uint8_t g_memory[DEBUGGER_MEMORY_SIZE];

// Z80 register contents when it is halted.
static Z80Registers g_registers;

// Array of hardware breakpoints.
static uint32_t g_breakpoints[Z80_HARDWARE_BREAKPOINTS];

// Array of hardware watchpoints.
static Z80Watchpoint g_watchpoints[Z80_HARDWARE_WATCHPOINTS];

// Reason for debug trap.
PlatformTrapReason g_reason;


static void handlePauseState(Z80Bus::Request* pReq);
static void handleRunningState(Z80Bus::Request* pReq);
static bool doesMatchWatchpoint(uint32_t address, PlatformWatchpointType type);
static void handleSingleStepStartState(Z80Bus::Request* pReq);
static void handleHaltStartState(Z80Bus::Request* pReq);
static void handleHaltWaitRst8State(Z80Bus::Request* pReq);
static void handleHaltPushingState(Z80Bus::Request* pReq);
static uint16_t readHalfWord(uint16_t address);
static void handleHaltedState(Z80Bus::Request* pReq);
static void writeHalfWord(uint16_t address, uint16_t value);
static void handleHaltPoppingState(Z80Bus::Request* pReq);
static void handleHaltDoneState(Z80Bus::Request* pReq);

// UNDONE: Temporary.
static const uint8_t g_testProgram[] =
{
    // LD SP, 0x0000
    0x31, 0x00, 0x00,
    // LD A, 0x14
    0x3E, 0x14,
    // LD I, A
    0xED, 0x47,
    // LD A, 0x15
    0x3E, 0x15,
    // LD R, A
    0xED, 0x4F,
    // LD BC, 0x0001
    0x01, 0x01, 0x00,
    // PUSH BC
    0xC5,
    // POP AF
    0xF1,
    // LD BC, 0x0203
    0x01, 0x03, 0x02,
    // LD DE, 0x0405
    0x11, 0x05, 0x04,
    // LD HL, 0x0607
    0x21, 0x07, 0x06,
    // LD IX, 0x0809
    0xDD, 0x21, 0x09, 0x08,
    // LD IY, 0x0A0B
    0xFD, 0x21, 0x0B, 0x0A,
    // EXX
    0xD9,
    // EX AF, AF'
    0x08,
    // LD BC, 0x0C0D
    0x01, 0x0D, 0x0C,
    // PUSH BC
    0xC5,
    // POP AF
    0xF1,
    // LD BC, 0x0E0F
    0x01, 0x0F, 0x0E,
    // LD DE, 0x1011
    0x11, 0x11, 0x10,
    // LD HL, 0x1213
    0x21, 0x13, 0x12,
    // EX AF, AF'
    0x08,
    // EXX
    0xD9,
    // JR 0
    0x18, (uint8_t)-2
};

static void runDebugger()
{
    // Fill the Z80 RAM with NOPs (0x00).
    memset(g_memory, 0, sizeof(g_memory));

    // UNDONE: Temp test code.
    memcpy(g_memory, g_testProgram, sizeof(g_testProgram));

    // Initialize the MRI core.
    initDebugger(g_memory, sizeof(g_memory),
                 g_breakpoints, sizeof(g_breakpoints)/sizeof(g_breakpoints[0]),
                 g_watchpoints, sizeof(g_watchpoints)/sizeof(g_watchpoints[0]));

    // Initialize the globals used to communicate between the 2 cores.
    g_command = COMMAND_PAUSE;
    g_state = STATE_PAUSED;

    // Run the Z80 memory request handling code on its own core (Core1).
    multicore_reset_core1();
    multicore_launch_core1(core1Z80Runner);

    // Now infinite loop between the Z80 running on its own or halted in GDB.
    bool wasInterrupted = false;
    while (true)
    {
        if ((g_state == STATE_PAUSED || g_state == STATE_RUNNING) && Platform_CommHasReceiveData())
        {
            // GDB is sending commands so halt the Z80.
            g_command = COMMAND_HALT;
            wasInterrupted = true;
        }

        if (g_state == STATE_HALTED)
        {
            // The Z80 is halted so enter MRI to allow GDB to debug it.
            enterDebugger(&g_registers, wasInterrupted ? SIGINT : SIGTRAP, &g_reason);
            wasInterrupted = false;
            g_reason.type = MRI_PLATFORM_TRAP_TYPE_UNKNOWN;

            if (shouldReset())
            {
                // Reset the Z80 and prepare to resume the CPU without restoring register contents.
                g_z80Bus.resetZ80();
                g_command = COMMAND_HALT_RESET_RESUME;
            }
            else
            {
                // Debugging is complete so restore the Z80 registers and prepare to resume the CPU.
                g_command = COMMAND_HALT_RESUME;
            }

            while (g_state != STATE_HALT_DONE)
            {
                // Wait for the Z80 control thread to finish processing the resume command.
            }

            if (Platform_IsSingleStepping())
            {
                g_command = COMMAND_SINGLE_STEP;
            }
            else
            {
                g_command = COMMAND_RUN;
            }
        }
    }
}

static void __not_in_flash_func(core1Z80Runner)()
{
    // This is the Z80 control thread which runs out of RAM on the second core, Core1.
    // The second core is used so that things like the USB stack don't steal away CPU cycles from servicing Z80 memory
    // requests.
    // It runs out of RAM so that FLASH wait cycles don't slow down the servicing of Z80 memory requests.
    Z80Bus::Request req;
    // Just keep looping around forever in this thread, pulling Z80 memory requests from the PIO FIFO.
    // They will be serviced by a state machine which knows how to handle single stepping, halting, breakpoints, etc.
    while (true)
    {
        // The state machine automatically transitions from one state to the next state except when it is PAUSED,
        // RUNNING, or HALT_DONE.
        bool switchableState;
        switch (g_state)
        {
            case STATE_PAUSED:
            case STATE_RUNNING:
            case STATE_HALT_DONE:
                switchableState = true;
                break;
            default:
                switchableState = false;
                break;
        }

        // See if the main thread has sent a new command that should cause a switch into a new state.
        if (switchableState)
        {
            switch (g_command)
            {
                case COMMAND_PAUSE:
                    g_state = STATE_PAUSED;
                    break;
                case COMMAND_RUN:
                    g_state = STATE_RUNNING;
                    break;
                case COMMAND_SINGLE_STEP:
                    g_state = STATE_SINGLE_STEP_START;
                    break;
                case COMMAND_HALT:
                    g_state = STATE_HALT_START;
                    break;
                default:
                    break;
            }
        }

        if (req.isCompleted)
        {
            // Fetch next read/write transfer request.
            g_z80Bus.getNextRequest(&req);
            assert ( req.isMReq );
        }

        // What to run next depends on the current state of the control thread.
        switch (g_state)
        {
            case STATE_PAUSED:
                handlePauseState(&req);
                break;
            case STATE_RUNNING:
                handleRunningState(&req);
                break;
            case STATE_SINGLE_STEP_START:
                handleSingleStepStartState(&req);
                break;
            case STATE_HALT_START:
                handleHaltStartState(&req);
                break;
            case STATE_HALT_WAIT_RST8_VECTOR:
                handleHaltWaitRst8State(&req);
                break;
            case STATE_HALT_PUSHING:
                handleHaltPushingState(&req);
                break;
            case STATE_HALTED:
                handleHaltedState(&req);
                break;
            case STATE_HALT_POPPING:
                handleHaltPoppingState(&req);
                break;
            case STATE_HALT_DONE:
                handleHaltDoneState(&req);
                break;
            default:
                assert ( false );
                break;
        }
   }
}

// Most recent Z80 memory request is left in WAIT state. Stay in this state as long as g_command is COMMAND_PAUSE.
static void __not_in_flash_func(handlePauseState)(Z80Bus::Request* pReq)
{
    while (g_command == COMMAND_PAUSE)
    {
        // Wait here with Z80 WAIT' signal asserted until pause is undone.
    }

    // Current request is left outstanding.
    // It was just left in a WAIT' state all this time.
}

// Z80 is allowed to run normally, reading and writing from/to the RAM contents in g_memory.
static void __not_in_flash_func(handleRunningState)(Z80Bus::Request* pReq)
{
    // Perform the memory read or write from/to RAM and send response to Z80 via PIO.
    bool isRead = pReq->isRead;
    bool isM1 = pReq->isM1;
    bool isMReq = pReq->isMReq;
    bool wasPrefixDetected = pReq->wasPrefixDetected;
    uint16_t address = pReq->address;
    if (isRead)
    {
        if (doesMatchWatchpoint(address, MRI_PLATFORM_READ_WATCHPOINT))
        {
            // There is a watchpoint on this read so leave this request outstanding.
            // Will be processed by next state.
            g_state = STATE_HALT_START;
            return;
        }

        if (isMReq && isM1 && !wasPrefixDetected)
        {
            for (size_t i = 0 ; i < sizeof(g_breakpoints)/sizeof(g_breakpoints[0]) ; i++)
            {
                if (g_breakpoints[i] == address)
                {
                    // There is a breakpoint on this instruction so leave this request outstanding.
                    // Will be processed by next state.
                    g_state = STATE_HALT_START;
                    g_reason.type = MRI_PLATFORM_TRAP_TYPE_HWBREAK;
                    g_reason.address = address;
                    return;
                }
            }
        }

        // Process memory reads from RAM.
        pReq->value = g_memory[address];
    }
    else
    {
        if (doesMatchWatchpoint(address, MRI_PLATFORM_WRITE_WATCHPOINT))
        {
            // There is a watchpoint on this write so leave this request outstanding.
            // Will be processed by next state.
            g_state = STATE_HALT_START;
            return;
        }

        g_memory[address] = pReq->value;
    }
    g_z80Bus.completeRequest(pReq);
}

static bool __not_in_flash_func(doesMatchWatchpoint)(uint32_t address, PlatformWatchpointType type)
{
    for (size_t i = 0 ; i < sizeof(g_watchpoints)/sizeof(g_watchpoints[0]) ; i++)
    {
        if (address >= g_watchpoints[i].startAddress &&
            address < g_watchpoints[i].endAddress &&
            (g_watchpoints[i].type & type) != 0)
        {
            switch (g_watchpoints[i].type)
            {
                case MRI_PLATFORM_WRITE_WATCHPOINT:
                    g_reason.type = MRI_PLATFORM_TRAP_TYPE_WATCH;
                    break;
                case MRI_PLATFORM_READ_WATCHPOINT:
                    g_reason.type = MRI_PLATFORM_TRAP_TYPE_RWATCH;
                    break;
                case MRI_PLATFORM_READWRITE_WATCHPOINT:
                    g_reason.type = MRI_PLATFORM_TRAP_TYPE_AWATCH;
                    break;
            }
            g_reason.address = g_watchpoints[i].startAddress;

            return true;
        }
    }

    return false;
}



// RST 8 vector handler starts at this address.
static const uint16_t RST8_VECTOR_ADDRESS = 0x0008;

// Register pushing code to save all Z80 registers to the stack.
static const uint8_t g_rst8PushCode[] =
{
    // PUSH AF
    0xF5,
    // PUSH BC
    0xC5,
    // PUSH DE
    0xD5,
    // PUSH HL
    0xE5,
    // PUSH IX
    0xDD, 0xE5,
    // PUSH IY
    0xFD, 0xE5,
    // EXX
    0xD9,
    // EXX AF, AF'
    0x08,
    // PUSH AF
    0xF5,
    // PUSH BC
    0xC5,
    // PUSH DE
    0xD5,
    // PUSH HL
    0xE5,
    // LD A, R
    0xED, 0x5F,
    // LD C, A
    0x4F,
    // LD A, I
    0xED, 0x57,
    // LD B, A
    0x47,
    // PUSH BC
    0xC5

};

// Register popping code to restore all Z80 registers from the stack.
static uint8_t g_rst8PopCode[] =
{
    // LD SP, ABCD ; ABCD will be filled in before execution starts.
    0x31, 0x00, 0x00,
    // POP BC
    0xC1,
    // LD A, B
    0x78,
    // LD I, A
    0xED, 0x47,
    // LD A, C
    0x79,
    // LD R, A
    0xED, 0x4F,
    // POP HL
    0xE1,
    // POP DE
    0xD1,
    // POP BC
    0xC1,
    // POP AF
    0xF1,
    // EXX AF, AF'
    0x08,
    // EXX
    0xD9,
    // POP IY
    0xFD, 0xE1,
    // POP IX
    0xDD, 0xE1,
    // POP HL
    0xE1,
    // POP DE
    0xD1,
    // POP BC
    0xC1,
    // POP AF
    0xF1,
    // RETN
    0xED, 0x45
};

// Record the last two bytes written and their addresses as it could have been the PC being pushed to the stack.
static uint8_t  g_last2BytesWritten[2];
static uint16_t g_last2AddressesWritten[2];

// SP at the time of the halt. Based on g_last2AddressesWritten[0] since it will indicate where the PC was pushed onto
// the stack as part of the Z80 entering the "RST 8" handler.
static uint32_t g_SP = 0xFFFFFFFF;

// Start single stepping mode. Transition to HALT_START state after one M1' cycle has been received.
static void __not_in_flash_func(handleSingleStepStartState)(Z80Bus::Request* pReq)
{
    bool isRead = pReq->isRead;
    uint16_t address = pReq->address;
    bool isM1 = pReq->isM1;
    if (!isRead)
    {
        // Process the most recent write request.
        g_memory[address] = pReq->value;
    }
    else
    {
        // Process reads from RAM like normal.
        uint8_t value = g_memory[address];
        pReq->value = value;

        if (isM1 && !g_z80Bus.isPrefixOpcode(value))
        {
            // This is the last M1 request for this instruction so issue a halt on the next M1 request.
            g_state = STATE_HALT_START;
        }
    }
    g_z80Bus.completeRequest(pReq);
}

// Wait for next instruction fetch and insert "RST 8" instead of expected instruction.
static void __not_in_flash_func(handleHaltStartState)(Z80Bus::Request* pReq)
{
    // This is the Z80 machine code for a "RST 8" instruction.
    const uint8_t RST8_OPCODE = 0xCF;

    bool isRead = pReq->isRead;
    uint16_t address = pReq->address;
    bool isM1 = pReq->isM1;
    bool wasPrefixDetected = pReq->wasPrefixDetected;
    if (!isRead)
    {
        // Process the most recent write request.
        g_memory[address] = pReq->value;
    }
    else
    {
        if (isM1 && !wasPrefixDetected)
        {
            // This is the first M1 fetch for an instruction so return a "RST 8" instead.
            pReq->value = RST8_OPCODE;

            // In debug builds, can assert that at least one of these bytes has been changed by the time the first instruction
            // from the RST 8 vector is fetched. This makes sure that the PC was pushed to the stack as expected.
#ifndef NDEBUG
            memset(g_last2BytesWritten, 0xFF, sizeof(g_last2BytesWritten));
            memset(g_last2AddressesWritten, 0xFF, sizeof(g_last2AddressesWritten));
#endif
            // Wait for instruction fetch from RST 8 vector.
            g_state = STATE_HALT_WAIT_RST8_VECTOR;
        }
        else
        {
            // Process all other reads as usual.
            pReq->value = g_memory[address];
        }
    }
    g_z80Bus.completeRequest(pReq);
}

// Waiting for first RST 8 vector instruction to be fetched. Also records the 2 most recent memory writes as they will
// be the automatic push of the PC to the stack pointed to by SP. This is how the value of SP is discovered.
static void __not_in_flash_func(handleHaltWaitRst8State)(Z80Bus::Request* pReq)
{
    bool isRead = pReq->isRead;
    uint16_t address = pReq->address;
    bool isM1 = pReq->isM1;
    (void)isM1;
    if (!isRead)
    {
        // UNDONE: These should be the 2 stack writes for PC given that RST 8 instruction was inserted and we know exactly what happens next.
        //         So I don't need to actually push to device's RAM. I can hide in a global instead.
        // First process the most recent write request.
        uint8_t value = pReq->value;
        g_memory[address] = value;

        // Next, track the Z80 writes as the last 2 before fetching code from the RST 8 vector will be the pushed PC.
        g_last2BytesWritten[0] = g_last2BytesWritten[1];
        g_last2BytesWritten[1] = value;
        g_last2AddressesWritten[0] = g_last2AddressesWritten[1];
        g_last2AddressesWritten[1] = address;
    }
    else
    {
        // The first read must be to fetch the first instruction from the RST 8 vector.
        assert ( isM1 && address == RST8_VECTOR_ADDRESS );
        assert ( g_last2BytesWritten[0] != 0xFF || g_last2BytesWritten[1] != 0xFF ||
                 g_last2AddressesWritten[0] != 0xFFFF || g_last2AddressesWritten[1] != 0xFFFF );
        assert ( g_last2AddressesWritten[0] == g_last2AddressesWritten[1] + 1 );

        g_SP = g_last2AddressesWritten[1] + 2;
        if (g_SP == 0x0000)
        {
            g_SP = 0x10000;
        }
        g_state = STATE_HALT_PUSHING;

        // Current request is left outstanding.
        // Will be processed by next state.
        return;
    }
    g_z80Bus.completeRequest(pReq);
}

// Z80 is now given code which pushes all of the registers to the stack.
static void __not_in_flash_func(handleHaltPushingState)(Z80Bus::Request* pReq)
{
    bool isRead = pReq->isRead;
    uint16_t address = pReq->address;
    if (isRead)
    {
        assert ( address >= RST8_VECTOR_ADDRESS && address < RST8_VECTOR_ADDRESS + sizeof(g_rst8PushCode) + 1 );
        if (address < RST8_VECTOR_ADDRESS + sizeof(g_rst8PushCode))
        {
            // Read code from the global g_rst8PushCode array.
            pReq->value = g_rst8PushCode[address - RST8_VECTOR_ADDRESS];
        }
        else
        {
            // Have made it to the end of the register pushing code so copy the Z80 register values from what was just
            // saved on the stack and then signal the main thread that the halt is now complete.
            g_registers.sp = g_SP;
            g_registers.pc = readHalfWord(g_SP - 2);
            g_registers.af = readHalfWord(g_SP - 4);
            g_registers.bc = readHalfWord(g_SP - 6);
            g_registers.de = readHalfWord(g_SP - 8);
            g_registers.hl = readHalfWord(g_SP - 10);
            g_registers.ix = readHalfWord(g_SP - 12);
            g_registers.iy = readHalfWord(g_SP - 14);
            g_registers._af = readHalfWord(g_SP - 16);
            g_registers._bc = readHalfWord(g_SP - 18);
            g_registers._de = readHalfWord(g_SP - 20);
            g_registers._hl = readHalfWord(g_SP - 22);
            g_registers.ir = readHalfWord(g_SP - 24);

            // Subtract 1 from PC to account for the 1-byte "RST 8" instruction used to halt the CPU.
            g_registers.pc -= 1;

            // Correct for RST and PUSHing M1 cycles encountered before R value is captured.
            uint8_t r = g_registers.ir;
            r -= 17;
            g_registers.ir = (g_registers.ir & 0xFF00) | (uint16_t)r;

            g_state = STATE_HALTED;

            // Current request is left outstanding.
            // Leave Z80 in WAIT state until GDB has requested that execution be resumed.
            return;
        }
    }
    else
    {
        // Service writes (pushing register values to the stack) as normal.
        g_memory[address] = pReq->value;
    }

    g_z80Bus.completeRequest(pReq);
}

// Reads a little endian 16-bit value from the Z80's stack in RAM.
static uint16_t __not_in_flash_func(readHalfWord)(uint16_t address)
{
    return (uint16_t)g_memory[address] | ((uint16_t)g_memory[address+1] << 8);
}

// All registers have been pushed to the stack and then copied over to g_registers.
// Waits for COMMAND_HALT_RESUME or COMMAND_HALT_RESET_RESUME in g_command before continuing to next state.
// Once a resume has been requested, it copies update register values back onto the stack and then transitions to
// the HALT_POPPING state.
static void __not_in_flash_func(handleHaltedState)(Z80Bus::Request* pReq)
{
    while (g_command != COMMAND_HALT_RESUME && g_command != COMMAND_HALT_RESET_RESUME)
    {
        // Wait here with Z80 WAIT' signal asserted until execution should be resumed.
    }

    // Switch directly into STATE_HALT_DONE if the Z80 was just reset since registers don't need to be restored.
    if (g_command == COMMAND_HALT_RESET_RESUME)
    {
        g_state = STATE_HALT_DONE;
        // Current request is left outstanding.
        // It is still just left in a WAIT' state all this time.
        return;
    }

    // Correct for POPing and RETN M1 cycles that will be encountered after R value is restored.
    uint8_t r = g_registers.ir;
    r -= 16;
    g_registers.ir = (g_registers.ir & 0xFF00) | (uint16_t)r;

    // Copy the registers from GDB to Z80 stack so that they can be popped in the next state.
    g_SP = g_registers.sp;
    if (g_SP == 0x0000)
    {
        g_SP = 0x10000;
    }
    writeHalfWord(g_SP - 2, g_registers.pc);
    writeHalfWord(g_SP - 4, g_registers.af);
    writeHalfWord(g_SP - 6, g_registers.bc);
    writeHalfWord(g_SP - 8, g_registers.de);
    writeHalfWord(g_SP - 10, g_registers.hl);
    writeHalfWord(g_SP - 12, g_registers.ix);
    writeHalfWord(g_SP - 14, g_registers.iy);
    writeHalfWord(g_SP - 16, g_registers._af);
    writeHalfWord(g_SP - 18, g_registers._bc);
    writeHalfWord(g_SP - 20, g_registers._de);
    writeHalfWord(g_SP - 22, g_registers._hl);
    writeHalfWord(g_SP - 24, g_registers.ir);

    // Modify popping code to explicitly set SP to the value corresponding to g_registers.sp after the context has
    // been pushed to the stack (SP - 24).
    uint16_t initialPopSP = g_SP - 24;
    g_rst8PopCode[1] = initialPopSP;
    g_rst8PopCode[2] = initialPopSP >> 8;

    g_state = STATE_HALT_POPPING;

    // Current request is left outstanding.
    // It was just left in a WAIT' state all this time.
}

// Writes a little endian 16-bit value to the Z80's stack in RAM.
static void __not_in_flash_func(writeHalfWord)(uint16_t address, uint16_t value)
{
    g_memory[address] = value;
    g_memory[address+1] = value >> 8;
}

// The Z80 is now given code which pops all of the registers from the stack.
static void __not_in_flash_func(handleHaltPoppingState)(Z80Bus::Request* pReq)
{
    // Perform the read of the code from the RST 8 vector or registers from the stack.
    bool isRead = pReq->isRead;
    uint16_t address = pReq->address;
    assert ( isRead );
    (void)isRead;
    if (address >= RST8_VECTOR_ADDRESS + sizeof(g_rst8PushCode) &&
        address < RST8_VECTOR_ADDRESS + sizeof(g_rst8PushCode) + sizeof(g_rst8PopCode))
    {
        // Read code from the g_rst8PopCode array.
        pReq->value = g_rst8PopCode[address - (RST8_VECTOR_ADDRESS + sizeof(g_rst8PushCode))];
    }
    else
    {
        // Should be a stack read.
        assert ( address >= g_SP - 24 && address < g_SP );
        pReq->value = g_memory[address];

        // If just finished popping off the last byte of the return address from the stack then we can switch state.
        if (address == g_SP - 1)
        {
            g_state = STATE_HALT_DONE;
        }
    }

    g_z80Bus.completeRequest(pReq);
}

// The Z80 has returned to start executing at PC but is held in WAIT state until g_command is set to something other
// than COMMAND_HALT_RESUME.
static void __not_in_flash_func(handleHaltDoneState)(Z80Bus::Request* pReq)
{
    if (g_command == COMMAND_HALT_RESET_RESUME)
    {
        // Fake that this request has been completed since the PIO state machine has been reset since this request was
        // made.
        pReq->isCompleted = true;
    }

    while (g_command == COMMAND_HALT_RESUME || g_command == COMMAND_HALT_RESET_RESUME)
    {
        // Wait here with Z80 WAIT' signal asserted until execution should be resumed.
    }

    // Current request is left outstanding.
    // It was just left in a WAIT' state all this time.
}
