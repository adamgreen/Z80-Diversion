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
// Routines to expose remote GDB debug functionality for the Z80 via the mri debug core.
#include <assert.h>
#include <stdio.h>
#include <pico/stdio.h>
#include "mri_platform.h"

// MRI C headers
extern "C"
{
    #include <core/buffer.h>
    #include <core/core.h>
    #include <core/mri.h>
    #include <core/platforms.h>
    #include <core/signal.h>
    #include <core/semihost.h>
}


// Globals to store away pointers passed in from main program when calling initDebugger() and enterDebugger().
static uint8_t*           g_pMemory = NULL;
static uint32_t*          g_pBreakpoints = NULL;
static size_t             g_breakpointCount = 0;
static Z80Watchpoint*     g_pWatchpoints = NULL;
static size_t             g_watchpointCount = 0;
static Z80Registers*      g_pRegisters = NULL;
static PlatformTrapReason g_reason;
static uint8_t            g_signal = SIGINT;
static bool               g_shouldReset = false;

bool initDebugger(uint8_t* pMemory, size_t memorySize,
                  uint32_t* pBreakpoints, size_t breakpointCount,
                  Z80Watchpoint* pWatchpoints, size_t watchpointCount)
{
    assert ( pMemory );
    assert ( pBreakpoints || breakpointCount == 0 );
    assert ( pWatchpoints || watchpointCount == 0 );
    assert ( memorySize ==  DEBUGGER_MEMORY_SIZE );

    g_pMemory = pMemory;
    g_pBreakpoints = pBreakpoints;
    g_breakpointCount = breakpointCount;
    for (size_t i = 0 ; i < g_breakpointCount ; i++)
    {
        g_pBreakpoints[i] = FREE_BREAKPOINT;
    }
    g_pWatchpoints = pWatchpoints;
    g_watchpointCount = watchpointCount;
    for (size_t i = 0 ; i < g_watchpointCount ; i++)
    {
        g_pWatchpoints[i].startAddress = FREE_BREAKPOINT;
    }

    mriInit("");

    return true;
}

void enterDebugger(Z80Registers* pRegisters, uint8_t signal, PlatformTrapReason* pReason)
{
    g_pRegisters = pRegisters;
    g_signal = signal;
    g_reason = *pReason;

    // UNDONE:
    ContextSection contextSection = { .pValues = g_pRegisters->regs, sizeof(g_pRegisters->regs)/sizeof(g_pRegisters->regs[0]) };
    MriContext context;
    Context_Init(&context, &contextSection, 1);

    mriDebugException(&context);
}

bool shouldReset()
{
    return g_shouldReset;
}





// *********************************************************************************************************************
// MRI PLATFORM LAYER
//
// Routines needed by the MRI core for platform specific operations. Declarations are in MRI's platforms.h
// *********************************************************************************************************************
void Platform_Init(Token* pParameterTokens)
{
    // Disable write buffering on stdout used by Platform_Comm*() functions.
    setvbuf(stdout, NULL, _IONBF, 0);
}




// *********************************************************************************************************************
// Routines called by the MRI core to access the storage used for buffering inbound and outbound packets from/to GDB.
// g_packetBuffer must be large enough to contain a 'G' packet sent from GDB to update all of the CPU registers in
// the context at once. This is:
//      1 (byte for 'G' itself) +
//        [ 2 (text hex digits per byte) *
//          2 (bytes per 16-bit register) *
//          13 (registers stored in context for CPU) ] +
//      4 (bytes for packet overhead of '$', '#', and 2 hex digit checksum)
//      = 1 + 2 * 2 * 13 + 4 = 57
// *********************************************************************************************************************
// UNDONE: Try increasing the size to see if it helps with performance.
static char g_packetBuffer[8*1024];

char* Platform_GetPacketBuffer(void)
{
    return g_packetBuffer;
}

size_t Platform_GetPacketBufferSize(void)
{
    return sizeof(g_packetBuffer);
}




// *********************************************************************************************************************
// Routine called by the MRI core to create the T response to be sent back to GDB on debug stops.
// *********************************************************************************************************************
static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint16_t registerValue);
static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);

void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    sendRegisterForTResponse(pBuffer, PC, g_pRegisters->regs[PC]);
    sendRegisterForTResponse(pBuffer, SP, g_pRegisters->regs[SP]);
}

static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint16_t registerValue)
{
    Buffer_WriteByteAsHex(pBuffer, registerOffset);
    Buffer_WriteChar(pBuffer, ':');
    writeBytesToBufferAsHex(pBuffer, &registerValue, sizeof(registerValue));
    Buffer_WriteChar(pBuffer, ';');
}

static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;

    for (i = 0 ; i < byteCount ; i++)
    {
        Buffer_WriteByteAsHex(pBuffer, *pByte++);
    }
}



// *********************************************************************************************************************
// Routines called by the MRI core each time the CPU is halted and resumed.
// *********************************************************************************************************************
static uintmri_t g_originalPC = 0;
void Platform_EnteringDebugger(void)
{
    g_originalPC = Platform_GetProgramCounter();
    g_shouldReset = false;
    Platform_DisableSingleStep();
}

void Platform_LeavingDebugger(void)
{
}



// *********************************************************************************************************************
// Routines called by the MRI core to read and write memory on the target device.
// *********************************************************************************************************************
uint64_t Platform_MemRead64(uintmri_t address)
{
    return *(uint64_t*)(g_pMemory + address);
}

uint32_t Platform_MemRead32(uintmri_t address)
{
    return *(uint32_t*)(g_pMemory + address);
}

uint16_t Platform_MemRead16(uintmri_t address)
{
    return *(uint16_t*)(g_pMemory + address);
}

uint8_t Platform_MemRead8(uintmri_t address)
{
    return *(uint8_t*)(g_pMemory + address);
}

void Platform_MemWrite64(uintmri_t address, uint64_t value)
{
    *(uint64_t*)(g_pMemory + address) = value;
}

void Platform_MemWrite32(uintmri_t address, uint32_t value)
{
    *(uint32_t*)(g_pMemory + address) = value;
}

void Platform_MemWrite16(uintmri_t address, uint16_t value)
{
    *(uint16_t*)(g_pMemory + address) = value;
}

void Platform_MemWrite8(uintmri_t address, uint8_t value)
{
    *(uint8_t*)(g_pMemory + address) = value;
}

int Platform_WasMemoryFaultEncountered(void)
{
    // All 64k of RAM are always accessible on this board so there is never a fault + the Z80 doesn't really have the
    // the concept of a memory fault that it can catch.
    return 0;
}

void Platform_SyncICacheToDCache(uintmri_t address, size_t size)
{
}



// *********************************************************************************************************************
// Implementation of the Platform_Comm* functions.
// Routines used by the MRI core to communicate with GDB. The implementations below use the Pico SDK's USB based STDIO
// library for communication.
// *********************************************************************************************************************
static int  g_savedChar = 0;
static bool g_haveSavedChar = false;

int Platform_CommHasReceiveData(void)
{
    if (g_haveSavedChar)
    {
        return 1;
    }

    int nextByte = getchar_timeout_us(0);
    if (nextByte == PICO_ERROR_TIMEOUT)
    {
        return 0;
    }

    g_savedChar = nextByte;
    g_haveSavedChar = true;
    return 1;
}

int Platform_CommHasTransmitCompleted(void)
{
    stdio_flush();
    return 1;
}

int Platform_CommReceiveChar(void)
{
    while (!Platform_CommHasReceiveData())
    {
    }
    g_haveSavedChar = false;
    return g_savedChar;
}

void Platform_CommSendBuffer(Buffer* pBuffer)
{
    size_t bytesWritten = fwrite(Buffer_GetArray(pBuffer), 1, Buffer_GetLength(pBuffer), stdout);
    assert ( bytesWritten == Buffer_GetLength(pBuffer) );
    (void)bytesWritten;
}

void Platform_CommSendChar(int character)
{
    putchar_raw(character);
}



// *********************************************************************************************************************
// Platform_HandleGDBCommand() is called by the MRI core to give platforms a chance to override and handle any command
// sent from GDB.
// *********************************************************************************************************************
uint32_t Platform_HandleGDBCommand(Buffer* pBuffer)
{
    return 0;
}



// *********************************************************************************************************************
// Routines called by the MRI core when dealing with exception/fault causes.
// *********************************************************************************************************************
uint8_t Platform_DetermineCauseOfException(void)
{
    return g_signal;
}

PlatformTrapReason Platform_GetTrapReason(void)
{
    return g_reason;
}

void Platform_DisplayFaultCauseToGdbConsole(void)
{
    // There are no fault status registers to dump in a human friendly fashion on the Z80.
    return;
}



// *********************************************************************************************************************
// Routines called by the MRI core to enable/disable single stepping on the target CPU.
// On this Z80 board, just a flag is set and the Z80 control state machine just contains code to check the state of this
// global to determine if it should enter its single stepping state or not.
// *********************************************************************************************************************
static bool g_isSingleStepping = false;
void Platform_EnableSingleStep(void)
{
    g_isSingleStepping = true;
}

void Platform_DisableSingleStep(void)
{
    g_isSingleStepping = false;
}

int Platform_IsSingleStepping(void)
{
    return g_isSingleStepping;
}



// *********************************************************************************************************************
// Routines called by the MRI core to access the CPU's program counter.
// *********************************************************************************************************************
uintmri_t Platform_GetProgramCounter(void)
{
    return g_pRegisters->pc;
}

void Platform_SetProgramCounter(uintmri_t newPC)
{
    g_pRegisters->pc = newPC;
}

void Platform_AdvanceProgramCounterToNextInstruction(void)
{
    assert ( false );
}

int Platform_WasProgramCounterModifiedByUser(void)
{
    return Platform_GetProgramCounter() != g_originalPC;
}



// *********************************************************************************************************************
// Routines called by the MRI core to retrieve the XML describing the CPU's memory layout.
// *********************************************************************************************************************
static const char g_memoryMapXML[] = "<?xml version=\"1.0\"?>"
                                     "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                     "<memory-map>"
                                     "<memory type=\"ram\" start=\"0x0000\" length=\"0x10000\"> </memory>"
                                     "</memory-map>";

size_t Platform_GetDeviceMemoryMapXmlSize(void)
{
    return sizeof(g_memoryMapXML) - 1;
}

const char* Platform_GetDeviceMemoryMapXml(void)
{
    return g_memoryMapXML;
}



// *********************************************************************************************************************
// Routines called by the MRI core to retrieve the XML describing the CPU's register context.
// *********************************************************************************************************************
static const char g_targetXML[] =   "<?xml version=\"1.0\"?>"
                                    "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">"
                                    "<feature name=\"org.gnu.gdb.z80.cpu\">"
                                    "  <flags id=\"af_flags\" size=\"2\">"
                                    "    <field name=\"C\" start=\"0\" end=\"0\"/>"
                                    "    <field name=\"N\" start=\"1\" end=\"1\"/>"
                                    "    <field name=\"P/V\" start=\"2\" end=\"2\"/>"
                                    "    <field name=\"F3\" start=\"3\" end=\"3\"/>"
                                    "    <field name=\"H\" start=\"4\" end=\"4\"/>"
                                    "    <field name=\"F5\" start=\"5\" end=\"5\"/>"
                                    "    <field name=\"Z\" start=\"6\" end=\"6\"/>"
                                    "    <field name=\"S\" start=\"7\" end=\"7\"/>"
                                    "  </flags>"
                                    "  <reg name=\"af\" bitsize=\"16\" type=\"af_flags\"/>"
                                    "  <reg name=\"bc\" bitsize=\"16\" type=\"uint16\"/>"
                                    "  <reg name=\"de\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"hl\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"sp\" bitsize=\"16\" type=\"data_ptr\" />"
                                    "  <reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\" />"
                                    "  <reg name=\"ix\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"iy\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"af'\" bitsize=\"16\" type=\"af_flags\"/>"
                                    "  <reg name=\"bc'\" bitsize=\"16\" type=\"uint16\"/>"
                                    "  <reg name=\"de'\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"hl'\" bitsize=\"16\" type=\"data_ptr\"/>"
                                    "  <reg name=\"ir\" bitsize=\"16\" type=\"uint16\"/>"
                                    "</feature>";

size_t Platform_GetTargetXmlSize(void)
{
    return sizeof(g_targetXML) - 1;
}

const char* Platform_GetTargetXml(void)
{
    return g_targetXML;
}



// *********************************************************************************************************************
// Routines called by the MRI core to set and clear breakpoints.
// *********************************************************************************************************************
static uint32_t* findMatchingBreakpoint(uintmri_t address);
static uint32_t* findFreeBreakpoint();

__throws void Platform_SetHardwareBreakpointOfGdbKind(uintmri_t address, uintmri_t kind)
{
    uint32_t* pBreakpoint = findMatchingBreakpoint(address);
    if (pBreakpoint == NULL)
    {
        pBreakpoint = findFreeBreakpoint();
    }
    if (pBreakpoint == NULL)
    {
        __throw(exceededHardwareResourcesException);
    }

    *pBreakpoint = address;
}

static uint32_t* findMatchingBreakpoint(uintmri_t address)
{
    for (size_t i = 0 ; i < g_breakpointCount ; i++)
    {
        if (g_pBreakpoints[i] == address)
        {
            return &g_pBreakpoints[i];
        }
    }
    return NULL;
}

static uint32_t* findFreeBreakpoint()
{
    for (size_t i = 0 ; i < g_breakpointCount ; i++)
    {
        if (g_pBreakpoints[i] == FREE_BREAKPOINT)
        {
            return &g_pBreakpoints[i];
        }
    }
    return NULL;
}

__throws void Platform_SetHardwareBreakpoint(uintmri_t address)
{
    Platform_SetHardwareBreakpointOfGdbKind(address, 0);
}

__throws void Platform_ClearHardwareBreakpointOfGdbKind(uintmri_t address, uintmri_t kind)
{
    uint32_t* pBreakpoint = findMatchingBreakpoint(address);
    if (pBreakpoint != NULL)
    {
        *pBreakpoint = FREE_BREAKPOINT;
    }
}

__throws void Platform_ClearHardwareBreakpoint(uintmri_t address)
{
    Platform_ClearHardwareBreakpointOfGdbKind(address, 0);
}



// *********************************************************************************************************************
// Routines called by the MRI core to set and clear watch points.
// *********************************************************************************************************************
static Z80Watchpoint* findMatchingWatchpoint(uint32_t startAddress, uint32_t endAddress,  PlatformWatchpointType type);
static Z80Watchpoint* findFreeWatchpoint();

__throws void Platform_SetHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t startAddress = (uint32_t)address;
    uint32_t endAddress = startAddress + (uint32_t)size;

    Z80Watchpoint* pWatchpoint = findMatchingWatchpoint(startAddress, endAddress, type);
    if (pWatchpoint == NULL)
    {
        pWatchpoint = findFreeWatchpoint();
    }
    if (pWatchpoint == NULL)
    {
        __throw(exceededHardwareResourcesException);
    }
    pWatchpoint->startAddress = startAddress;
    pWatchpoint->endAddress = endAddress;
    pWatchpoint->type = type;
}

static Z80Watchpoint* findMatchingWatchpoint(uint32_t startAddress, uint32_t endAddress,  PlatformWatchpointType type)
{
    for (size_t i = 0 ; i < g_watchpointCount ; i++)
    {
        if (g_pWatchpoints[i].startAddress == startAddress &&
            g_pWatchpoints[i].endAddress == endAddress &&
            g_pWatchpoints[i].type == type)
        {
            return &g_pWatchpoints[i];
        }
    }
    return NULL;
}

static Z80Watchpoint* findFreeWatchpoint()
{
    for (size_t i = 0 ; i < g_watchpointCount ; i++)
    {
        if (g_pWatchpoints[i].startAddress == FREE_WATCHPOINT)
        {
            return &g_pWatchpoints[i];
        }
    }
    return NULL;
}

__throws void Platform_ClearHardwareWatchpoint(uintmri_t address, uintmri_t size,  PlatformWatchpointType type)
{
    uint32_t startAddress = (uint32_t)address;
    uint32_t endAddress = startAddress + (uint32_t)size;

    Z80Watchpoint* pWatchpoint = findMatchingWatchpoint(startAddress, endAddress, type);
    if (pWatchpoint != NULL)
    {
        pWatchpoint->startAddress = FREE_WATCHPOINT;
    }
}



// *********************************************************************************************************************
// Routines called by the MRI core to reset the device.
// *********************************************************************************************************************
void Platform_ResetDevice(void)
{
    g_shouldReset = true;
}



// *********************************************************************************************************************
// Semihost functionality for redirecting operations such as file I/O to the GNU debugger.
// *********************************************************************************************************************
PlatformInstructionType Platform_TypeOfCurrentInstruction(void)
{
    return MRI_PLATFORM_INSTRUCTION_OTHER;
}

PlatformSemihostParameters Platform_GetSemihostCallParameters(void)
{
    PlatformSemihostParameters params = { .parameter1 = 0, .parameter2 = 0, .parameter3 = 0, .parameter4 = 0 };
    return params;
}

void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int errNo)
{
}

int Semihost_HandleSemihostRequest(void)
{
    assert ( false );
    return 0;
}

int Semihost_IsDebuggeeMakingSemihostCall(void)
{
    return 0;
}



// *********************************************************************************************************************
// Routines called by the MRI core for RTOS support. Can just return 0 or a NULL pointer as RTOS thread dumping isn't
// support for this Z80 board.
// *********************************************************************************************************************
uintmri_t Platform_RtosGetHaltedThreadId(void)
{
    return 0;
}

uintmri_t Platform_RtosGetFirstThreadId(void)
{
    return 0;
}

uintmri_t Platform_RtosGetNextThreadId(void)
{
    return 0;
}

const char* Platform_RtosGetExtraThreadInfo(uintmri_t threadId)
{
    return NULL;
}

MriContext* Platform_RtosGetThreadContext(uintmri_t threadId)
{
    return NULL;
}

int Platform_RtosIsThreadActive(uintmri_t threadId)
{
    return 0;
}

int Platform_RtosIsSetThreadStateSupported(void)
{
    return 0;
}

void Platform_RtosSetThreadState(uintmri_t threadId, PlatformThreadState state)
{
}

void Platform_RtosRestorePrevThreadState(void)
{
}

void Platform_HandleFaultFromHighPriorityCode(void)
{
}
