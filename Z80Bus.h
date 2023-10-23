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
// Class which accesses the Z80 bus, leveraging the RP2040's PIO peripheral.
#ifndef Z80BUS_H_
#define Z80BUS_H_

#include <hardware/pio.h>


// Class used to interface to the Z80 microprocessor bus.
class Z80Bus
{
    public:
        // Constructor just sets up object. Need to call init() methods to actually initialize the PIO state machine.
        Z80Bus()
        {
        }

        virtual ~Z80Bus()
        {
            uninit();
        }

        // Call this init() method to load the assembly language code into the first available PIO instance
        // available.
        //
        // frequency - Frequency to run the Z80 CLK in Hz.
        // resetPin - The pin numbed of the Z80 RESET' signal.
        // z80ClockPin - The pin number of the Z80 CLK signal.
        // mreqPin - The pin connected to the Z80 MREQ' signal. Its pin number must be shiftOddPin + 1.
        // ioreqPin - The pin connected to the Z80 IOREQ' signal. Its pin number must be mreqPin + 1.
        // rdPin - The pin connected to the Z80 RD' signal.
        // wrPin - The pin connected to the Z80 WR' signal. Its pin number must be rdPin + 1.
        // d0Pin - The pin connected to the Z80 D0 signal. Its pin number must be wrPin + 1.
        // d1Pin - The pin connected to the Z80 D1 signal. Its pin number must be d0Pin + 1.
        // d2Pin - The pin connected to the Z80 D2 signal. Its pin number must be d1Pin + 1.
        // d3Pin - The pin connected to the Z80 D3 signal. Its pin number must be d2Pin + 1.
        // d4Pin - The pin connected to the Z80 D4 signal. Its pin number must be d3Pin + 1.
        // d5Pin - The pin connected to the Z80 D5 signal. Its pin number must be d4Pin + 1.
        // d6Pin - The pin connected to the Z80 D6 signal. Its pin number must be d5Pin + 1.
        // d7Pin - The pin connected to the Z80 D7 signal. Its pin number must be d6Pin + 1.
        // m1Pin - The pin connected to the Z80 M1' signal. Its pin number must d7Pin + 1.
        // waitPin - The pin connected to the Z80 WAIT' signal.
        // rfshPin - The pin connected to the Z80 RFSH' signal.
        // haltPin - The pin connected to the Z80 HALT' signal.
        // shiftClockPin - The pin connected to the clock signal of the two shift registers.
        // shiftLatchPin - The pin connect to the latch signal of the two shift registers. Its pin number must be
        //                 shiftClockPin + 1.
        // shiftEvenPin - The pin connected to the serial out signal of the even bits shift register.
        // shiftOddPin - The pin connected to the serial out signal of the odd bits shift register. Its pin number must
        //               be shiftEvenPin + 1.
        //
        // Returns true if everything was initialized successfully.
        // Returns false if the assembly language code fails to load into either PIO instance.
        bool init(uint32_t frequency,
                  uint32_t resetPin, uint32_t z80ClockPin,
                  uint32_t mreqPin, uint32_t ioreqPin,
                  uint32_t rdPin, uint32_t wrPin,
                  uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                  uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                  uint32_t m1Pin, uint32_t waitPin, uint32_t rfshPin, uint32_t haltPin,
                  uint32_t shiftClockPin, uint32_t shiftLatchPin, uint32_t shiftEvenPin, uint32_t shiftOddPin);

        // Call this uninit() method to shutdown the PIO state machine and free up the associated resources, including
        // the program space.
        void uninit();

        // Structure used to return information about the latest MREQ or IOREQ from the Z80.
        struct Request
        {
            // 16-bit address requested by Z80.
            uint16_t address;
            // Request has either MREQ' or IOREQ' asserted.
            bool     isMReq;
            // Request has either RD' or WR' asserted.
            bool     isRead;
            // Was M1' asserted?
            bool     isM1;
            // Is this a M1' request for a Z80 prefix opcode?
            bool     isPrefixCode;
            // Was the previous M1' request for Z80 prefix opcode?
            bool     wasPrefixDetected;
            // Has this request been completed yet?
            bool     isCompleted;
            // 8-bit data value for this Z80 request.
            // isRead == false
            //  getNextRequest() will set this to 8-bit value to be written to address.
            // isRead == true
            //  Caller must set to appropriate value (based on address) before calling completeRequest().
            uint8_t  value;

            Request()
            {
                address = 0;
                isMReq = false;
                isRead = false;
                isM1 = false;
                isCompleted = true;
                wasPrefixDetected = false;
                value = 0;
            }
        };

        // Returns the next Z80 8-bit read or write request in Request structure.
        // Caller must call completeRequest() once done processing this read/write request so that the Z80 WAIT'
        // signal can be de-asserted.
        //
        // pReq - Pointer to Request structure to be filled in with information about the current Z80 read/write
        //        request. Will be passed into completeRequest() as well.
        void getNextRequest(Request* pReq)
        {
            assert ( pReq->isCompleted );

            uint32_t dataBits = pio_sm_get_blocking(m_readWritePIO, m_readWriteStateMachine);
            bool isM1 = !(dataBits & (1 << 10));
            bool isRead = !(dataBits & (1 << 0));
            bool isWrite = !(dataBits & (1 << 1));
            assert ( isRead ^ isWrite );
            assert ( !isM1 || (isM1 && !isWrite) );
            (void)isWrite;
            pReq->value = (dataBits >> 2) & 0xFF;
            pReq->isRead = isRead;
            pReq->isM1 = isM1;

            uint32_t addressBits = pio_sm_get_blocking(m_addressPIO, m_addressStateMachine);
            bool isMReq = !!(addressBits & 0x10000);
            bool isIoReq = !!(addressBits & 0x20000);
            assert ( isMReq ^ isIoReq );
            (void)isIoReq;
            pReq->address = addressBits & 0xFFFF;
            pReq->isMReq = isMReq;
            pReq->wasPrefixDetected = m_wasPrefixDetected;

            pReq->isCompleted = false;
        }

        // Called once caller is done processing the most recent read/write request returned from getNextRequest().
        // Will de-assert Z80 WAIT' signal. For Z80 RD' requests it will also provide the requested 8-bit value to
        // the Z80 over D0-D7.
        //
        // pReq - Pointer to the Request structure filled in by the most recent call to getNextRequest().
        //        When processing Z80 read requests, pReq->value must be filled in by the caller with the appropriate
        //        8-bit value to be sent to the Z80 over D0-D7.
        void completeRequest(Request* pReq)
        {
            assert ( !pReq->isCompleted );

            const uint32_t pinDirOutput = 0xFF;
            const uint32_t pinDirInput = 0x00;

            // Let ReadWriteStateMachine know that CPU is done processing current Z80 read/write request.
            // ReadWriteStateMachine sets D0-D7 pin directions correctly for request (output for reads and input for
            // writes) and then sets D0-D7 pins to pReq->value. It then waits for RD' or WR' (which depends on WAIT
            // instruction sent by CPU below) to be de-asserted. Once RD'/WR' has been de-asserted then D0-D7 pin
            // directions are set back to input.
            bool isRead = pReq->isRead;
            uint32_t value = pReq->value & 0xFF;
            uint32_t pinDirsAndData = (value << 8) | (isRead ? pinDirOutput : pinDirInput);
            uint32_t pioInstructionAndPinDirs = (pinDirInput << 16) | (isRead ? m_waitReadInstruction : m_waitWriteInstruction);
            pio_sm_put_blocking(m_readWritePIO, m_readWriteStateMachine, pinDirsAndData);
            pio_sm_put_blocking(m_readWritePIO, m_readWriteStateMachine, pioInstructionAndPinDirs);

            // Let AddressStateMachine know that CPU is done processing current Z80 read/write request.
            // AddressStateMachine will de-assert Z80 WAIT' signal and wait for MREQ' or IOREQ' (which depends on WAIT
            // instruction sent by CPU below) to be de-asserted.
            // NOTE: AddressStateMachine is responsible for de-asserting WAIT' so complete it last.
            bool isMReq = pReq->isMReq;
            bool isM1 = pReq->isM1;
            pio_sm_put_blocking(m_addressPIO, m_addressStateMachine, isMReq ? m_waitMREQInstruction : m_waitIOREQInstruction);

            if (isRead && isMReq && isM1)
            {
                m_wasPrefixDetected = isPrefixOpcode(value);
            }

            pReq->isCompleted = true;
        }

        static bool isPrefixOpcode(uint8_t value)
        {
            return value == 0xCB || value == 0xDD || value == 0xED || value == 0xFD;
        }


    protected:
        void resetZ80(uint32_t z80ResetPin, uint32_t z80ClockPin, uint32_t z80Frequency);
        bool initAddressStateMachine(uint32_t frequency, uint32_t waitPin,
                                     uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                     uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                     uint32_t rfshPin);
        bool initAddressStateMachine(PIO pio, uint32_t frequency, uint32_t waitPin,
                                     uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                     uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                     uint32_t rfshPin);
        bool initReadWriteStateMachine(uint32_t rdPin, uint32_t wrPin,
                                       uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                       uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                       uint32_t m1Pin);
        bool initReadWriteStateMachine(PIO pio,
                                       uint32_t rdPin, uint32_t wrPin,
                                       uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                       uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                       uint32_t m1Pin);
        void initClock(uint32_t frequency, uint32_t z80ClockPin);
        void cacheWaitInstructionOpCodes();


        // Unknown values can be set to this.
        const uint32_t UNKNOWN_VAL = 0xFFFFFFFF;

        // PIO State Machine Resources.
        PIO             m_addressPIO = NULL;
        PIO             m_readWritePIO = NULL;

        uint32_t        m_addressStateMachine = UNKNOWN_VAL;
        uint32_t        m_addressProgramOffset = UNKNOWN_VAL;
        uint32_t        m_readWriteStateMachine = UNKNOWN_VAL;
        uint32_t        m_readWriteProgramOffset = UNKNOWN_VAL;

        // Reset pin of the Z80.
        uint32_t        m_resetPin = 0;
        // Pins used by the first state machine.
        // Set pin.
        uint32_t        m_z80ClockPin = 0;
        // Side set pins.
        uint32_t        m_shiftClockPin = 0;
        uint32_t        m_shiftLatchPin = 0;
        // Input pins.
        uint32_t        m_shiftEvenPin = 0;
        uint32_t        m_shiftOddPin = 0;
        uint32_t        m_mreqPin = 0;
        uint32_t        m_ioreqPin = 0;
        // Jump conditional pin.
        uint32_t        m_rfshPin = 0;
        // Pins used by the second state machine.
        // Side set pins.
        uint32_t        m_waitPin = 0;
        // Input pins.
        uint32_t        m_rdPin = 0;
        uint32_t        m_wrPin = 0;
        uint32_t        m_m1Pin = 0;
        uint32_t        m_haltPin = 0;
        // Input/Output pins.
        uint32_t        m_d0Pin = 0;
        uint32_t        m_d1Pin = 0;
        uint32_t        m_d2Pin = 0;
        uint32_t        m_d3Pin = 0;
        uint32_t        m_d4Pin = 0;
        uint32_t        m_d5Pin = 0;
        uint32_t        m_d6Pin = 0;
        uint32_t        m_d7Pin = 0;

        // Cache of WAIT GPIO instructions that are sent to state machines to execute upon completion of read/write
        // request.
        uint32_t m_waitReadInstruction = 0;
        uint32_t m_waitWriteInstruction = 0;
        uint32_t m_waitMREQInstruction = 0;
        uint32_t m_waitIOREQInstruction = 0;

        // Did the previous M1 request include a Z80 prefix opcode.
        bool m_wasPrefixDetected = false;
};

#endif // Z80BUS_H_
