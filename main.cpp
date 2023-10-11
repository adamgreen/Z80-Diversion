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
#include <stdio.h>
#include <string.h>


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

// What frequency should the Z80 be clocked at?
static const uint32_t Z80_FREQUENCY = 500000;


#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <z80_bus.pio.h>

// Class used to interface to the Z80 microprocessor bus.
class Z80_Bus
{
    public:
        // Constructor just sets up object. Need to call init() methods to actually initialize the PIO state machine.
        Z80_Bus()
        {
        }

        virtual ~Z80_Bus()
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
            // UNDONE: Make value uint8_t and address uint16_t.
            // 16-bit address requested by Z80.
            uint32_t address;
            // Request has either MREQ' or IOREQ' asserted.
            bool isMReq;
            // Request has either RD' or WR' asserted.
            bool isRead;
            // Was M1' asserted?
            bool isM1;
            // 8-bit data value for this Z80 request.
            // isRead == false
            //  getNextRequest() will set this to 8-bit value to be written to address.
            // isRead == true
            //  Caller must set to appropriate value (based on address) before calling completeRequest().
            uint32_t value;
        };

        // Returns the next Z80 8-bit read or write request in Request structure.
        // Caller must call completeRequest() once done processing this read/write request so that the Z80 WAIT'
        // signal can be de-asserted.
        //
        // pReq - Pointer to Request structure to be filled in with information about the current Z80 read/write
        //        request. Will be passed into completeRequest() as well.
        void getNextRequest(Request* pReq)
        {
            uint32_t dataBits = pio_sm_get_blocking(m_readWritePIO, m_readWriteStateMachine);
            bool isM1 = !(dataBits & (1 << 10));
            bool isRead = !(dataBits & (1 << 0));
            bool isWrite = !(dataBits & (1 << 1));
            assert ( isRead ^ isWrite );
            assert ( !isM1 || (isM1 && !isWrite) );
            pReq->value = (dataBits >> 2) & 0xFF;
            pReq->isRead = isRead;
            pReq->isM1 = isM1;

            uint32_t addressBits = pio_sm_get_blocking(m_addressPIO, m_addressStateMachine);
            bool isMReq = !!(addressBits & 0x10000);
            bool isIoReq = !!(addressBits & 0x20000);
            assert ( isMReq ^ isIoReq );
            pReq->address = addressBits & 0xFFFF;
            pReq->isMReq = isMReq;
        }

        // Called once caller is done processing the most recent read/write request returned from getNextRequest().
        // Will de-assert Z80 WAIT' signal. For Z80 RD' requests it will also provide the requested 8-bit value to
        // the Z80 over D0-D7.
        //
        // pReq - Pointer to the Request structure filled in by the most recent call to getNextRequest().
        //        When processing Z80 read requests, pReq->value must be filled in by the caller with the appropriate
        //        8-bit value to be sent to the Z80 over D0-D7.
        void completeRequest(const Request* pReq)
        {
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
            pio_sm_put_blocking(m_addressPIO, m_addressStateMachine, isMReq ? m_waitMREQInstruction : m_waitIOREQInstruction);
        }


    protected:
        void resetZ80(uint32_t z80ResetPin, uint32_t z80ClockPin, uint32_t z80Frequency);
        bool initClockStateMachine(uint32_t frequency, uint32_t z80ClockPin);
        bool initClockStateMachine(PIO pio, uint32_t frequency, uint32_t z80ClockPin);
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
        void cacheWaitInstructionOpCodes();


        // Unknown values can be set to this.
        const uint32_t UNKNOWN_VAL = 0xFFFFFFFF;

        PIO             m_clockPIO = NULL;
        PIO             m_addressPIO = NULL;
        PIO             m_readWritePIO = NULL;

        uint32_t        m_clockStateMachine = UNKNOWN_VAL;
        uint32_t        m_clockProgramOffset = UNKNOWN_VAL;
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
};


bool Z80_Bus::init(uint32_t frequency,
                   uint32_t resetPin, uint32_t z80ClockPin,
                   uint32_t mreqPin, uint32_t ioreqPin,
                   uint32_t rdPin, uint32_t wrPin,
                   uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                   uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                   uint32_t m1Pin, uint32_t waitPin, uint32_t rfshPin, uint32_t haltPin,
                   uint32_t shiftClockPin, uint32_t shiftLatchPin, uint32_t shiftEvenPin, uint32_t shiftOddPin)
{
    // Verify that the side set pins are in ascending pin number order.
    assert ( shiftLatchPin == shiftClockPin + 1 );
    // Verify that the input pins are in ascending pin number order.
    assert ( shiftOddPin == shiftEvenPin + 1);
    assert ( mreqPin = shiftOddPin + 1 );
    assert ( ioreqPin == mreqPin + 1 );
    assert ( wrPin == rdPin + 1 );
    assert ( d0Pin == wrPin + 1 );
    assert ( d1Pin == d0Pin + 1 );
    assert ( d2Pin == d1Pin + 1 );
    assert ( d3Pin == d2Pin + 1 );
    assert ( d4Pin == d3Pin + 1 );
    assert ( d5Pin == d4Pin + 1 );
    assert ( d6Pin == d5Pin + 1 );
    assert ( d7Pin == d6Pin + 1 );
    assert ( m1Pin == d7Pin + 1);

    // Cleanup the PIO if it was already initialized before continuing to initialize it again, with probably different
    // parameters (ie. frequency).
    uninit();

    resetZ80(resetPin, z80ClockPin, frequency);

    if (!initAddressStateMachine(frequency, waitPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin, mreqPin, ioreqPin, rfshPin) ||
        !initReadWriteStateMachine(rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin) ||
        !initClockStateMachine(frequency, z80ClockPin))
    {
        uninit();
        return false;
    }
    cacheWaitInstructionOpCodes();

    m_haltPin = haltPin;

    return true;
}

void Z80_Bus::resetZ80(uint32_t z80ResetPin, uint32_t z80ClockPin, uint32_t z80Frequency)
{
    // Configure pins connected to the Z80 CLK and RESET' signals as output.
    const uint32_t clockPinMask = 1 << z80ClockPin;
    const uint32_t resetPinMask = 1 << z80ResetPin;
    const uint32_t pinMask = clockPinMask | resetPinMask;
    // Start out with both the Z80 CLK and RESET' signals low.
    gpio_clr_mask(pinMask);
    // Configure these pins as SIO outputs.
    gpio_set_dir_out_masked(pinMask);
    gpio_set_function(z80ResetPin, GPIO_FUNC_SIO);
    gpio_set_function(z80ClockPin, GPIO_FUNC_SIO);

    // Reset the Z80 by manually clocking it for 4 cycles while asserting the RESET' pin low.
    uint32_t sleepTime = 1000000 / (z80Frequency * 2);
    if (sleepTime == 0)
    {
        sleepTime = 1;
    }
    for (int i = 0 ; i < 4 ; i++)
    {
        sleep_us(sleepTime);
        gpio_set_mask(clockPinMask);
        sleep_us(sleepTime);
        gpio_clr_mask(clockPinMask);
    }

    // De-assert the RESET' pin.
    gpio_set_mask(resetPinMask);
}

bool Z80_Bus::initClockStateMachine(uint32_t frequency, uint32_t z80ClockPin)
{
    if (initClockStateMachine(pio0, frequency, z80ClockPin) || initClockStateMachine(pio1, frequency, z80ClockPin))
    {
        return true;
    }
    return false;
}

bool Z80_Bus::initClockStateMachine(PIO pio, uint32_t frequency, uint32_t z80ClockPin)
{
    // See if the PIO code for the clocking state machine will fit in this PIO instance with a free state machine.
    if (!pio_can_add_program(pio, &z80_clk_program))
    {
        return false;
    }
    int32_t stateMachine = pio_claim_unused_sm(pio, false);
    if (stateMachine == -1)
    {
        return false;
    }

    // Remember the clock pin along with PIO instance and state machine.
    m_z80ClockPin = z80ClockPin;
    m_clockPIO = pio;
    m_clockStateMachine = stateMachine;

    // Load the PIO program.
    m_clockProgramOffset = pio_add_program(pio, &z80_clk_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80_clk_program_get_default_config(m_clockProgramOffset);

    // Set the Z80 CLK frequency by setting the PIO state machine clock divisor.
    float cpuFrequency = (float)clock_get_hz(clk_sys);
    sm_config_set_clkdiv(&smConfig, cpuFrequency / ((float)frequency * 2.0f));

    // SET pins should be configured for the Z80 clock pin.
    sm_config_set_set_pins(&smConfig, m_z80ClockPin, 1);

    // Complete the state machine configuration.
    pio_sm_init(pio, stateMachine, m_clockProgramOffset, &smConfig);

    // Configure following pins to be outputs controlled by PIO:
    //  Z80 CLK pin should start out set to '0'.
    uint32_t outputPinMask = 1 << m_z80ClockPin;
    pio_sm_set_pins_with_mask(pio, stateMachine, 0, outputPinMask);
    pio_sm_set_pindirs_with_mask(pio, stateMachine, outputPinMask, outputPinMask);
    pio_gpio_init(pio, m_z80ClockPin);

    // Now start the state machine.
    pio_sm_set_enabled(pio, stateMachine, true);

    return true;
}

bool Z80_Bus::initAddressStateMachine(uint32_t frequency, uint32_t waitPin,
                                      uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                      uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                      uint32_t rfshPin)
{
    if (initAddressStateMachine(pio0, frequency, waitPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin, mreqPin, ioreqPin, rfshPin) ||
        initAddressStateMachine(pio1, frequency, waitPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin, mreqPin, ioreqPin, rfshPin))
    {
        return true;
    }
    return false;
}



bool Z80_Bus::initAddressStateMachine(PIO pio, uint32_t frequency, uint32_t waitPin,
                                      uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                      uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                      uint32_t rfshPin)
{
    // See if the PIO code for the address state machine will fit in this PIO instance with a free state machine.
    if (!pio_can_add_program(pio, &z80_mreq_ioreq_program))
    {
        return false;
    }
    int32_t stateMachine = pio_claim_unused_sm(pio, false);
    if (stateMachine == -1)
    {
        return false;
    }

    // Remember the pins along with PIO instance and state machine.
    m_waitPin = waitPin;
    m_shiftClockPin = shiftClockPin;
    m_shiftLatchPin = shiftLatchPin;
    m_shiftEvenPin = shiftEvenPin;
    m_shiftOddPin = shiftOddPin;
    m_mreqPin = mreqPin;
    m_ioreqPin = ioreqPin;
    m_rfshPin = rfshPin;
    m_addressPIO = pio;
    m_addressStateMachine = stateMachine;

    // Load the PIO program.
    m_addressProgramOffset = pio_add_program(pio, &z80_mreq_ioreq_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80_mreq_ioreq_program_get_default_config(m_addressProgramOffset);

    // Side Set needs to be configured for the Shift Clock and Latch pins.
    sm_config_set_sideset_pins(&smConfig, m_shiftClockPin);

    // SET pins should be configured for the WAIT' pin.
    sm_config_set_set_pins(&smConfig, m_waitPin, 1);

    // ISR should be configured for auto push with a threshold of 16+2=18.
    // IOREQ', MREQ', A15...A0
    // Left shift through ISR as IOREQ' and MREQ' are placed in most significant bits and the address shifted in below
    // with msb of address shifted in first.
    const bool autoPush = true;
    const bool shiftRight = true;
    const uint32_t pushThreshold = 16+2;
    sm_config_set_in_shift(&smConfig, !shiftRight, autoPush, pushThreshold);

    // OSR should be configured for no auto pull.
    // Right shift through OSR when used to shift out MREQ' and IOREQ'.
    const bool autoPull = true;
    const uint32_t pullThreshold = 0;
    sm_config_set_out_shift(&smConfig, shiftRight, !autoPull, pullThreshold);

    // INPUT pins should be configured for:
    //  Shift_Even as least significant bit.
    //  Shift_Odd
    //  MREQ'
    //  IOREQ' as most significant bit.
    sm_config_set_in_pins(&smConfig, m_shiftEvenPin);

    // Jump conditional pin should be configured as the Z80 RFSH' pin.
    sm_config_set_jmp_pin(&smConfig, m_rfshPin);

    // Set the shift register clock frequency based on the desired Z80 clock frequency by setting the PIO state machine
    // clock divisor. Run it 8 x 2 x 2 times faster so that it can shift out all 8 bits in one phase of the Z80 clock.
    float cpuFrequency = (float)clock_get_hz(clk_sys);
    sm_config_set_clkdiv(&smConfig, cpuFrequency / ((float)frequency * 2.0f * 8.0f * 2.0f));

    // Complete the state machine configuration.
    pio_sm_init(pio, stateMachine, m_addressProgramOffset, &smConfig);

    // Configure following pins to be outputs controlled by PIO:
    //  Shift Clock pin should start out set to '0'.
    //  Shift Latch pin should start out set to '1'.
    //  WAIT' pin should start out set to '1'.
    uint32_t outputPinMask = (1 << m_shiftClockPin) | (1 << m_shiftLatchPin) | (1 << m_waitPin);
    uint32_t inputPinMask = (1 << m_shiftEvenPin) | (1 << m_shiftOddPin) | (1 << m_mreqPin) | (1 << m_ioreqPin);
    uint32_t highOutputPins = (1 << m_shiftLatchPin) | (1 << m_waitPin);
    pio_sm_set_pins_with_mask(pio, stateMachine, highOutputPins, outputPinMask);
    pio_sm_set_pindirs_with_mask(pio, stateMachine, outputPinMask, outputPinMask | inputPinMask);
    pio_gpio_init(pio, m_shiftClockPin);
    pio_gpio_init(pio, m_shiftLatchPin);
    pio_gpio_init(pio, m_waitPin);

    // Disable the pull-up/down resistors on the Shift_Even/Odd signals.
    gpio_disable_pulls(m_shiftEvenPin);
    gpio_disable_pulls(m_shiftOddPin);

    // Improve the rise/fall times of the shift clock signal by increasing the drive strength to 12mA and switching to
    // the faster slew rate. From my experiments the drive strength has a bigger impact (drops rise/fall times from 8ns
    // to 6ns).
    gpio_set_slew_rate(m_shiftClockPin, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(m_shiftClockPin, GPIO_DRIVE_STRENGTH_12MA);

    // Disable the synchronizers on the Shift_Even/Odd signals as they are synchronized to the Shift_Clock.
    // This should remove the input latency on these pins by 2 clock cycles (2 cycles @ 125MHz == 16ns).
    pio->input_sync_bypass |= (1 << m_shiftEvenPin) | (1 << m_shiftOddPin);

    // Now start the state machine.
    pio_sm_set_enabled(pio, stateMachine, true);

    return true;
}

bool Z80_Bus::initReadWriteStateMachine(uint32_t rdPin, uint32_t wrPin,
                                        uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                        uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                        uint32_t m1Pin)
{
    if (initReadWriteStateMachine(pio0, rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin) ||
        initReadWriteStateMachine(pio1, rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin))
    {
        return true;
    }
    return false;
}

bool Z80_Bus::initReadWriteStateMachine(PIO pio,
                                        uint32_t rdPin, uint32_t wrPin,
                                        uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                        uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                        uint32_t m1Pin)
{
    // See if the PIO code for the read write state machine will fit in this PIO instance with a free state machine.
    if (!pio_can_add_program(pio, &z80_rd_wr_program))
    {
        return false;
    }
    int32_t stateMachine = pio_claim_unused_sm(pio, false);
    if (stateMachine == -1)
    {
        return false;
    }

    // Remember the pins along with PIO instance and state machine.
    m_rdPin = rdPin;
    m_wrPin = wrPin;
    m_d0Pin = d0Pin;
    m_d1Pin = d1Pin;
    m_d2Pin = d2Pin;
    m_d3Pin = d3Pin;
    m_d4Pin = d4Pin;
    m_d5Pin = d5Pin;
    m_d6Pin = d6Pin;
    m_d7Pin = d7Pin;
    m_m1Pin = m1Pin;
    m_readWritePIO = pio;
    m_readWriteStateMachine = stateMachine;

    // Load the PIO program.
    m_readWriteProgramOffset = pio_add_program(pio, &z80_rd_wr_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80_rd_wr_program_get_default_config(m_readWriteProgramOffset);

    // ISR should be configured for auto push with a threshold of 2+8+1=11.
    // RD', WR', D0 - D7, M1'
    // Left shift to through ISR so that the bits are right aligned in the 32-bit value.
    const bool autoPush = true;
    const bool shiftRight = true;
    const uint32_t pushThreshold = 2+8+1;
    sm_config_set_in_shift(&smConfig, !shiftRight, autoPush, pushThreshold);

    // OSR should be configured for no auto pull and write shift.
    const bool autoPull = true;
    const uint32_t pullThreshold = 0;
    sm_config_set_out_shift(&smConfig, shiftRight, !autoPull, pullThreshold);

    // INPUT pins should be configured for:
    // RD' as least significant bit.
    // WR'
    // D0-D7
    // M1',
    // RFSH'
    // HALT' as most significant bit.
    sm_config_set_in_pins(&smConfig, m_rdPin);

    // OUTPUT (PINS and PINDIRS) should be configured for:
    // D0-D7
    sm_config_set_out_pins(&smConfig, m_d0Pin, 8);

    // Complete the state machine configuration.
    pio_sm_init(pio, stateMachine, m_readWriteProgramOffset, &smConfig);

    // D0-D7 should be configured for PIO usage as well though as they are outputs but default to input.
    pio_gpio_init(pio, m_d0Pin);
    pio_gpio_init(pio, m_d1Pin);
    pio_gpio_init(pio, m_d2Pin);
    pio_gpio_init(pio, m_d3Pin);
    pio_gpio_init(pio, m_d4Pin);
    pio_gpio_init(pio, m_d5Pin);
    pio_gpio_init(pio, m_d6Pin);
    pio_gpio_init(pio, m_d7Pin);

    // Now start the state machine.
    pio_sm_set_enabled(pio, stateMachine, true);

    return true;
}

void Z80_Bus::uninit()
{
    if (m_clockPIO == NULL && m_addressPIO == NULL && m_readWritePIO == NULL)
    {
        return;
    }

    // Stop the state machines and free up their resources.
    pio_sm_set_enabled(m_clockPIO, m_clockStateMachine, false);
    pio_sm_unclaim(m_clockPIO, m_clockStateMachine);
    pio_remove_program(m_clockPIO, &z80_clk_program, m_clockProgramOffset);

    pio_sm_set_enabled(m_readWritePIO, m_readWriteStateMachine, false);
    pio_sm_unclaim(m_readWritePIO, m_readWriteStateMachine);
    pio_remove_program(m_readWritePIO, &z80_rd_wr_program, m_readWriteProgramOffset);

    pio_sm_set_enabled(m_addressPIO, m_addressStateMachine, false);
    pio_sm_unclaim(m_addressPIO, m_addressStateMachine);
    pio_remove_program(m_addressPIO, &z80_mreq_ioreq_program, m_addressProgramOffset);

    m_clockPIO = NULL;
    m_addressPIO = NULL;
    m_readWritePIO = NULL;
    m_clockStateMachine = UNKNOWN_VAL;
    m_addressStateMachine = UNKNOWN_VAL;
    m_readWriteStateMachine = UNKNOWN_VAL;
    m_clockProgramOffset = UNKNOWN_VAL;
    m_addressProgramOffset = UNKNOWN_VAL;
    m_readWriteProgramOffset = UNKNOWN_VAL;
}

void Z80_Bus::cacheWaitInstructionOpCodes()
{
    // Cache away the WAIT GPIO instructions that need to be sent to and executed by the state machines now that we
    // know which GPIO pins are associated with MREQ', IOREQ', RD', and WR'.
    m_waitReadInstruction = pio_encode_wait_gpio(true, m_rdPin);
    m_waitWriteInstruction = pio_encode_wait_gpio(true, m_wrPin);
    m_waitMREQInstruction = pio_encode_wait_gpio(true, m_mreqPin) | pio_encode_sideset(2, 0b10);
    m_waitIOREQInstruction = pio_encode_wait_gpio(true, m_ioreqPin) | pio_encode_sideset(2, 0b10);
}



int main()
{
    stdio_init_all();

    // UNDONE: Giving time for serial terminal to reconnect.
    sleep_ms(2000);

    // Configure the PIO state machines to manage the Z80 bus and let the RP2040 core know when memory or I/O requests
    // have been made by the Z80 so that the RP2040 core can handle them.
    printf("Starting up...\n");
    static Z80_Bus z80Bus;
    bool result = z80Bus.init(Z80_FREQUENCY, Z80_RESET_PIN, Z80_CLK_PIN,
                              Z80_MREQ_PIN, Z80_IOREQ_PIN, Z80_RD_PIN, Z80_WR_PIN,
                              Z80_D0_PIN, Z80_D1_PIN, Z80_D2_PIN, Z80_D3_PIN,
                              Z80_D4_PIN, Z80_D5_PIN, Z80_D6_PIN, Z80_D7_PIN,
                              Z80_M1_PIN, Z80_WAIT_PIN, Z80_RFSH_PIN, Z80_HALT_PIN,
                              SHIFT_CLOCK_PIN, SHIFT_LATCH_PIN, SHIFT_EVEN_PIN, SHIFT_ODD_PIN);
    assert ( result );

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

    printf("Z80 should now be running...\n");
    volatile uint16_t expectedCount = 0x0000;
    volatile size_t traceIndex = 0;
    while (true)
    {
        // Fetch next read/write transfer request.
        volatile struct Z80_Bus::Request req;
        z80Bus.getNextRequest((Z80_Bus::Request*)&req);
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
                printf("[%04lX]->%02lX %s\n", req.address, req.value, req.isM1 ? "M1" : "");
            }
        }
        else
        {
            memory[address] = req.value;
            if (Z80_FREQUENCY <= 1000)
            {
                printf("[%04lX]<-%02lX\n", req.address, req.value);
            }
        }
        z80Bus.completeRequest((Z80_Bus::Request*)&req);

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

    return 0;
}
