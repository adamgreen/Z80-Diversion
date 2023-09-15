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
static const uint32_t Z80_FREQUENCY = 1000;


#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <z80_bus.pio.h>

// Class used to interface to the Z80 microprocessor bus.
class Z80_Bus
{
    protected:
        // The critical path in the PIO code takes this many state machine cycles for a single Z80 clock cycle.
        const uint32_t STATE_MACHINE_CYCLES_PER_Z80_CYCLE = 26 * 2;

    public:
        // Constructor just sets up object. Need to call init() methods to actually initialize the PIO state machine.
        Z80_Bus()
        {
        }

        virtual ~Z80_Bus()
        {
            uninit();
        }

        // Call this init() method to load the assembly language code into the caller specified PIO (pio0 or pio1).
        //
        // pio - The PIO instance to be used, pio0 or pio1.
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
        // Returns false if the assembly language code fails to load into the specified PIO instance.
        bool init(PIO pio, uint32_t frequency,
                  uint32_t resetPin, uint32_t z80ClockPin,
                  uint32_t mreqPin, uint32_t ioreqPin,
                  uint32_t rdPin, uint32_t wrPin,
                  uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                  uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                  uint32_t m1Pin, uint32_t waitPin, uint32_t rfshPin, uint32_t haltPin,
                  uint32_t shiftClockPin, uint32_t shiftLatchPin, uint32_t shiftEvenPin, uint32_t shiftOddPin);

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

        // UNDONE: Document
        uint32_t readAddressBits()
        {
            restartDmaBeforeItStops();
            return m_latestAddress;
        }
        uint32_t readDataBits()
        {
            return pio_sm_get_blocking(m_pio, m_stateMachine2);
        }
        void writeDataBits(uint32_t dataBits)
        {
            pio_sm_put_blocking(m_pio, m_stateMachine2, dataBits);
        }

    protected:
        enum { DMA_MAX_TRANSFER_COUNT = 0xFFFFFFFF, DMA_REFRESH_THRESHOLD = 0x80000000 };

        void resetZ80(uint32_t z80ResetPin, uint32_t z80ClockPin, uint32_t z80Frequency);
        void initClockingStateMachine(PIO pio, int32_t stateMachine, uint32_t dmaChannel, uint32_t frequency,
                                      uint32_t z80ClockPin,
                                      uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                      uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                      uint32_t rfshPin);
        void initReadWriteStateMachine(PIO pio, int32_t stateMachine,
                                       uint32_t waitPin,
                                       uint32_t rdPin, uint32_t wrPin,
                                       uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                       uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                       uint32_t m1Pin);

        // Can only queue up 0xFFFFFFFF DMA transfers at a time. Every once in awhile we will want to reset the
        // transfer count back to 0xFFFFFFFF so that it doesn't stop pulling the latest encoder counts from the PIO.
        inline void restartDmaBeforeItStops()
        {
            uint32_t dmaTransfersLeft = dma_channel_hw_addr(m_dmaChannel)->transfer_count;
            if (dmaTransfersLeft > DMA_REFRESH_THRESHOLD)
            {
                // There are still a lot of transfers left in this DMA operation so just return.
                return;
            }

            // Stopping the DMA channel and starting it again will cause it to use all of the original settings,
            // including the 0xFFFFFFFF transfer count.
            dma_channel_abort(m_dmaChannel);
            dma_channel_start(m_dmaChannel);
        }

        // Unknown values can be set to this.
        const uint32_t UNKNOWN_VAL = 0xFFFFFFFF;

        PIO             m_pio = NULL;
        uint32_t        m_stateMachine1 = UNKNOWN_VAL;
        uint32_t        m_stateMachine2 = UNKNOWN_VAL;
        uint32_t        m_program1Offset = UNKNOWN_VAL;
        uint32_t        m_program2Offset = UNKNOWN_VAL;
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
        // Latest 16-bit address requested by Z80 along with the MREQ'/IOREQ' signal values.
        volatile uint32_t   m_latestAddress = 0;
        // DMA channel used to read address from PIO state machine and place in m_latestAddress;
        uint32_t        m_dmaChannel = 0;
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
    if (init(pio0, frequency, resetPin, z80ClockPin, mreqPin, ioreqPin, rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin, waitPin, rfshPin, haltPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin) ||
        init(pio1, frequency, resetPin, z80ClockPin, mreqPin, ioreqPin, rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin, waitPin, rfshPin, haltPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin))
    {
        return true;
    }
    return false;
}

bool Z80_Bus::init(PIO pio, uint32_t frequency,
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

    // Create a program structure to represent both state machine programs together to make sure that they will both
    // fit in the same PIO instance.
    pio_program mergedProgramInfo =
    {
        .instructions = NULL,
        .length = (uint8_t)(z80_clk_mreq_ioreq_program.length + z80_rd_wr_program.length),
        .origin = -1
    };
    // The 2 programs together can't be larger than what will fit in a single PIO instance.
    assert ( mergedProgramInfo.length <= 32 );
    if (!pio_can_add_program(pio, &mergedProgramInfo))
    {
        return false;
    }

    // See if there are 2 consecutive state machines available.
    const bool panicIfNotAvailable = true;
    int32_t stateMachine1 = pio_claim_unused_sm(pio, !panicIfNotAvailable);
    int32_t stateMachine2 = pio_claim_unused_sm(pio, !panicIfNotAvailable);
    if (stateMachine2 != stateMachine1 + 1)
    {
        // No consecutive state machines so return a failure code.
        return false;
    }

    // Attempt to claim a DMA channel to be used for updating m_latestAddress in the background.
    int dmaChannel = dma_claim_unused_channel(false);
    if (dmaChannel < 0)
    {
        // No free DMA channels so return a failure code.
        return -1;
    }

    resetZ80(resetPin, z80ClockPin, frequency);

    // Make sure that the IRQ shared between the 2 state machines is cleared.
    // NOTE: The IRQ used has the same index as the second state machine.
    assert ( pio_interrupt_get(pio, stateMachine2) == 0 );
    pio_interrupt_clear(pio, stateMachine2);

    // Configure and start the 2 state machines.
    // Start the read/write state machine first so that it is ready for the IRQ that will be set by the clocking
    // state machine.
    initReadWriteStateMachine(pio, stateMachine2, waitPin, rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin);
    initClockingStateMachine(pio, stateMachine1, dmaChannel, frequency, z80ClockPin, shiftClockPin, shiftLatchPin, shiftEvenPin, shiftOddPin, mreqPin, ioreqPin, rfshPin);

    m_pio = pio;
    m_stateMachine1 = stateMachine1;
    m_stateMachine2 = stateMachine2;
    m_dmaChannel = dmaChannel;
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

void Z80_Bus::initClockingStateMachine(PIO pio, int32_t stateMachine, uint32_t dmaChannel, uint32_t frequency,
                                       uint32_t z80ClockPin,
                                       uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                       uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                       uint32_t rfshPin)
{
    // Remember the pins.
    m_z80ClockPin = z80ClockPin;
    m_shiftClockPin = shiftClockPin;
    m_shiftLatchPin = shiftLatchPin;
    m_shiftEvenPin = shiftEvenPin;
    m_shiftOddPin = shiftOddPin;
    m_mreqPin = mreqPin;
    m_ioreqPin = ioreqPin;
    m_rfshPin = rfshPin;

    // Load the PIO program.
    m_program1Offset = pio_add_program(pio, &z80_clk_mreq_ioreq_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80_clk_mreq_ioreq_program_get_default_config(m_program1Offset);

    // Side Set needs to be configured for the Shift Clock and Latch pins.
    sm_config_set_sideset_pins(&smConfig, m_shiftClockPin);

    // SET pins should be configured for the Z80 clock pin.
    sm_config_set_set_pins(&smConfig, m_z80ClockPin, 1);

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

    // Set the Z80 CLK frequency by setting the PIO state machine clock divisor.
    float cpuFrequency = (float)clock_get_hz(clk_sys);
    sm_config_set_clkdiv(&smConfig, cpuFrequency / ((float)frequency * (float)STATE_MACHINE_CYCLES_PER_Z80_CYCLE));

    // Complete the state machine configuration.
    pio_sm_init(pio, stateMachine, m_program1Offset, &smConfig);

    // Configure following pins to be outputs controlled by PIO:
    //  Z80 CLK pin should start out set to '0'.
    //  Shift Clock pin should start out set to '0'.
    //  Shift Latch pin should start out set to '1'.
    uint32_t outputPinMask = (1 << m_z80ClockPin) | (1 << m_shiftClockPin) | (1 << m_shiftLatchPin);
    uint32_t inputPinMask = (1 << m_shiftEvenPin) | (1 << m_shiftOddPin) | (1 << m_mreqPin) | (1 << m_ioreqPin);
    uint32_t highOutputPins = (1 << m_shiftLatchPin);
    pio_sm_set_pins_with_mask(pio, stateMachine, highOutputPins, outputPinMask);
    pio_sm_set_pindirs_with_mask(pio, stateMachine, outputPinMask, outputPinMask | inputPinMask);
    pio_gpio_init(pio, m_z80ClockPin);
    pio_gpio_init(pio, m_shiftClockPin);
    pio_gpio_init(pio, m_shiftLatchPin);

    // Disable the pull-up/down resistors on the Shift_Even/Odd signals.
    gpio_disable_pulls(m_shiftEvenPin);
    gpio_disable_pulls(m_shiftOddPin);

    // Disable the synchronizers on the Shift_Even/Odd signals as they are synchronized to the Shift_Clock.
    // This should remove the input latency on these pins by 2 clock cycles (2 cycles @ 125MHz == 16ns).
    m_pio->input_sync_bypass |= (1 << m_shiftEvenPin) | (1 << m_shiftOddPin);

    // Configure a DMA channel to read the latest address values from this PIO state machine and copy it into
    // m_latestAddress.
    dma_channel_config dmaConfig = dma_channel_get_default_config(dmaChannel);
    channel_config_set_read_increment(&dmaConfig, false);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_dreq(&dmaConfig, pio_get_dreq(pio, stateMachine, false));

    dma_channel_configure(dmaChannel, &dmaConfig,
        &m_latestAddress,               // Destination pointer
        &pio->rxf[stateMachine],        // Source pointer
        DMA_MAX_TRANSFER_COUNT,         // Largest possible number of transfers
        true                            // Start immediately
    );

    // Now start the state machine.
    pio_sm_set_enabled(pio, stateMachine, true);
}

void Z80_Bus::initReadWriteStateMachine(PIO pio, int32_t stateMachine,
                                        uint32_t waitPin,
                                        uint32_t rdPin, uint32_t wrPin,
                                        uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                        uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                        uint32_t m1Pin)
{
    // Remember the pins.
    m_waitPin = waitPin;
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

    // Load the PIO program.
    m_program2Offset = pio_add_program(pio, &z80_rd_wr_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80_rd_wr_program_get_default_config(m_program2Offset);

    // Side Set needs to be configured for the Z80 WAIT' pin.
    sm_config_set_sideset_pins(&smConfig, m_waitPin);

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
    pio_sm_init(pio, stateMachine, m_program2Offset, &smConfig);

    // Initialize the Y register to have a value of 0b01 to be compared with value of RD and WR pins.
    pio_sm_exec_wait_blocking(pio, stateMachine, pio_encode_set(pio_y, 1));

    // WAIT' pin should start out set to '1'.
    // D0-D7 should be configured for PIO usage as well though as they are outputs but default to input.
    uint32_t waitPinMask = 1 << m_waitPin;
    pio_sm_set_pins_with_mask(pio, stateMachine, waitPinMask, waitPinMask);
    pio_sm_set_pindirs_with_mask(pio, stateMachine, waitPinMask, waitPinMask);
    pio_gpio_init(pio, m_waitPin);
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
}

void Z80_Bus::uninit()
{
    if (m_pio == NULL)
    {
        return;
    }

    // Stop the DMA channel used to read out the address information and free it up.
    dma_channel_abort(m_dmaChannel);
    dma_channel_unclaim(m_dmaChannel);

    // Stop the state machine and free it up.
    pio_sm_set_enabled(m_pio, m_stateMachine1, false);
    pio_sm_unclaim(m_pio, m_stateMachine1);
    pio_sm_set_enabled(m_pio, m_stateMachine2, false);
    pio_sm_unclaim(m_pio, m_stateMachine2);

    // Free up the space in the PIO instance.
    pio_remove_program(m_pio, &z80_clk_mreq_ioreq_program, m_program1Offset);
    pio_remove_program(m_pio, &z80_rd_wr_program, m_program2Offset);

    m_stateMachine1 = UNKNOWN_VAL;
    m_stateMachine2 = UNKNOWN_VAL;
    m_pio = NULL;
    m_program1Offset = UNKNOWN_VAL;
    m_program2Offset = UNKNOWN_VAL;
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

    printf("Z80 should now be running...\n");
    const uint32_t pinDirOutput = 0xFF;
    const uint32_t pinDirInput = 0x00;
    const uint32_t waitReadInstruction = pio_encode_wait_gpio(true, Z80_RD_PIN) | pio_encode_sideset(1, 1);
    const uint32_t waitWriteInstruction = pio_encode_wait_gpio(true, Z80_WR_PIN) | pio_encode_sideset(1, 1);
    uint16_t expectedCount = 0x0000;
    while (true)
    {
        // Fetch next read/write transfer request.
        uint32_t dataInBits = z80Bus.readDataBits();
        bool isRead = !(dataInBits & 0x1);
        bool isWrite = !(dataInBits & 0x2);
        uint8_t dataIn = (dataInBits >> 2) & 0xFF;

        // Read in latest address which corresponds to the current read/write transfer request.
        uint32_t addressBits = z80Bus.readAddressBits();
        uint32_t address = addressBits & 0xFFFF;
        bool isMReq = !!(addressBits & 0x10000);
        bool isIoReq = !!(addressBits & 0x20000);
        assert ( address < sizeof(memory) );
        assert ( isMReq && !isIoReq );

        // Perform the memory read or write from/to RAM and send response to Z80 via PIO.
        uint32_t pinDirsAndData;
        uint32_t pioInstructionAndPinDirs;
        if (isRead)
        {
            uint32_t memByte = memory[address];
            pinDirsAndData = memByte << 8 | pinDirOutput;
            pioInstructionAndPinDirs = (pinDirInput << 16) | waitReadInstruction;
            if (Z80_FREQUENCY <= 1000)
            {
                printf("[%04lX]->%02lX\n", address, memByte);
            }
        }
        else
        {
            assert ( isWrite );
            memory[address] = dataIn;
            pinDirsAndData = 0x00 << 8 | pinDirInput;
            pioInstructionAndPinDirs = (pinDirInput << 16) | waitWriteInstruction;
            if (Z80_FREQUENCY <= 1000)
            {
                printf("[%04lX]<-%02X\n", address, dataIn);
            }
        }
        z80Bus.writeDataBits(pinDirsAndData);
        z80Bus.writeDataBits(pioInstructionAndPinDirs);

        // Check the 16-bit counter on each iteration of the loop.
        if (isRead && address == 0x0000)
        {
            uint16_t counter;
            memcpy(&counter, &memory[0xA], sizeof(counter));
            assert ( counter == expectedCount++ );
        }
    }

    return 0;
}
