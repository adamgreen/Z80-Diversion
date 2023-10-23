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
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include <Z80Bus.pio.h>
#include "Z80Bus.h"

bool Z80Bus::init(uint32_t frequency,
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
        !initReadWriteStateMachine(rdPin, wrPin, d0Pin, d1Pin, d2Pin, d3Pin, d4Pin, d5Pin, d6Pin, d7Pin, m1Pin))
    {
        uninit();
        return false;
    }
    initClock(frequency, z80ClockPin);
    cacheWaitInstructionOpCodes();

    m_haltPin = haltPin;

    return true;
}

void Z80Bus::resetZ80(uint32_t z80ResetPin, uint32_t z80ClockPin, uint32_t z80Frequency)
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

bool Z80Bus::initAddressStateMachine(uint32_t frequency, uint32_t waitPin,
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



bool Z80Bus::initAddressStateMachine(PIO pio, uint32_t frequency, uint32_t waitPin,
                                      uint32_t shiftClockPin, uint32_t shiftLatchPin,
                                      uint32_t shiftEvenPin, uint32_t shiftOddPin, uint32_t mreqPin, uint32_t ioreqPin,
                                      uint32_t rfshPin)
{
    // See if the PIO code for the address state machine will fit in this PIO instance with a free state machine.
    if (!pio_can_add_program(pio, &z80AddressStateMachine_program))
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
    m_addressProgramOffset = pio_add_program(pio, &z80AddressStateMachine_program);

    // Update the "wait 1 gpio rfshPin side 0b10" instruction at the beginning of this state machine's instruction
    // stream as rfshPin is not provided to the PIO assembler.
    m_addressPIO->instr_mem[m_addressProgramOffset + z80AddressStateMachine_offset_wait_instr] =
        pio_encode_wait_gpio(true, m_rfshPin) | pio_encode_sideset(2, 0b10);
    m_waitMREQInstruction = pio_encode_wait_gpio(true, m_mreqPin) | pio_encode_sideset(2, 0b10);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80AddressStateMachine_program_get_default_config(m_addressProgramOffset);

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

bool Z80Bus::initReadWriteStateMachine(uint32_t rdPin, uint32_t wrPin,
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

bool Z80Bus::initReadWriteStateMachine(PIO pio,
                                        uint32_t rdPin, uint32_t wrPin,
                                        uint32_t d0Pin, uint32_t d1Pin, uint32_t d2Pin, uint32_t d3Pin,
                                        uint32_t d4Pin, uint32_t d5Pin, uint32_t d6Pin, uint32_t d7Pin,
                                        uint32_t m1Pin)
{
    // See if the PIO code for the read write state machine will fit in this PIO instance with a free state machine.
    if (!pio_can_add_program(pio, &z80ReadWriteStateMachine_program))
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
    m_readWriteProgramOffset = pio_add_program(pio, &z80ReadWriteStateMachine_program);

    // Fetch the default state machine configuration for running this PIO program on a state machine.
    pio_sm_config smConfig = z80ReadWriteStateMachine_program_get_default_config(m_readWriteProgramOffset);

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

void Z80Bus::initClock(uint32_t frequency, uint32_t z80ClockPin)
{
    uint32_t pwmSlice = pwm_gpio_to_slice_num(z80ClockPin);
    uint32_t pwmChannel = pwm_gpio_to_channel(z80ClockPin);
    pwm_config config = pwm_get_default_config();
    uint32_t cpuFrequency = clock_get_hz(clk_sys);
    uint32_t minFrequency = cpuFrequency / 125;

    uint16_t top;
    if (frequency * 2 >= minFrequency)
    {
        pwm_config_set_clkdiv(&config, (float)cpuFrequency / ((float)frequency * 2.0f));
        top = 2;
    }
    else
    {
        pwm_config_set_clkdiv(&config, 125.0f);
        top = ((float)cpuFrequency / 125.0f) / (float)frequency + 0.5f;
    }
    assert ( top > 1 );

    pwm_config_set_wrap(&config, top - 1);
    gpio_set_function(z80ClockPin, GPIO_FUNC_PWM);
    pwm_init(pwmSlice, &config, true);
    pwm_set_chan_level(pwmSlice, pwmChannel, top / 2);

    // Remember the clock pin.
    m_z80ClockPin = z80ClockPin;
}

void Z80Bus::uninit()
{
    // Stop the state machines and free up their resources.
    if (m_readWritePIO != NULL)
    {
        pio_sm_set_enabled(m_readWritePIO, m_readWriteStateMachine, false);
        pio_sm_unclaim(m_readWritePIO, m_readWriteStateMachine);
        pio_remove_program(m_readWritePIO, &z80ReadWriteStateMachine_program, m_readWriteProgramOffset);
        m_readWritePIO = NULL;
        m_readWriteStateMachine = UNKNOWN_VAL;
        m_readWriteProgramOffset = UNKNOWN_VAL;
    }
    if (m_addressPIO != NULL)
    {
        pio_sm_set_enabled(m_addressPIO, m_addressStateMachine, false);
        pio_sm_unclaim(m_addressPIO, m_addressStateMachine);
        pio_remove_program(m_addressPIO, &z80AddressStateMachine_program, m_addressProgramOffset);
        m_addressPIO = NULL;
        m_addressStateMachine = UNKNOWN_VAL;
        m_addressProgramOffset = UNKNOWN_VAL;
    }
}

void Z80Bus::cacheWaitInstructionOpCodes()
{
    // Cache away the WAIT GPIO instructions that need to be sent to and executed by the state machines now that we
    // know which GPIO pins are associated with MREQ', IOREQ', RD', and WR'.
    m_waitReadInstruction = pio_encode_wait_gpio(true, m_rdPin);
    m_waitWriteInstruction = pio_encode_wait_gpio(true, m_wrPin);
    m_waitMREQInstruction = pio_encode_wait_gpio(true, m_mreqPin) | pio_encode_sideset(2, 0b10);
    m_waitIOREQInstruction = pio_encode_wait_gpio(true, m_ioreqPin) | pio_encode_sideset(2, 0b10);
}
