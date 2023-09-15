# Z80 Diversion
Some notes about my current [Z80](https://www.zilog.com/index.php?option=com_product&Itemid=26&task=docs&businessLine=&parent_id=139&familyId=20&productId=Z84C00) diversion.


## Project Purpose
I want to verify some unit tests which exercise all of the instructions supported by the Z80 microprocessor. These tests can be used to verify Z80 emulation. Having the ability to also runs these tests against real Z80 hardware is desireable as it allows verifying the quality of the tests themselves. A few years ago, I did something similar for the ARMv6-M instruction tests that I wrote for my [pinkySim project](https://github.com/adamgreen/pinkySim).


## Reading List
![Build Your Own Z80 Computer Book Cover](photos/20230827-BuildYourOwnZ80ComputerBook.jpg)<br>
It has been around a decade since I last looked at the Z80 so I had to skim through a few Z80 references to refresh my memory of what it takes to interface this little guy to the outside world. These references included:
* [Build Your Own Z80 Computer by Steve Ciarcia](https://en.wikipedia.org/wiki/Build_Your_Own_Z80_Computer) ([Free PDF](http://www.pestingers.net/pdfs/other-computers/build-your-own-z80.pdf)).
* [Z80 CPU User Manual](https://www.zilog.com/docs/z80/UM0080.pdf)
* [Z80 Product Specification](https://www.zilog.com/docs/z80/ps0178.pdf)
* [Other Official Z80 Documents](https://www.zilog.com/index.php?option=com_product&Itemid=26&task=docs&businessLine=1&parent_id=139&familyId=20&productId=Z84C00)

I also skimmed some RP2040 documentation again as well to make sure that I had a good chance of successfully interfacing it to the Z80 before going too far down that path:
* [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
* [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)


## High Level BOM
My current plan is to design and implement a simple Z80 based board that can be used to run and verify unit tests. To that end,this board will include:
* A **Z80 Microprocessor** - I am planning to use a [Z84C0020PEG](https://www.digikey.com/en/products/detail/zilog/Z84C0020PEG/928994) device since it is:
  * CMOS Based
  * 20 MHz capable. I picked the fastest CMOS version of the device available. It can run at 20MHz when powered by a 5V supply but it should be able to run at a lower 3.3V (same as the RP2040) when clocked at lower frequencies (between 1 and 2.5 MHz).
  * 40-pin DIP package. Currently Digikey only has the DIP version of the 20MHz capable part in stock. I would have preferred the LQFP SMD version but it doesn't look like it will be in stock for awhile. It also made early bread boarding possible.<br>![Photo of Z80 IC](photos/20230901-Z80_Photo.jpg)
* A [RP2040 based Pico](https://www.digikey.com/en/products/detail/raspberry-pi/SC0915/13624793). I want to connect this to the Z80 bus and use it to service Z80 memory requests. My plan is to make it work like a hardware debugger for the Z80. Features of the RP2040 that can be leveraged for this project include:
  * USB 1.1 controller and PHY. This can be used for easy connection to a desktop/laptop. The unit tests can be deployed and verified over this USB connection.
  * 264kB SRAM. This can be the backing store for the 64kB of address space supported by the Z80 while leaving lots around for other fun.
  * 2MB of FLASH. Lots of room for the required firmware and storage of any ROM images that we may want to use with the Z80.
  * Programmable I/O. This will be useful for interfacing with the Z80 bus without bit banging everything from the CPU.
  * Dual core Cortex-M0+ cores. Can dedicate a core to just servicing memory read/writes requests from the Z80 as they are detected and pushed to the CPU by the PIO state machines. This way things like USB interrupts don't slow down processing of Z80 memory requests.<br>![Photo of Raspberry Pi Pico](photos/20230901-RaspberryPiPico_RP2040.jpg)
  * The Pico has the required voltage regulation to allow the board to be powered direclty from USB.
* 2 x [SN74HC165 8-bit shift registers](https://www.digikey.com/en/products/detail/texas-instruments/SN74HC165N/376966). These will be used to latch the 16-bit address sent from the Z80 and push it into the RP2040 over 2 serial lines, one for the even bits and the other for the odd bits. I will probably run this chip at 5V and run the serial outputs through a 3/5 voltage divider to make it compatible with the RP2040 while also allowing it to run as close to a 50MHz as possible. It can be run at 3.3V like the RP2040 but its maximum serial clock frequency would decrease. There is a good chance that I will use a faster version of this chip in future revisions.<br>![Photo of 8-bit Shift Register](photos/20230901-8BitShiftRegister.jpg)


## Programmable I/O
I plan to leverage the programmable I/O (PIO) peripheral on the RP2040 for interfacing to the Z80 bus. My current thoughts are to use 2 PIO state machines for this interfacing:
* One state machine will generate the main **CLK** signal, sample the **MREQ'** / **IOREQ'** signals and shift out the 16-bit address bits using two 8-bit shift registers:
  * If it sees either of the **MREQ'** or **IOREQ'** signals go active (pulled low) then it will latch the 16-bit address bits and start shifting out the address, two bits at a time.
  * Once the 16-bit address has been fully shifted in, it will be pushed to the TX FIFO for the CPU to pick up.
  * It can also issue an IRQ to the second state machine to unblock it as a read/write request is coming soon.
* The other state machine samples the **RD'**/**WR'** signals to determine if a read or write memory transfer is being requested by the Z80:
  * It can block on an IRQ until the first state machine detects an incoming transfer request and sets the matching IRQ.
  * It then keeps sampling the **RD'**/**WR'** signals until it sees one of them go active (pulled low).
  * If a read has been requested then the **WAIT'** signal can be asserted to let the Z80 know that it is waiting for the RP2040 to access the requested data.
  * The state of the **RD'**/**WR'** lines and **D0** - **D7** are then transferred to the CPU so that between this data and the 16-bit address from the other state machine, the CPU has enough information to answer the memory request.
  * The state machine will then wait for the CPU to send back the required response. This will include bits needed for setting **D0**-**D7** to the correct input/output state, the requested byte if a read request was made, bits to set **D0**-**D7** back to input only, and a PIO instruction that either waits on the **RD'** or **WR'** signal being de-asserted.
  * It also de-asserts the **WAIT'** signal once the request has been completed.


## Current Pin Map
The PIO state machines place constraints on some signals as they will need to be mapped to consecutively numbered GPIO pins so that they can be shifted in/out as needed. This is how the pins are currently mapped between the RP2040 and Z80:
| RP2040 | Z80 / Shift | PIO Pin Group    | Notes |
|--------|-------------|------------------|-------|
| GP0  | Shift_In_Even | Input            | |
| GP1  | Shift_In_Odd  | Input            | |
| GP2  | MREQ'         | Input            | |
| GP3  | IOREQ'        | Input            | |
| GP4  | CLK           | Set              | |
| GP5  | Shift_Clock   | Side Set         | |
| GP6  | Shift_Latch   | Side Set         | |
| GP7  | RD'           | Input            | |
| GP8  | WR'           | Input            | |
| GP9  | D0            | Input/Output     | |
| GP10 | D1            | Input/Output     | |
| GP11 | D2            | Input/Output     | |
| GP12 | D3            | Input/Output     | |
| GP13 | D4            | Input/Output     | |
| GP14 | D5            | Input/Output     | |
| GP15 | D6            | Input/Output     | |
| GP16 | D7            | Input/Output     | |
| GP17 | M1'           | Input            | |
| GP18 | WAIT'         | Side Set         | 2.2kΩ Pull-Up |
| GP19 | RFSH'         | Jmp Condition    | |
| GP20 | HALT'         | Input / Non-PIO  | |
| GP21 | NMI'          | Output / Non-PIO | 2.2kΩ Pull-Up |
| GP22 | INT'          | Output / Non-PIO | 2.2kΩ Pull-Up |
| GP26 | RESET'        | Output / Non-PIO | 3 CLK Long Pulse (3µs @ 1MHz) |
| N/C  | BUSREQ'       | Not Connected    | 2.2kΩ Pull-Up |


## Updated Schematic
![Updated Schematic](photos/20230902-Schematic.jpg)<br>
After wiring up this circuit on a breadboard, I found that things worked better if I made some changes to my original schematic:
* I originally thought it would work best to shift in the Z80 address bit pairs in from least significant to most significant. This results in the CPU receiving the address in the upper 16 bits and the **MREQ'** and **IOREQ'** signals values in bits 14-15. It would be nicer if the address bits were in the lower 16-bits where they can be easily masked off and used. Doing the shifts in this order required me reversing the order of the Z80 address bit connections to each of the shift registers.
* Added 0.1µF bypass capacitors to the IC power supply lines.
* I discovered that the SN74HC165 shift registers didn't have output rise and fall times that were consistent with running them at 50MHz even though I was supplying them with 5V.
  * The high resistance voltage divider I had in the original schematic to level shift this output made the problem worse.
  * Reducing the voltage divider from 55kΩ to 550Ω did shorten the rise and fall times but not enough to work at 50MHz. It does work up to 10MHz now though.
  * If I switch to SMD parts then I think there are variants of the 74*165 shift registers that can run at 3.3V and give the required <= 10ns rise/fall times. I may try this in the future but I will just use the parts that I already have for now.
* I also wanted to test that the Z80 runs reliably at 3.3V when run at lower clock rates. The Z80 hasn't hit any issues running NOPs at 400kHz. I haven't been able to run it any faster due to the shift register issues mentioned previously.
* I removed the 330Ω pull-up resistor from the **CLK** signal. It was needed for old TTL clocking circuits used with the older NMOS version of the Z80. I am using a CMOS version of the Z80 and driving **CLK** directly from a pin of the CMOS based RP2040 microcontroller so it isn't needed anymore.


## GDB for Z80?
I have past experience with using GDB and GDB debug stubs to debug Cortex-M microcontrollers. Can I leverage this experience by using GDB with the Z80? If so then I could use it for deploying test code to the Z80, single stepping, examining memory and registers, etc. When I tested my ARMv6-M instruction tests on real Cortex-M hardware, I used an existing GDB debug stub and modified my test harness to pretend to be GDB so that it could place required inputs and code into RAM, setup the registers, single step over the test instruction, and then interrogate memory and registers afterwards to verify that the expected side effects occurred.

Can I do the same for the Z80? I found a page on the web which gives me a lot of hope that it can:
* [chciken's TLMBoy: Implementing the GDB Remote Serial Protocol](https://www.chciken.com/tlmboy/2022/04/03/gdb-z80.html)


## Current Project State
The Z80, RP2040 Pico, and SN74HC165 shift register devices have arrived from Digikey and I have completed the initial schematic. I also wired it up on my breadboard.

![Photo of Breadboard Setup](photos/20230908-Breadboard.jpg)

The screenshot below shows some early traces from my logic analyzer as I began to get the Z80 successfully starting up and executing NOPs supplied by the RP2040.

![Initial Trace](photos/20230908-AnalyzerTrace.png)

## Next Steps
* Use KiCAD to design the required PCB.
* Order PCBs from [OSH Park](https://oshpark.com).

