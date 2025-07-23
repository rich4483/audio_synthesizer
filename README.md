# audio_synthesizer
Audio Synthesizer based on Atmel microcontroller


Schematic of hardware design:
941-01A6.pdf

Assembly language source code:
941-01x5.asm


I designed the hardware and firmware for this audio synthesizer in 2001. The use of assembly language seems ancient by today’s standards. But at the time, it was common practice in our company to use assembly language targeting Atmel microcontrollers for new hardware projects. We had a well established code library and tools for this development work.

We also used C extensively for some projects, but when working with an 8-bit micro with very limited resources (i.e. only 8kB of flash, and 256 bytes of RAM), it is critical to be as efficient as possible. In this environment, writing assembly code offered many advantages.


