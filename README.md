# SWIM_Programmer
Swim programmer for STM8 based on STM32f103

This code is taken from Sourceforge 
https://sourceforge.net/p/stm32f100tostm8s105swimlibrary/wiki/Home/

Original Readme:

STM32F100 to STM8S105 SWIM library

This project is dead from the beginning. It was a preparation for larger project which didn't happen. Now I found the code and decided to post it here. Perhaps someone will find a use for it.
About

STM32F100 to STM8S105 SWIM library is as it says library (if you call pair '.c' and '.h' a library) to program FLASH memory of STM8S105 using STM32F100 as a programmer.
SWIM (Single wire interface module) is one-wire bi-directional debugging protocol used by old 8-bit processors made by ST.
What you need

I was testing this with two discovery kits from ST:

STM32 Value line discovery kit (st.com/stm32-discovery) fitted with STM32F100 as a programmer
STM8S Discovery kit (st.com/stm8s-discovery) fitted with STM8S105 as a target device
I copied the debugging connection from ST-Link which is part of STM8S discovery kit:

Connection diagram
(See swim_connection.png)
Coincidentally: you need to unsolder SB1 and SB2 on STM8S Discovery and disconnect the on-board ST-Link programmer.
Pins on STM8S side are definite. They are RESET and SWIM debugging pin.
Pins on STM32 side can be moved. In the source code you will find two GPIO pins (for reseting the target and for output on the SWIM line) and one pin (SWIM_IN) connected directly to the timer receiving data.

How does it work
The programmer (STM32) drives SWIM line with GPIO pin. Timing is done with NOP instruction so it may not work on faster MCU. On faster MCU's it might be possible to use timer, but on STM32F100 I had to use one instruction delays for output driving. According to documentation SWIM should work in slower speeds, but in real it needs the right frequency.
When the direction of the line turns, the programmer changes the GPIO pin to high impedance but doesn't use it to capture incoming data. There is timer input named SWIM_IN which samples the input data and stores delays between them through DMA to buffer. Data are then decoded by higher layers.
Third pin used is a simple GPIO pin driving target's RESET. Reset after programming will reboot into the new FLASH program and reinitialize SWIM.

Project Members:
Tomáš Jakubík (admin)

