README.txt
==========

STM32F746G-DISCO LTDC Framebuffer with LittleVGL Example

Configure and build
-------------------

tools/configure.sh stm32f746g-disco/lvgl
make


Configuration
------------

This configuration provides a demo of LVGL with
a) 1 LTDC with 16bpp pixel format and a resolution of 480x272.
b) 1 FT5336 Touch Panel Driver in Single Point and Polling mode.


Loading
-------

st-flash write nuttx.bin 0x8000000


Executing
---------

The ltdc is initialized during boot up.  Interaction with NSH is via the serial
console provided by ST-LINK USB at 115200 8N1 baud.
From the nsh comandline execute the lvgldemo example:

  nsh> lvgldemo

The test will start a LVGL demo and will not terminate.
