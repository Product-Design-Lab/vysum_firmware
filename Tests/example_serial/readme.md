# Serial echo

Note that the serial event function doesn't work the same on xiao as on atmel IC. Original sketch is faking a ISR effect, and on xiao the function is not attached to the loop...

to use with XIAO BLE sense library, comment out #include <Adafruit_TinyUSB.h>
to use with XIAO nrf sense library, include <Adafruit_TinyUSB.h>.
the Tiny USB library uses usbCDC instead of uart, allowing any baudrate settings to be used on the PC
