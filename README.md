# LPC1114-ArtNet-uIP
LPC1114/302 1-universe ArtNet DMX gateway using uIP for DM9000A

This project is a single universe ArtNet v3 DMX gateway based on LPC1114/302, DM9000A ethernet controller and uIP library.
I've used an already existed hardware from USR IOT and specific the USR-TCP232-24.
It can be found here (http://en.usr.cn/Ethernet-Module-T24/RS223-RS485-serial-to-TCP-IP-ethernet-server-module-converter.html)
and also is sold in e-bay very cheap.

The project can be compiled with the free version of Keil compiler. 
Soon, it will be available also for Coocox compiler.

To write the firmware place the jumber on the [UPD] position, reset the power and use Flash Magic.
Then set the jumber to the oposite position and reset again.

The firmware except ArtNet v.3 also supports a custom UDP protocol for custom UDP clients and the supported
commands can be found in the tmf file for the br@y's terminal.

Enjoy.
