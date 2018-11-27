LPC1114/302 1-universe ArtNet DMX gateway using uIP for DM9000A
====

This project is a single universe ArtNet v3 DMX gateway based on `LPC1114/302`,
`DM9000A` ethernet controller and uIP library. I've hacked an already existed
hardware from USR IOT and specific the USR-TCP232-24 which is the one in the
following image:

![USR-TCP232-24](http://www.stupid-projects.com/wp-content/uploads/2017/10/USR-TCP232-24-300x300.jpg)


This can be found in ebay or alibaba. It has been replaced by the newest
`USR-TCP232-410` which based on the `TM4C129EKCPD` which is an ARM Cortex-M4
with integrated MAC. Therefore, this make the project a bit obsolete.

Also, in case you need to test it and you don't have a DMX device you can buy a
`DMX512` RGB decoder driver like the `PX24506` which is shown in the picture.

![PX24506](http://www.stupid-projects.com/wp-content/uploads/2017/10/PX24506-300x300.jpg)

You can find a few more info [here](http://www.stupid-projects.com/hacking-an-rs232485-to-eth-board)

The project can be compiled with the free version of Keil compiler.

To write the firmware place the jumber on the `[UPD]` position, reset the power
and use Flash Magic. Then set the jumber to the oposite position and reset again.

The firmware except ArtNet v.3 also supports a custom UDP protocol for custom
UDP clients and the supported commands can be found in the tmf file for the
`br@y's` terminal or even better [CuteCom](https://gitlab.com/cutecom/cutecom).

Enjoy.
