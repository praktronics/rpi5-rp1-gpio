# rpi5-rp1-gpio
Tests on using the PCI->RP1 direct register access to use GPIO pins and other peripherals

Shows some simple code for how to use the PCI BAR to
1. Map the RP1 peripheral memory to user space
2. Set the pin properties
3. Use the RIO (registered io) peripheral to blink some LEDs

The way the Pi5/RP1 handles this is VERY different to the BCMXXXX predecessors.
The silicon IP is different, so registers, approach, etc. are all different.