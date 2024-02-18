/*
    GPIO access on RP1 through PCI BAR1
    2024 Feb
    Praktronics
    GPL3
    

    run with sudo or as root for permissions to access to /dev/mem
    this file is self-contained and should compile with gcc -o rpi5-rp1-gpio rpi5-rp1-gpio.c

    ~/rpi5-rp1-gpio $ gcc -o rpi5-rp1-gpio rpi5-rp1-gpio.c
    ~/rpi5-rp1-gpio $ sudo ./rpi5-rp1-gpio 

*/


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>

void delay_ms(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// pci bar info
// from: https://github.com/G33KatWork/RP1-Reverse-Engineering/blob/master/pcie/hacks.py
#define RP1_BAR1 0x1f00000000
#define RP1_BAR1_LEN 0x400000

// offsets from include/dt-bindings/mfd/rp1.h
// https://github.com/raspberrypi/linux/blob/rpi-6.1.y/include/dt-bindings/mfd/rp1.h
#define RP1_IO_BANK0_BASE 0x0d0000
#define RP1_RIO0_BASE 0x0e0000
#define RP1_PADS_BANK0_BASE 0x0f0000

// the following info is from the RP1 datasheet (draft & incomplete as of 2024-02-18)
// https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
#define RP1_ATOM_XOR_OFFSET 0x1000
#define RP1_ATOM_SET_OFFSET 0x2000
#define RP1_ATOM_CLR_OFFSET 0x3000

#define PADS_BANK0_VOLTAGE_SELECT_OFFSET 0
#define PADS_BANK0_GPIO_OFFSET 0x4

#define RIO_OUT_OFFSET 0x00
#define RIO_OE_OFFSET 0x04
#define RIO_NOSYNC_IN_OFFSET 0x08
#define RIO_SYNC_IN_OFFSET 0x0C
//                           3         2         1
//                          10987654321098765432109876543210
#define CTRL_MASK_FUNCSEL 0b00000000000000000000000000011111
#define PADS_MASK_OUTPUT  0b00000000000000000000000011000000

#define CTRL_FUNCSEL_RIO 0x05


void *mapgpio(off_t dev_base, off_t dev_size)
{
    int fd;
    void *mapped;
    
    printf("sizeof(off_t) %d\n", sizeof(off_t));

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        printf("Can't open /dev/mem\n");
        return (void *)0;
    }

    mapped = mmap(0, dev_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base);
    // close(fd);

    printf("base address: %llx, size: %x, mapped: %p\n", dev_base, dev_size, mapped);

    if (mapped == (void *)-1)
    {
        printf("Can't map the memory to user space.\n");
        return (void *)0;
    }

    return mapped;
}


typedef struct
{
    uint8_t number;
    volatile uint32_t *status;
    volatile uint32_t *ctrl;
    volatile uint32_t *pad;
} gpio_pin_t;

typedef struct
{
    volatile void *rp1_peripherial_base;
    volatile void *gpio_base;
    volatile void *pads_base;
    volatile uint32_t *rio_out;
    volatile uint32_t *rio_output_enable;
    volatile uint32_t *rio_nosync_in;

    gpio_pin_t *pins[27];

} rp1_t;

bool create_rp1(rp1_t **rp1)
{

    rp1_t *r = (rp1_t *)calloc(1, sizeof(rp1_t));
    if (r == NULL)
        return false;

    void *base = mapgpio(RP1_BAR1, RP1_BAR1_LEN);
    if (base == NULL)
        return false;

    r->rp1_peripherial_base = base;
    r->gpio_base = base + RP1_IO_BANK0_BASE;
    r->pads_base = base + RP1_PADS_BANK0_BASE;
    r->rio_out = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OUT_OFFSET);
    r->rio_output_enable = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_OE_OFFSET);
    r->rio_nosync_in = (volatile uint32_t *)(base + RP1_RIO0_BASE + RIO_NOSYNC_IN_OFFSET);

    *rp1 = r;

    return true;
}

bool create_pin(uint8_t pinnumber, rp1_t *rp1)
{
    gpio_pin_t *newpin = calloc(1, sizeof(gpio_pin_t));
    if(newpin == NULL) return false;

    newpin->number = pinnumber;

    // each gpio has a status and control register
    // adjacent to each other. control = status + 4 (uint8_t)
    newpin->status = (uint32_t *)(rp1->gpio_base + 8 * pinnumber);
    newpin->ctrl = (uint32_t *)(rp1->gpio_base + 8 * pinnumber + 4);
    newpin->pad = (uint32_t *)(rp1->pads_base + PADS_BANK0_GPIO_OFFSET + pinnumber * 4);

    // set the function
    *(newpin->ctrl + RP1_ATOM_CLR_OFFSET / 4) = CTRL_MASK_FUNCSEL; // first clear the bits
    *(newpin->ctrl + RP1_ATOM_SET_OFFSET / 4) = CTRL_FUNCSEL_RIO;  // now set the value we need

    rp1->pins[pinnumber] = newpin;
    printf("pin %d stored in pins array %p\n", pinnumber, rp1->pins[pinnumber]);

    return true;
}

int pin_enable_output(uint8_t pinnumber, rp1_t *rp1)
{

    printf("Attempting to enable output\n");
    //printf("rp1 @ %p\n", rp1);
    //printf("pin @ %p\n", rp1->pins[pinnumber]);

    //gpio_pin_t *pin = NULL;
    //printf("pin: %p\n", pin);

    //pin = rp1->pins[pinnumber];

    //printf("pin: %p\n", pin);
    //printf("pin pads %p\n", pin->pad);
    //printf("oe: %p\n", rp1->rio_output_enable);

    // first enable the pad to output
    // pads needs to have OD[7] -> 0 (don't disable output)
    // and                IE[6] -> 0 (don't enable input)
    // we use atomic access to the bit clearing alias with a mask
    // divide the offset by 4 since we're doing uint32* math

    volatile uint32_t *writeadd = rp1->pins[pinnumber]->pad + RP1_ATOM_CLR_OFFSET / 4;

    printf("attempting write for %p at %p\n", rp1->pins[pinnumber]->pad, writeadd);

    *writeadd = PADS_MASK_OUTPUT;

    // now set the RIO output enable using the atomic set alias
    *(rp1->rio_output_enable + RP1_ATOM_SET_OFFSET / 4) = 1 << rp1->pins[pinnumber]->number;

    return 0;
}

void pin_on(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_SET_OFFSET / 4) = 1 << pin;
}
void pin_off(rp1_t *rp1, uint8_t pin)
{
    *(rp1->rio_out + RP1_ATOM_CLR_OFFSET / 4) = 1 << pin;
}


const uint8_t pins[] = {17, 27, 22, 23};

int main(void)
{

    int i, j;
    // create a rp1 device
    printf("creating rp1\n");
    rp1_t *rp1;
    if (!create_rp1(&rp1))
    {
        printf("unable to create rp1\n");
        return 2;
    }

    // let's see if we can dump the iobank registers
    uint32_t *p;

    for (i = 0; i < 27; i++)
    {

        p = (uint32_t *)(rp1->gpio_base + i * 8);

        printf(
            "gpio %0d: status @ p = %lx, ctrl @ p = %lx\n",
            i,
            *p, *(p + 1));
    }

    printf("creating pins\n");
    clock_t start = clock();

    for(i=0;i<4;i++) {
        if(!create_pin(pins[i], rp1)) {
            printf("unable to create pin %d\n", pins[i]);
            return 3;
        };
        pin_enable_output(pins[i], rp1);
    }

    clock_t end = clock();

    printf("Created pins: %ld, End: %ld, Elapsed: %ld\n", start, end, end - start);

    for (int i = 1; i < 10; i++)
    {
        delay_ms(500);
        for(j=0;j<4;j++) {
            pin_on(rp1, pins[j]);
            delay_ms(100);
        }

        delay_ms(500);

        for(j=0;j<4;j++) {
            pin_off(rp1, pins[j]);
            delay_ms(100);
        }        
    }

    printf("done\n\n");

    return 0;
}


// eof
