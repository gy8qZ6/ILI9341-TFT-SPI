/*!
 * Written by: Cornelius Radicke
 * 
 * This library is inspired by and may contain code from the
 * Adafruit ILI9341 library, so I'm including their header here:
 *
 *******************************************************************************
 * @file Adafruit_ILI9341.h
 *
 * This is the documentation for Adafruit's ILI9341 driver for the
 * Arduino platform.
 *
 * This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
 *    http://www.adafruit.com/products/1651
 * Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/2478
 * 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/1770
 * 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
 *    https://www.adafruit.com/product/1770
 * TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers
 *    https://www.adafruit.com/product/3315
 *
 * These displays use SPI to communicate, 4 or 5 pins are required
 * to interface (RST is optional).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <bcm2835.h>

#include "ili9341_spi.h"
#include "glcdfont.h"

#define ILI9341_SPI_DC_LOW() bcm2835_gpio_write(_dc_pin, LOW)// Command mode
#define ILI9341_SPI_DC_HIGH() bcm2835_gpio_write(_dc_pin, HIGH)// Data mode

#define ILI9341_RST_LOW() bcm2835_gpio_write(_rst_pin, LOW)
#define ILI9341_RST_HIGH() bcm2835_gpio_write(_rst_pin, HIGH)

uint16_t _width = ILI9341_TFTWIDTH;
uint16_t _height = ILI9341_TFTHEIGHT;
uint8_t _dc_pin;
// CS pins handled in user program
//uint8_t _cs2_pin = RPI_V2_GPIO_P1_16;
//uint8_t _cs_pin = RPI_V2_GPIO_P1_13;
uint8_t _rst_pin;

// init library 
void ili9341_spi_init(uint16_t width, uint16_t height, uint8_t dc_pin, 
                        uint8_t rst_pin, char *spidev)
{
    _width = width;
    _height = height;
    _dc_pin = dc_pin;
    _rst_pin = rst_pin;

    init_spidev(spidev);
    init_gpio();
}

int fd; // SPIDEV file descriptor

// initialization commands for ILI9341 Display
static const uint8_t initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

// send command + optional arguments to ILI9341
// TODO use write and read operations instead of ioctl
//      because we are not using the advanced functionality here
static int sendCommand(uint8_t cmd, const uint8_t *addr, uint8_t numArgs)
{
    struct spi_ioc_transfer xfer[1];
    memset(xfer, 0, sizeof xfer);
    xfer[0].tx_buf = (unsigned long)&cmd;
    xfer[0].len = 1;
    //xfer[0].cs_change = 0;
	//bcm2835_gpio_write(cs_pin, LOW);

    // send command with activated Data/Control bit
    ILI9341_SPI_DC_LOW();
    uint8_t status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        printf("error\n");
        perror("SPI_IOC_MESSAGE");
        //bcm2835_close();
        return 1;
    }
    ILI9341_SPI_DC_HIGH();

    // send Command Arguments if we have any
    if (numArgs)
    {
        xfer[0].tx_buf = (unsigned long) addr;
        xfer[0].len = numArgs;
        //xfer[1].cs_change = 0;

        status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
        if (status < 0) {
            printf("error\n");
            perror("SPI_IOC_MESSAGE");
            //bcm2835_close();
            return 1;
        }
    }
	//bcm2835_gpio_write(cs_pin, HIGH);
}

// initialize ILI9341 Display
void begin()
{
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = *addr++) > 0) {
    x = *addr++;
    numArgs = x & 0x7F;
    //printf("sendCommand 0x%02x 0x%02x 0x%02x\n", cmd, addr, numArgs);
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    if (x & 0x80)
      delay(150);
  }

  _width = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;
}

// -----------------------


// send a command to ILI9341 that receives a 1Byte answer
static uint8_t readcommand8(uint8_t commandByte)//, uint8_t index) {
{
  uint8_t result;

  ILI9341_SPI_DC_LOW(); // Command mode
  write(fd, &commandByte, 1);
  ILI9341_SPI_DC_HIGH(); // Data mode

  read(fd, &result, 1);
  return result;
}

// send 1 Byte to ILI9341
static void SPI_WRITE8(uint8_t value)
{
    write(fd, &value, 1);
}

// send 2 Bytes to ILI9341
static void SPI_WRITE16(uint16_t value)
{
    uint8_t msb = value >> 8; 
    uint8_t lsb = (uint8_t)value; 
    write(fd, &msb, 1);
    write(fd, &lsb, 1);
}

// send 1Byte command to ILI9341
static void writeCommand(uint8_t cmd)
{
    ILI9341_SPI_DC_LOW();
    SPI_WRITE8(cmd);
    ILI9341_SPI_DC_HIGH();
}
    
// set up a pixel drawing area on the display
static void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
                                     uint16_t h) {
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
  writeCommand(ILI9341_PASET); // Row address set
  SPI_WRITE16(x1);
  SPI_WRITE16(x2);
  writeCommand(ILI9341_CASET); // Column address set
  SPI_WRITE16(y1);
  SPI_WRITE16(y2);
  writeCommand(ILI9341_RAMWR); // Write to RAM
}

// control one pixel
void writePixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
    setAddrWindow(x, y, 1, 1);
    SPI_WRITE16(color);
  }
}

// color pixels that where defined by setAddrWindow() before
// we have to send the color value for each pixel individually
// that means we just send the same color value repeatedly here
// if we want to color an area
static int writeColor(uint16_t color, uint32_t len) 
{
    if (!len)
        return 0; // Avoid 0-byte transfers

    // max buf len per ioctl call is 2048
    int max_len = 2048;

    uint8_t hi = color >> 8, lo = color;
    uint8_t *buf;//[len*2];
    buf = (uint8_t*)malloc(max_len*2);
    for (int i =0; i<max_len;i++)
    {
        buf[2*i] = hi;
        buf[2*i+1] = lo;
    }

    int iterations = len / max_len;
    while (iterations--) {
        write(fd, buf, max_len*2);
    }
    write(fd, buf, (len % max_len)*2);

    free(buf);
}

// draw a filled rectangle
void fillRect(int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
    if ((x + width <= _width) && (y + height <= _height)) {
        setAddrWindow(x, y, width, height);
        writeColor(color, (uint32_t)width*height);
    }
  }
}

// invert the colors of the whole display
void invert(uint8_t mode)
{
    ILI9341_SPI_DC_LOW();
    if (mode)
    {
        SPI_WRITE8(ILI9341_INVON);
    } else
    {
        SPI_WRITE8(ILI9341_INVOFF);
    }
    ILI9341_SPI_DC_HIGH();
}

// draw an ASCII char on the display
void drawChar(int16_t x, int16_t y, unsigned char c,
                          uint16_t color, uint16_t bg, uint8_t size_x,
                          uint8_t size_y) {

  if ((x >= _width) ||              // Clip right
      (y >= _height) ||             // Clip bottom
      ((x + 6 * size_x - 1) < 0) || // Clip left
      ((y + 8 * size_y - 1) < 0))   // Clip top
    return;

  //if (!_cp437 && (c >= 176))
  if (c >= 176)
    c++; // Handle 'classic' charset behavior

  for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
    uint8_t line = font[c * 5 + i];//pgm_read_byte(&font[c * 5 + i]);
    for (int8_t j = 7; j >= 0; j--, line >>= 1) {
      if (line & 1) {
        if (size_x == 1 && size_y == 1)
          writePixel(x + i, y + j, color);
        else
          fillRect(x + i * size_x, y + j * size_y, size_x, size_y,
                        color);
      } else if (bg != color) {
        if (size_x == 1 && size_y == 1)
          writePixel(x + i, y + j, bg);
        else
          fillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
      }
    }
  }
  if (bg != color) { // If opaque, draw vertical line for last column
    if (size_x == 1 && size_y == 1)
      //writeFastVLine(x + 5, y, 8, bg);
      fillRect(x + 5, y, 1, 8, bg);
      
    else
      fillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
  }
}

// init the spidev interface for communicating with the SPI driver
static int init_spidev(char *name)
{
    fd = open(name, O_RDWR);
    if (fd < 0) {
        perror("open");
        printf("error opening spidev\n");
        exit(1);
    }

    uint32_t spi_speed = 50000000;        //1000000 = 1MHz (1uS per bit) 
    uint8_t status_value;

    status_value = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
        perror("Could not set SPI speed (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
        perror("Could not set SPI speed (RD)...ioctl fail");
        exit(1);
    }

    return 0;
    /*
    // TODO: check this again: has no effect:
    // send byte WITHOUT CHANGING CHIPSELECT
    uint8_t mode = SPI_MODE_0 | SPI_NO_CS;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode)<0)
    {
        printf("error setting mode\n");
    }
    */
}

// init GPIOs that we use for Reset and Data/Control line
// chip select are handled in user program
static int init_gpio()
{
    // to control GPIO Pins
    if (!bcm2835_init()) {
      printf("error bcm2835_init()\n");
      fflush(stdout);
      return 1;
    }

    // Set the pin to be an output
    bcm2835_gpio_fsel(_dc_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(_rst_pin, BCM2835_GPIO_FSEL_OUTP);
    ILI9341_SPI_DC_HIGH();
    /*
    bcm2835_gpio_fsel(cs_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(cs2_pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(cs_pin, HIGH);
	bcm2835_gpio_write(cs2_pin, HIGH);
    */
}

void ili9341_reset()
{
    if (_rst_pin >= 0) {
       // Toggle _rst low to reset
       //pinMode(rst_pin, OUTPUT);
       //digitalWrite(rst_pin, HIGH);
       ILI9341_RST_HIGH();
       delay(100);
       //digitalWrite(rst_pin, LOW);
       ILI9341_RST_LOW();
       delay(100);
       //digitalWrite(rst_pin, HIGH);
       ILI9341_RST_HIGH();
       delay(200);
    }
}

void status()
{
    uint8_t cmd = ILI9341_RDMODE;// 0x0A     ///< Read Display Power Mode
    uint8_t res = readcommand8(cmd);//, uint8_t index) {
    printf("sent: 0x%02x rcv: 0x%02x\n", cmd, res);
}
