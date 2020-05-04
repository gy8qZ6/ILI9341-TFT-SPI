/* TODO: try to use hardware chip selects */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <bcm2835.h>
#include "Adafruit_ILI9341.h"
#include "glcdfont.c"

#include <math.h>

#include <sys/inotify.h>

#include "test.h"

/*
amount of time between each pixel on the x axis
but special behaviour because we modulo minutes part of
the timestamp (0 - 60) by this value
     5: 12 per hour
    10: 6 per hour
    15: 4 per hour
    20: 3 per hour
    30: 2 per hour
   >59: 1 per hour
*/
#define DATA_INTERVAL_MINUTES 15 
#define GRAPH_XAXIS_MARK_INTERVAL 12*60 // 12hours

// read sensor data from this file
#define LOG_FILE "/home/pi/driver_dev/SPI/BME280.log"
#define GRAPH_BUF_LEN 300 // length of ring buffer for sensor values == length of x axis in pixels

// inotify to watch LOG_FILE
#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define EVENT_BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )


// holds configuration values for function drawGraph
struct graph_config
{
    char type;
    uint32_t min;       // draw y-axis from current minimum to maximum sensor value
    uint32_t max;
    uint32_t mark_big;  // big mark interval on y-axis
    uint32_t mark_small;    // small mark interval on y-axis
    uint32_t (*get_val_func)(uint16_t); // function pointer to either get T,P, or H sensor values
                                        // from the ringbuffer
};
struct graph_config temp = {
    .type = 'T',
    .min = -1,
    .max = 0,
    .mark_big = 100,
    .mark_small = 50,
    .get_val_func = &get_temp
};   
struct graph_config pres = {
    .type = 'P',
    .min = -1,
    .max = 0,
    .mark_big = 100,
    .mark_small = 50,
    .get_val_func = &get_press
};   
struct graph_config hum = {
    .type = 'H',
    .min = -1,
    .max = 0,
    .mark_big = 500,
    .mark_small = 250,
    .get_val_func = &get_hum
};   

uint16_t _width, _height;
uint8_t dc_pin = RPI_V2_GPIO_P1_22;
uint8_t cs2_pin = RPI_V2_GPIO_P1_16;
uint8_t cs_pin = RPI_V2_GPIO_P1_13;
uint8_t rst_pin = RPI_V2_GPIO_P1_18;
int fd;
int fd_inotify;
int wd_inotify;
uint64_t logfile_pos = 0;

// ringbuffer for sensor values
// one ringbuffer element stores sensor value for all three types
// NOTE: we don't store the timestamp from the logfile in the ringbuffer
struct sensor_vals {
    // all sensor values in integers * 100 instead of floats
    uint16_t temp;
    uint16_t hum;
    uint32_t press;
};
struct sensor_vals *values;
uint16_t rb_read_index = 0, rb_write_index = 0;

// have some fun with graph drawing
// make a color gradient over the entire graph
/* color is 16bit of RGB with R:5 G:6 B:5 bits each */
uint16_t color_increase(uint16_t color)
{
    uint8_t red_max, blue_max;
    uint8_t green_max = 0x003f; 
    red_max = blue_max = 0x001f;
    uint8_t b = color & 0x001f; // 11111
    uint8_t g = color >> 5 & 0x003f; // 111111
    uint8_t r = color >> 11;

    // gives gradient from black to light green
    // first increase green color intensity, then red
    if (g != green_max) 
    {
        g++;
    } 
    else
    {
        r++;
    }

    /*
    if ((b++) > blue_max)
    {
        if ((g++) > green_max)
            r++;
    }
    */
    /*
    if (r == red_max) 
    {
        g++;
    } 
    else
    {
        r++;
    }
    */

    /*
    uint8_t step = 1;
    if ((b += blue_max / step) > blue_max)
    {
        if ((g += green_max /step) > green_max)
            r += red_max / step;
    }
    */
    
    color = 0;
    b = b & 0x001f; // 11111
    g = g & 0x003f; // 111111
    r = r & 0x001f;;
    color = r << 11 | g << 5 | b;
    return color;
}

/*
 * convert str to integer
 * used to read values from CSV logfile
 * implemented this for fun instead of using the standard atoi()
 *
 * param:   takes pointer to pointer of string value in logfile line
 * returns: value as an integer
 *          and s now points to the next value in the source string
 */
uint32_t str_to_int(char **s)
{
    uint32_t res = 0;
    char c;
    
    while ((c = *(*s)++) != ',' && c != '\n' && c != ':')
    {
        // read in float value as integer by ignoring decimal point
        // used only on float strings with two decimal places (i.e. 13.xy),
        // so int32 can hold the value
        if (c == '.') continue;  
        // convert ASCII char into digit and append to our integer value
        res = res * 10 + (c & 0x0F);
    }
    return res;
}

/* read stored data from a file at program start into ringbuffer
 * update ringbuffer with new data on subsequent calls
 */
int init_data_from_file()
{
    int16_t no_lines = 0;
    FILE *f;
    if (!(f = fopen(LOG_FILE, "r")))
    {
        perror("error");
        exit(1);
    }

    // TODO: intelligent searching for the first relevant datum
    // read every line from the start of the file, if timestamp
    // is one of the desired, save the date in ringbuffer
        
    if (logfile_pos == 0)
    {
        // init ringbuffer head on first call of this function
        values = (struct sensor_vals*) malloc(sizeof (struct sensor_vals) * GRAPH_BUF_LEN);
    }
    else
    {
        // seek to proper position in file on subsequent calls to this function
        fseek(f, logfile_pos--, SEEK_SET);
    }

    char line[40];
    struct sensor_vals *ptr = values;
    while (fgets(line, sizeof line, f))
    {
        
        char *sep = strchr(line, ':') + 1; // point sep to minute 
        uint8_t minute = (uint8_t) str_to_int(&sep); // sep now points to seconds value
        sep = strchr(sep, ',') + 1; // point sep to first sensor value

        // only read values fitting our intervals from log file
        if (minute % DATA_INTERVAL_MINUTES) continue;

        // output new measured values
        if (logfile_pos != 0) printf("read: %s\n", line);

        uint16_t temp = str_to_int(&sep);
        uint32_t press = str_to_int(&sep);
        uint16_t hum = str_to_int(&sep);

        // TODO: do proper interface for ringbuffer?

        // rb_write_buffer points to oldest datum, gets overwritten
        ptr = values + rb_write_index;
        ptr->temp = temp;
        ptr->hum = hum;
        ptr->press = press;

        // advance rb_write_index to the now oldest datum
        rb_write_index = ++rb_write_index % GRAPH_BUF_LEN;
        
        // if write_index has caught up to read_index cause buffer is full
        // advance read index
        if (rb_write_index == rb_read_index) 
        {
            rb_read_index = ++rb_read_index % GRAPH_BUF_LEN;
        }
        
    }
        
    // save current position so we can start reading logfile from here
    // on the next screen update
    logfile_pos = ftell(f);
    fclose(f);
}

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
int sendCommand(uint8_t cmd, const uint8_t *addr, uint8_t numArgs)
{
    struct spi_ioc_transfer xfer[1];
    memset(xfer, 0, sizeof xfer);
    xfer[0].tx_buf = (unsigned long)&cmd;
    xfer[0].len = 1;
    //xfer[0].cs_change = 0;
	//bcm2835_gpio_write(cs_pin, LOW);

    // send command with activated Data/Control bit
	bcm2835_gpio_write(dc_pin, LOW);
    uint8_t status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        printf("error\n");
        perror("SPI_IOC_MESSAGE");
        //bcm2835_close();
        return 1;
    }
	bcm2835_gpio_write(dc_pin, HIGH);

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
#define TEST_SPI_CS_LOW() bcm2835_gpio_write(cs_pin, LOW)
#define TEST_SPI_CS_HIGH() bcm2835_gpio_write(cs_pin, HIGH)

#define TEST_SPI_CS2_LOW() bcm2835_gpio_write(cs2_pin, LOW)
#define TEST_SPI_CS2_HIGH() bcm2835_gpio_write(cs2_pin, HIGH)

#define TEST_SPI_DC_LOW() bcm2835_gpio_write(dc_pin, LOW)// Command mode
#define TEST_SPI_DC_HIGH() bcm2835_gpio_write(dc_pin, HIGH)// Data mode

#define TEST_RST_LOW() bcm2835_gpio_write(rst_pin, LOW)
#define TEST_RST_HIGH() bcm2835_gpio_write(rst_pin, HIGH)

// send a command to ILI9341 that receives a 1Byte answer
uint8_t readcommand8(uint8_t commandByte)//, uint8_t index) {
{
  uint8_t result;

  TEST_SPI_DC_LOW(); // Command mode
  write(fd, &commandByte, 1);
  TEST_SPI_DC_HIGH(); // Data mode

  read(fd, &result, 1);
  return result;
}

// send 1 Byte to ILI9341
void SPI_WRITE8(uint8_t value)
{
    write(fd, &value, 1);
}

// send 2 Bytes to ILI9341
void SPI_WRITE16(uint16_t value)
{
    uint8_t msb = value >> 8; 
    uint8_t lsb = (uint8_t)value; 
    write(fd, &msb, 1);
    write(fd, &lsb, 1);
}

// send 1Byte command to ILI9341
void writeCommand(uint8_t cmd)
{
    TEST_SPI_DC_LOW();
    SPI_WRITE8(cmd);
    TEST_SPI_DC_HIGH();
}
    
// set up a pixel drawing area on the display
void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
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
int writeColor(uint16_t color, uint32_t len) 
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
    TEST_SPI_DC_LOW();
    if (mode)
    {
        SPI_WRITE8(ILI9341_INVON);
    } else
    {
        SPI_WRITE8(ILI9341_INVOFF);
    }
    TEST_SPI_DC_HIGH();
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

// functions to retrieve sensor values from ringbuffer element
uint32_t get_temp(uint16_t index)
{
    return values[index].temp;
}
uint32_t get_press(uint16_t index)
{
    return values[index].press;
}
uint32_t get_hum(uint16_t index)
{
    return values[index].hum;
}

/* draw both axis and graph for one sensor value */
void drawGraph(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
               struct graph_config* gc, uint16_t color, uint8_t flag_update)
{
    if ((x < 0 || x + width > _width) ||
        (y < 0 || y + height > _height))
    {
        printf("graph would be out of bounds, aborting");
        return;
    }

    // poo coordinates are absolute to the whole screen!!
    uint16_t poo_x = x + width/9;
    uint16_t poo_y = y + height/8;
    //uint16_t len_x = width - poo_x;//width*8/9;
    uint16_t len_x = width - width/9;//poo_x;//width*8/9;
    //uint16_t len_y = height-(height/10*2);
    uint16_t len_y = height - height/8;//poo_y - 10;

    //printf("drawg: %u %u %u %u      %u %u %u %u\n", x,y,width,height,poo_x,poo_y,len_x,len_y);

    uint32_t val_min;
    uint32_t val_max;
    uint32_t val_range;

    // we have this many pixels to draw for the graph (don't draw on the y axis)
    int16_t pixel_number = len_x - 1;

    // values[rb_write_index] is unused, last value is values[rb_write_index-1]
    // calculate starting index in ringbuffer values
    int16_t start_index = rb_write_index - pixel_number;
    //start_index = start_index < 0 ? GRAPH_BUF_LEN + start_index : start_index;
    if (start_index < 0)
    {
        start_index = GRAPH_BUF_LEN + start_index;
    }
    val_min = -1; // biggest unsigned int value
    val_max = 0; // lowest unsigned int value
    //printf("init: val_min: %u val_max: %u\n", val_min, val_max);

    for (int i = 0; i < pixel_number; i++)
    {
        // find min and max of sensor value
        uint32_t val = gc->get_val_func((start_index + i) % GRAPH_BUF_LEN);
        if (val < val_min) 
        {
            val_min = val;
        }
        if (val > val_max)
        {
            val_max = val;
        }
    }

    val_range = val_max - val_min;
    //printf("max: %i min: %i\n", val_max, val_min);
    
    uint8_t flag_redraw_y = 0;

    // avoid redrawing of coordinate systems x and y axis if we are only updating the graph
    if (flag_update)
    {
        // compare with old val_min and val_max values to see if we need to redraw 
        // the y axis
        if (val_max != gc->max || val_min != gc->min)
        {
            // blacken y axis marks
            fillRect(x, y, width/9, height, ILI9341_BLACK);//WHITE);
            flag_redraw_y = 1;
        }
    }
    else
    {
        // initial drawing of the graph, so draw everything

        // x axis
        fillRect(poo_x, poo_y, len_x, 1, color);//WHITE);
        // y axis
        fillRect(poo_x, poo_y, 1, len_y, color);//WHITE);

        // draw x axis and markings
        //len_x - 1 = number of pixels above x axis (without poo_x cause that would be drawing over the y axis)
        // draw mark every 6h: 6 * 60 / 15 = 24 pixel, counted from the very right
        // draw mark every 12h: 12 * 60 / 15 = 48 pixel, counted from the very right
        uint16_t pixel_step = GRAPH_XAXIS_MARK_INTERVAL / DATA_INTERVAL_MINUTES;

        // draw x axis marks and annotations 
        for (int16_t k = 0; k <= len_x / pixel_step; k++ )
        {
            fillRect(width - 1 - k * pixel_step, poo_y - 5, 1, 5, color);//WHITE);
            if (k)
            {
                //drawChar(width - 1 - 2 - k*pixel_step, poo_y - 15, 'X', ILI9341_GREEN, ILI9341_BLACK, 1, 1);

                char mark_str[8];
                //int8_t number = - k * GRAPH_XAXIS_MARK_INTERVAL / 60;
                int8_t number = k * GRAPH_XAXIS_MARK_INTERVAL / 60;
                //int16_t number = 8888;
                sprintf(mark_str, "%d", number); 
                //float pixel_offset = strlen(mark_str) / 2;
                
                for (uint8_t m = 0; m < strlen(mark_str); m++)
                {
                    drawChar(width - k*pixel_step - (uint8_t)(strlen(mark_str) * 6 / 2) + m * 6, poo_y - 15, mark_str[m], color, ILI9341_BLACK, 1, 1);
                }
            } else
            {
                char mark_str[4] = "now";
                for (uint8_t m = 0; m < strlen(mark_str); m++)
                {
                    drawChar(width - (uint8_t)(strlen(mark_str) * 6) + m * 6, poo_y - 15, mark_str[m], color, ILI9341_BLACK, 1, 1);
                }
            }
        }
    }

    gc->max = val_max;
    gc->min = val_min;

    // draw marks on y axis
    if (!flag_update || flag_redraw_y)
    {
        // but not at the point of origin
        // complex because resizing depends on val_min & val_max!
        uint16_t rest = gc->mark_small - (val_min % gc->mark_small);
        uint8_t number_of_marks = (val_max - (val_min + rest)) / gc->mark_small + (val_min % gc->mark_small?1:0);
        for (uint8_t i = 0; i < number_of_marks; i++) 
        {
            uint16_t tmp = rest + i * gc->mark_small;
            float rel_mark_pos = (float)tmp / val_range;
            uint32_t mark_val = val_min + rest + i * gc->mark_small;
            //printf("mark val: %d\n", mark_val);
            
            // draw small or big mark
            uint8_t length_div = mark_val % gc->mark_big ? 8 : 4;

            //uint16_t mark_x = poo_x - width/10/length_div;
            uint16_t mark_x = poo_x - (width - len_x)/length_div;
            uint16_t mark_y = poo_y + (int16_t)(len_y * rel_mark_pos);
            uint16_t mark_w = (width - len_x)/length_div;
            fillRect(mark_x, mark_y, mark_w, 1, color);//WHITE);

            // annotate big marks with value string
            if (length_div == 4) 
            {
                mark_val /= 100;

                char mark_str[10];
                
                // use own itoa()-like implementation instead of sprintf ;)
                int8_t k;
                for (k = 0; mark_val; mark_val /= 10, k++)
                {
                    mark_str[k] = mark_val % 10 | 0x30;
                }
                int8_t j = 0;
                for (k -=1; k >= 0; k--,j++)
                {
                    uint16_t char_x = mark_x - 6;// = poo_x - (width - len_x)/length_div;
                    //drawChar((int16_t)x + k * 6, (int16_t)mark_y - 3, mark_str[j], ILI9341_WHITE, ILI9341_BLACK, 1, 1);
                    drawChar(char_x - j * 6, (int16_t)mark_y - 3, mark_str[j], color, ILI9341_BLACK, 1, 1);
                }
            }
                
        }
    }

    uint16_t prev_y = 0;
    
    // starting color for graph gradient
    uint16_t c = ILI9341_BLACK;

    for (int i = 0; i < pixel_number; i++)
    {
        float rel_pos = (float) (gc->get_val_func((start_index + i) % GRAPH_BUF_LEN) - val_min) / 
                                    val_range;
        float tmp = rel_pos * (len_y - 1);  // there are len_y - 1 pixel above the x axis
        uint16_t y = roundf(tmp*10.0f)/10.0f;

        // color gradient
        if (i % 4 == 0) c = color_increase(c);

        // blacken current column
        fillRect(i + poo_x + 1, poo_y + 1, 1, len_y - 1, ILI9341_BLACK);//WHITE);

        /*
         * if we decide to switch back to line only graphs, then use this to get
         * unbroken graph lines:
        
        if (!i) writePixel(i + poo_x + 1, y + poo_y, c);

        if (i)  // skip for first drawn value
        {
            int16_t diff = (int16_t) y - prev_y;
            int8_t lost_pixel = diff % 2 ? 1 : 0; // if diff is odd, this makes one line longer than the other
            //diff = 0;
            if (diff > 1) 
            {
                // connect pixels by drawing a vertical line between them;
                // actually two to make the line a small slope towards the next pixel

                // x           x               x
                //     naive   |   what we do  |
                //     ->      |       ->       |
                //  x           x               x

                // lines are always drawn from the bottom up
                // lines don't overlap at their meeting point

                // left line from prev_y to the midpoint between both y values
                fillRect(i + poo_x, prev_y + 1 + poo_y, 1, diff / 2, c);//WHITE);

                // right line from the midpoint between both y values to the new y
                fillRect(i + poo_x + 1, prev_y + 1 + poo_y + diff / 2, 1, diff / 2 + lost_pixel, c);//WHITE);
                
            } else if (diff < -1)
            {
                diff *= -1;
                // left line from the midpoint to prev_y
                fillRect(i + poo_x, prev_y + poo_y - diff / 2, 1, diff / 2, c);//WHITE);

                // right line from the new y to the midpoint 
                fillRect(i + poo_x + 1, y + poo_y, 1, diff / 2 + lost_pixel, c);//WHITE);
            } else
            {
                writePixel(i + poo_x + 1, y + poo_y, c);
                //printf("drawg: y abs: %d\n", y + poo_y);
            } 
        }
        prev_y = y;
        */
       
        // this is sufficient if we (and even better than smoothing) in area-graph-mode
        fillRect(i + poo_x + 1, poo_y + 1, 1, y - 1, c);//WHITE);

        //writePixel(i + poo_x + 1, y + poo_y, color);

    }
    
    // draw x axis again because very low values can be drawn onto the x axis
    // if the axis is a different color this becomes visible as a gap we don't want
    // TODO fix?
    fillRect(poo_x, poo_y, len_x, 1, color);//WHITE);
}

// init the spidev interface for communicating with the SPI driver
int init_spidev(char *name)
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

// init GPIOs that we use for chip select and Data/Control line
int init_gpio()
{
    // to control GPIO Pins
    if (!bcm2835_init()) {
      printf("error bcm2835_init()\n");
      fflush(stdout);
      return 1;
    }

    // Set the pin to be an output
    bcm2835_gpio_fsel(dc_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(cs_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(cs2_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(rst_pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(cs_pin, HIGH);
	bcm2835_gpio_write(cs2_pin, HIGH);
	bcm2835_gpio_write(dc_pin, HIGH);
}

void ili9341_reset()
{
    if (rst_pin >= 0) {
       // Toggle _rst low to reset
       //pinMode(rst_pin, OUTPUT);
       //digitalWrite(rst_pin, HIGH);
       TEST_RST_HIGH();
       delay(100);
       //digitalWrite(rst_pin, LOW);
       TEST_RST_LOW();
       delay(100);
       //digitalWrite(rst_pin, HIGH);
       TEST_RST_HIGH();
       delay(200);
    }
}

void status()
{
    uint8_t cmd = ILI9341_RDMODE;// 0x0A     ///< Read Display Power Mode
    uint8_t res = readcommand8(cmd);//, uint8_t index) {
    printf("sent: 0x%02x rcv: 0x%02x\n", cmd, res);
}

void init_displays()
{
    // initialize two displays at once
    // both connected over the same SPI bus
    ili9341_reset();
    TEST_SPI_CS_LOW();
    TEST_SPI_CS2_LOW();
    begin();
    TEST_SPI_CS_HIGH();
    TEST_SPI_CS2_HIGH();

    // do a status test for each display
    TEST_SPI_CS_LOW();
    status();
    TEST_SPI_CS_HIGH();
    TEST_SPI_CS2_LOW();
    status();
    TEST_SPI_CS2_HIGH();
}

// init inotify for monitoring sensor logfile
void init_inotify()
{
    fd_inotify = inotify_init();

    if (fd_inotify < 0)
    {
        perror("inotify_init");
    }
    wd_inotify = inotify_add_watch(fd_inotify, LOG_FILE, IN_MODIFY);
}

/*  our main drawing function
    set up the screen layout here
    flag_update: only redraw dynamic content */
void screen_draw(uint8_t flag_update)
{
    // start drawing on the displays
    uint16_t fg_color = ILI9341_GREEN;
    uint16_t bg_color = ILI9341_BLACK;

    TEST_SPI_CS_LOW();

    if (!flag_update)
    {
        fillRect(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT, bg_color);
    }
    //drawGraph(0, 0, _width, _height);
    /*
    drawGraph(0, 0, _width, _height, 'H', ILI9341_BLUE);
    drawGraph(0, 0, _width, _height, 'P', ILI9341_GREEN);
    drawGraph(0, 0, _width, _height, 'T', ILI9341_RED);
    */
    /*
    drawGraph(0, _height/2-1, _width/2, _height/2, 'H', ILI9341_BLUE);
    drawGraph(_width/2-1, _height/2-1, _width/2, _height/2, 'P', ILI9341_GREEN);
    drawGraph(0, 0, _width, _height/2-1, 'T', ILI9341_RED);
    //drawGraph(0, 0, _width/2, _height/2-1, 'T', ILI9341_RED);
    */
   // drawGraph(0, _height/2-1, _width, _height/2, 'H', ILI9341_CYAN);//BLUE);
    drawGraph(0, _height/2, _width, _height/2, &hum, fg_color, flag_update);//CYAN);//GREEN);
    drawGraph(0, 0, _width, _height/2, &temp, fg_color, flag_update);//MAGENTA);//RED);
    //drawGraph(0, _height/2-1, _width/2, _height/2);

    TEST_SPI_CS_HIGH();

    TEST_SPI_CS2_LOW();
    //startWrite2();
    if (!flag_update)
    {
        fillRect(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT, bg_color);
    }
    //drawGraph(0, _height/2-1, _width, _height/2, 'P', ILI9341_ORANGE);//BLUE);
    drawGraph(0, 0, _width, _height, &pres, fg_color, flag_update);//ORANGE);//BLUE);
    TEST_SPI_CS2_HIGH();
    //endWrite2();
}

/*  monitor data log file for changes,
    read in the new dataset(s) and redraw graph
    if a new dataset of relevant time frame came in */
void update()
{
    uint32_t length;
    char buffer[EVENT_BUF_LEN];
    uint32_t i = 0;

    while ((length = read(fd_inotify, buffer, EVENT_BUF_LEN)) == EINTR)
    {
        perror("read");
    } 

    // process the inotify event
    while (i < length) 
    {     
        struct inotify_event *event = (struct inotify_event*) &buffer[i];
        if (event->mask & IN_MODIFY) 
        {
            /*
            printf("File %s modified.\n", event->name);
            printf("event len: %d\n", event->len);
            */
    
            uint64_t tmp = rb_write_index;
            // read new data from file and redraw graph
            init_data_from_file();
            if (rb_write_index != tmp)
            {
                // new relevant data came in
                screen_draw(1);
            }
        }
        i += EVENT_SIZE + event->len;
    }
}

int main(int argc, char **argv)
{
    char *name = argv[1];
    unsigned char buf[32], *bp;

    // initialize hardware
    init_spidev(name);
    init_gpio();

    // initialize sensor data
    init_data_from_file();

    init_displays();

    screen_draw(0);

    init_inotify();
    // redraw graph whenever new (relevant) data becomes available 
    // in the sensor data log file
    while (1)
    {
        update();
    }
    
    // this seems useless now ;)
    // TODO implement cleanup function when program gets killed
    inotify_rm_watch(fd_inotify, wd_inotify);
    close(fd_inotify);
    bcm2835_close();
    free(values);
    return 0;
}
