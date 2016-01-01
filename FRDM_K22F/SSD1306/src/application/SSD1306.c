#include "SSD1306.h"

#include "i2c.h"
#include "font6x8.h"
#include <MK22F51212.h>


#define ssd1306_lcdwidth  128
#define ssd1306_lcdheight 64
uint8_t poledbuff[ssd1306_lcdwidth*ssd1306_lcdheight>>3] = {0x00};


void ssd1306_singleCMD(uint8_t c)
{
    // Write Data on I2C
    I2C_WriteCtrl(0x3C, SSD_Command_Mode, &c, 1);
}

void ssd1306_DualCMD(uint8_t c0, uint8_t c1)
{
	uint8_t buff[2] ;
	buff[0] = c0;
	buff[1] = c1;

    // Write Data on I2C
    I2C_WriteCtrl(0x3C, SSD_Command_Mode ,buff, sizeof(buff))	;
}

void ssd1306_TrippleCMD(uint8_t c0, uint8_t c1, uint8_t c2)
{
	uint8_t buff[3] ;

	buff[0] = c0;
	buff[1] = c1;
	buff[2] = c2;

    // Write Data on I2C
    I2C_WriteCtrl(0x3C, SSD_Command_Mode ,buff, sizeof(buff))	;
}


void ssd1306_WriteData(uint8_t *dat, uint8_t len)
{
    // Write Data on I2C
    I2C_WriteCtrl(0x3C, SSD_Data_Mode ,dat, len)	;
}

void invertDisplay(uint8_t i)
{
  if (i)
    ssd1306_singleCMD(SSD_Inverse_Display);
  else
    ssd1306_singleCMD(SSD1306_Normal_Display);
}

// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void startscrollright(uint8_t start, uint8_t stop)
{
	ssd1306_singleCMD(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_singleCMD(0X00);
	ssd1306_singleCMD(start);
	ssd1306_singleCMD(0X00);
	ssd1306_singleCMD(stop);
	ssd1306_singleCMD(0X01);
	ssd1306_singleCMD(0XFF);
	ssd1306_singleCMD(SSD_Activate_Scroll);
	ssd1306_singleCMD(SSD_Deactivate_Scroll);
}

void display(void)
{

	uint16_t i=0 ;
	uint8_t x ;

	// pointer to OLED data buffer
	uint8_t * p = poledbuff;
    unsigned char buff[17] ;

    ssd1306_singleCMD(0x21);
    ssd1306_singleCMD(0x00);
    ssd1306_singleCMD(0x7F);

    ssd1306_singleCMD(0x22);
    ssd1306_singleCMD(0x00);
    ssd1306_singleCMD(0x07);

    // Setup D/C to switch to data mode
    buff[0] = SSD_Data_Mode;

    // loop trough all OLED buffer and
    // send a bunch of 16 data byte in one xmission
    for ( i=0; i<(ssd1306_lcdwidth*ssd1306_lcdheight)>>7; i++ ){
        for (x=1; x<=16; x++)
            buff[x] = *p++;
        I2C_Write(0x3C,	&buff[0], 17);
    }
}

// clear everything (in the buffer)
void clearDisplay(void)
{
    uint16_t i;
    for ( i=0; i<ssd1306_lcdwidth*ssd1306_lcdheight>>3; i++ ) {
        poledbuff[i] = 0x00;
    }
}

void ssd_init( void )
{
//  uint8_t multiplex;
  uint8_t chargepump;
  uint8_t compins;
  uint8_t contrast;
  uint8_t precharge;

		//multiplex = 0x3F;
		compins 	= 0x12;//0x12;
		contrast	= 0xCF;//(vcc_type==SSD_External_Vcc?0x9F:0xCF);

		chargepump = 0x14;
		precharge  = 0xF1;


	ssd1306_singleCMD(SSD_Display_Off);                    // 0xAE
	ssd1306_DualCMD(SSD1306_SETDISPLAYCLOCKDIV, 0x80);      // 0xD5 + the suggested ratio 0x80
	ssd1306_DualCMD(SSD1306_SETMULTIPLEX, 0x3F);
	ssd1306_DualCMD(SSD1306_SETDISPLAYOFFSET, 0x00);        // 0xD3 + no offset
	ssd1306_singleCMD(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_DualCMD(SSD1306_CHARGEPUMP, chargepump);
	ssd1306_DualCMD(SSD1306_MEMORYMODE, 0x00);              // 0x20 0x0 act like ks0108
	ssd1306_singleCMD(SSD1306_SEGREMAP | 0x1);
	ssd1306_singleCMD(SSD1306_COMSCANDEC);
	ssd1306_DualCMD(SSD1306_SETCOMPINS, compins);  // 0xDA
	ssd1306_DualCMD(SSD_Set_ContrastLevel, contrast);
	ssd1306_DualCMD(SSD1306_SETPRECHARGE, precharge); // 0xd9
	ssd1306_DualCMD(SSD1306_SETVCOMDETECT, 0x40);  // 0xDB
	ssd1306_singleCMD(SSD1306_DISPLAYALLON_RESUME);    // 0xA4
	ssd1306_singleCMD(SSD1306_Normal_Display);         // 0xA6

	// Reset to default value in case of
	// no reset pin available on OLED
	/*ssd1306_TrippleCMD( 0x21, 0, 127 );
	ssd1306_TrippleCMD( 0x22, 0,   7 );*/
	//stopscroll();

	// Empty uninitialized buffer
	//clearDisplay();
	//clearDisplay();

	//--turn on oled panel
	clearDisplay();

	display();
	//invertDisplay(1);
	//invertDisplay(0);
	//startscrollright(0x00, 0x01);
	//startscrollright(0x00, 0x01);
ssd1306_singleCMD(SSD_Display_On);
}


void ssd1306_setpos(uint8_t x, uint8_t y)
{
    ssd1306_TrippleCMD(0xb0 + y, ((x & 0xf0) >> 4) | 0x10 , (x & 0x0f) | 0x01);
}

void ssd1306_char_font6x8(char ch) {
	uint8_t c = ch - 32;
    ssd1306_WriteData(&ssd1306_font6x8[c * 6],6);
}

void ssd1306_writeString(uint8_t x, uint8_t y, char *str){
    ssd1306_setpos(x,y);
    while(*str != 0) {  // while end of string not reached and printable char
        ssd1306_char_font6x8(*str++); // write next character
    }
}


/*


// Low level I2C and SPI Write function
inline void Adafruit_SSD1306::fastSPIwrite(uint8_t d) {
	bcm2835_spi_transfer(d);
}
inline void Adafruit_SSD1306::fastI2Cwrite(uint8_t d) {
	bcm2835_spi_transfer(d);
}
inline void Adafruit_SSD1306::fastSPIwrite(char* tbuf, uint32_t len) {
	bcm2835_spi_writenb(tbuf, len);
}
inline void Adafruit_SSD1306::fastI2Cwrite(char* tbuf, uint32_t len) {
	bcm2835_i2c_write(tbuf, len);
}

// the most basic function, set a single pixel
void Adafruit_SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	uint8_t * p = poledbuff ;

  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

		// check rotation, move pixel around if necessary
	switch (getRotation())
	{
		case 1:
			swap(x, y);
			x = WIDTH - x - 1;
		break;

		case 2:
			x = WIDTH - x - 1;
			y = HEIGHT - y - 1;
		break;

		case 3:
			swap(x, y);
			y = HEIGHT - y - 1;
		break;
	}

	// Get where to do the change in the buffer
	p = poledbuff + (x + (y/8)*ssd1306_lcdwidth );

	// x is which column
	if (color == WHITE)
		*p |=  _BV((y%8));
	else
		*p &= ~_BV((y%8));
}

// Display instantiation
Adafruit_SSD1306::Adafruit_SSD1306()
{
	// Init all var, and clean
	// Command I/O
  rst = 0 ;
  dc  = 0 ;
  cs =  0 ;

	// Lcd size
  ssd1306_lcdwidth  = 0;
  ssd1306_lcdheight = 0;

	// Empty pointer to OLED buffer
	poledbuff = NULL;
}


// When not initialized program using this library may
// know protocol for correct init call, he could just know
// oled number in driver list
boolean Adafruit_SSD1306::oled_is_spi_proto(uint8_t OLED_TYPE)
{
	switch (OLED_TYPE)
	{
		case OLED_ADAFRUIT_SPI_128x32:
		case OLED_ADAFRUIT_SPI_128x64:
			return true;
		break;
	}

	// default
	return false;

}

// initializer for OLED Type
boolean Adafruit_SSD1306::select_oled(uint8_t OLED_TYPE)
{
	// Default type
	ssd1306_lcdwidth  = 128;
	ssd1306_lcdheight = 64;
	_i2c_addr = 0x00;

	// default OLED are using internal boost VCC converter
	vcc_type = SSD_Internal_Vcc;

	// Oled supported display
	// Setup size and I2C address
	switch (OLED_TYPE)
	{
		case OLED_ADAFRUIT_SPI_128x32:
			ssd1306_lcdheight = 32;
		break;

		case OLED_ADAFRUIT_SPI_128x64:
		;
		break;

		case OLED_ADAFRUIT_I2C_128x32:
			ssd1306_lcdheight = 32;
			_i2c_addr = ADAFRUIT_I2C_ADDRESS;
		break;

		case OLED_ADAFRUIT_I2C_128x64:
			_i2c_addr = ADAFRUIT_I2C_ADDRESS;
		break;

		case OLED_SEEED_I2C_128x64:
			_i2c_addr = SEEEED_I2C_ADDRESS ;
			vcc_type = SSD_External_Vcc;
		break;

		case OLED_SEEED_I2C_96x96:
			ssd1306_lcdwidth  = 96;
			ssd1306_lcdheight = 96;
			_i2c_addr = SEEEED_I2C_ADDRESS ;
		break;

		// houston, we have a problem
		default:
			return false;
		break;
	}

	// De-Allocate memory for OLED buffer if any
	if (poledbuff)
		free(poledbuff);

	// Allocate memory for OLED buffer
	poledbuff = (uint8_t *) malloc ( (ssd1306_lcdwidth * ssd1306_lcdheight / 8 ));
	if (!poledbuff)
    return false;

	// Init Raspberry PI GPIO
  if (!bcm2835_init())
    return false;

	return true;

}

// initializer for SPI - we indicate the pins used and OLED type
//
boolean Adafruit_SSD1306::init(int8_t DC, int8_t RST, int8_t CS, uint8_t OLED_TYPE)
{
  rst = RST;	// Reset Pin
  dc = DC;		// Data / command Pin
	cs = CS ;		// Raspberry SPI chip Enable (may be CE0 or CE1)

	// Select OLED parameters
	if (!select_oled(OLED_TYPE))
		return false;

	// Init & Configure Raspberry PI SPI
	bcm2835_spi_begin(cs);
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

	// 16 MHz SPI bus, but Worked at 62 MHz also
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);

	// Set the pin that will control DC as output
	bcm2835_gpio_fsel(dc, BCM2835_GPIO_FSEL_OUTP);

  // Setup reset pin direction as output
  bcm2835_gpio_fsel(rst, BCM2835_GPIO_FSEL_OUTP);

	return ( true);
}

// initializer for I2C - we only indicate the reset pin and OLED type !
boolean Adafruit_SSD1306::init(int8_t RST, uint8_t OLED_TYPE)
{
  dc = cs = -1; // DC and chip Select do not exist in I2C
  rst = RST;

	// Select OLED parameters
	if (!select_oled(OLED_TYPE))
		return false;

	// Init & Configure Raspberry PI I2C
	if (bcm2835_i2c_begin()==0)
		return false;

	bcm2835_i2c_setSlaveAddress(_i2c_addr) ;

	// Set clock to 400 KHz
	// does not seem to work, will check this later
	// bcm2835_i2c_set_baudrate(400000);

  // Setup reset pin direction as output
  bcm2835_gpio_fsel(rst, BCM2835_GPIO_FSEL_OUTP);

	return ( true);
}

void Adafruit_SSD1306::close(void)
{
	// De-Allocate memory for OLED buffer if any
	if (poledbuff)
		free(poledbuff);

	poledbuff = NULL;

	// Release Raspberry SPI
	if ( isSPI() )
		bcm2835_spi_end();

		// Release Raspberry I2C
	if ( isI2C() )
		bcm2835_i2c_end();

	// Release Raspberry I/O control
  bcm2835_close();
}




void Adafruit_SSD1306::invertDisplay(uint8_t i)
{
  if (i)
    ssd1306_command(SSD_Inverse_Display);
  else
    ssd1306_command(SSD1306_Normal_Display);
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c)
{
    char buff[2] ;

    // Clear D/C to switch to command mode
    buff[0] = SSD_Command_Mode ;
    buff[1] = c;

    // Write Data on I2C
    fastI2Cwrite(buff, sizeof(buff))	;
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c0, uint8_t c1)
{
	char buff[3] ;
	buff[1] = c0;
	buff[2] = c1;

		// Clear D/C to switch to command mode
		buff[0] = SSD_Command_Mode ;

		// Write Data on I2C
		fastI2Cwrite(buff, 3)	;
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c0, uint8_t c1, uint8_t c2)
{
	char buff[4] ;

	buff[1] = c0;
	buff[2] = c1;
	buff[3] = c2;
		// Clear D/C to switch to command mode
		buff[0] = SSD_Command_Mode;

		// Write Data on I2C
		fastI2Cwrite(buff, sizeof(buff))	;
}


// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrollright(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrollleft(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(0XFF);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrolldiagright(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(ssd1306_lcdheight);
	ssd1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD_Activate_Scroll);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void Adafruit_SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop)
{
	ssd1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	ssd1306_command(0X00);
	ssd1306_command(ssd1306_lcdheight);
	ssd1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	ssd1306_command(0X00);
	ssd1306_command(start);
	ssd1306_command(0X00);
	ssd1306_command(stop);
	ssd1306_command(0X01);
	ssd1306_command(SSD_Activate_Scroll);
}

void Adafruit_SSD1306::stopscroll(void)
{
	ssd1306_command(SSD_Deactivate_Scroll);
}

void Adafruit_SSD1306::ssd1306_data(uint8_t c)
{
		char buff[2] ;

		// Setup D/C to switch to data mode
		buff[0] = SSD_Data_Mode;
		buff[1] = c;

		// Write on i2c
		fastI2Cwrite(	buff, sizeof(buff))	;
}

void Adafruit_SSD1306::display(void)
{
  ssd1306_command(SSD1306_SETLOWCOLUMN  | 0x0); // low col = 0
  ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
  ssd1306_command(SSD1306_SETSTARTLINE  | 0x0); // line #0

	uint16_t i=0 ;

	// pointer to OLED data buffer
	uint8_t * p = poledbuff;
		char buff[17] ;
		uint8_t x ;

		// Setup D/C to switch to data mode
		buff[0] = SSD_Data_Mode;

		// loop trough all OLED buffer and
    // send a bunch of 16 data byte in one xmission
    for ( i=0; i<(ssd1306_lcdwidth*ssd1306_lcdheight/8); i+=16 )
		{
      for (x=1; x<=16; x++)
				buff[x] = *p++;

			fastI2Cwrite(	buff,  17);
    }
}

// clear everything (in the buffer)
void Adafruit_SSD1306::clearDisplay(void)
{
  memset(poledbuff, 0, (ssd1306_lcdwidth*ssd1306_lcdheight/8));
}
*/
