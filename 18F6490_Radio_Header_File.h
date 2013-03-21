//
// FILE     fm.h
// DATE     100305
// WRITTEN  RAC
// PURPOSE  Header for PIC firmware .   $
// LANGUAGE MPLAB C18
// KEYWORDS FM/RECEIVE/EXPERIMENT/PROJECT/TEACH/I2C/PIC
// PROJECT  FM TxRx Lab experiment
// CATEGORY FIRMWARE
// TARGET   PIC18F6490
//
//
//
// See also -
//
//  http://read.pudn.com/downloads142/sourcecode/embed/615356/AR1000FSamplev085.c__.htm
//  

// This is skeletal code which won't run until 'fleshed out'.
// It does give an idea of the setup and initialization required
// of the PIC and the FM tuner module.  The basics of communication
// over the I2C bus are also covered.
// 
//
//
#include <delays.h>

#define XS  			0			// Exit success
#define XF				1			// Exit fail

#define FMI2CADR		0x20		// Address (for writes) of FM module on I2C bus


#define DEVRD			0x01		// Read not write an I2C device 
#define FMCHIPVERSADR	0x1C		// Address of FM chip version
#define FMCHIPIDADR		0x1B		// Address of FM chip ID  
#define FMCHIPSTSADR	0x13		// Address of FM chip status

#define FMASKMUTE		0x0001		// Register 1, bit 1
#define FMASKTUNE		0x0200		// Register 2, bit 9
#define FMASKSTATUS		0x0020		// Register 0x13, bit 5
#define FMASKSEEK		0x4000		// Register 3, bit 14
#define FMASKRDCHAN		0xFF80		// Register 2, channel number bits

#define BUTN1			0b00000001	// Button number one
#define BUTN2			0b00000010	// Button number two
#define BUTN3			0b00000100
#define BUTN4			0b00001000
#define BUTN5			0b00010000
#define BUTN6			0b00100000
#define BUTN7			0b01000000
#define BUTN8			0b10000000

#define LCDSEGDP3		22			// LCD segment for decimal point
#define LCDSEGZ 		23			// LCD segment for Z

#define FMHIGHCHAN		(1080-690)	// Highest FM channel number
#define FMLOWCHAN		(875-690)
#define FALSE			0
#define TRUE			1
#define DetectsInARow 	5

//############################ADDITIONALS###########################################
//LCD Control pins
#define rs   		LATDbits.LATD6   		/* PORT for RS */
#define TRIS_rs     TRISDbits.TRISD6    	/* TRIS for RS */
#define en   	    LATDbits.LATD4  		/* PORT for E  */
#define TRIS_en     TRISDbits.TRISD4    	/* TRIS for E  */
//LCD Data pins
#define				lcd_data1 LATDbits.LATD2
#define				lcd_data2 LATDbits.LATD0
#define				lcd_data3 LATEbits.LATE6
#define				lcd_data4 LATEbits.LATE4

//#define				lcd_data1 LATEbits.LATE4
//#define				lcd_data2 LATEbits.LATE6
//#define				lcd_data3 LATDbits.LATD0
//#define				lcd_data4 LATDbits.LATD2

#define			    MAXDIGITS 4

#define ASk_SSK_down	PORTGbits.RG1
#define ASk_SSK_up		PORTGbits.RG3
#define PRESET_1		PORTGbits.RG4	
#define PRESET_2		PORTFbits.RF6
#define PRESET_3		PORTFbits.RF5
#define PRESET_4		PORTFbits.RF4
#define PRESET_5		PORTFbits.RF3
#define PRESET_6		PORTFbits.RF2

#define SEG_CLK 		LATCbits.LATC7
#define SEG_DAT1 		LATFbits.LATF0
#define SEG_DAT2 		LATAbits.LATA2
#define SEG_DAT3 		LATAbits.LATA0
#define SEG_DEC 		LATCbits.LATC0
#define SEG_ONE 		LATAbits.LATA4

#define CTRL_ROTA		PORTCbits.RC5
#define CTRL_ROTB		PORTBbits.RB4
#define CTRL_MUTE		PORTBbits.RB2

//****************************************************************
void lcd_ini();
void dis_cmd(unsigned char);
void dis_data(unsigned char);
void lcdcmd(unsigned char);
void lcddata(unsigned char);
void cleardt();
void line(unsigned char pos);
void Name_s(void);
void freq(unsigned int curr);
void spc(unsigned int dst);
void fqchng(unsigned int chn);
void fqout(void);
void volume();
void mute(void);
void unmute(void);
void vol_cnt (unsigned int vol_val);
void Delay_ms(int d);
// Global error number
int gerr		 = 0;			
unsigned int 	i;
unsigned int 	temp0,temp1,temp2,temp3,temp4,temp5;
unsigned int temp=0x20;
unsigned char buf0;
unsigned int 	curr_stn2[MAXDIGITS];
unsigned int    parameter;

unsigned char buttons;           
unsigned long currentChannel; 
unsigned int tune_type; 
unsigned int mute_value; 



unsigned int vi;			// Index to volume table (0 = quiet, 17 = loud)
unsigned int regImg[18];	// FM register bank images

// Bit field for current debounced pushb state.  0 = not pushed,
unsigned char 	buttons;			
unsigned long 	currentChannel;
unsigned int 	tune_type; 
unsigned int 	mute_value;
	unsigned char txt_Fav1[]="Favourite 1 set ";
	unsigned char txt_Fav2[]="Favourite 2 set ";
	unsigned char txt_Fav3[]="Favourite 3 set ";
	unsigned char txt_Fav4[]="Favourite 4 set ";
	unsigned char txt_Fav5[]="Favourite 5 set ";
	unsigned char txt_Fav6[]="Favourite 6 set ";

	unsigned char txt_Fav_curr1[]=" Playing Fav 1  ";
	unsigned char txt_Fav_curr2[]=" Playing Fav 2  ";
	unsigned char txt_Fav_curr3[]=" Playing Fav 3  ";
	unsigned char txt_Fav_curr4[]=" Playing Fav 4  ";
	unsigned char txt_Fav_curr5[]=" Playing Fav 5  ";
	unsigned char txt_Fav_curr6[]=" Playing Fav 6  ";

//**********************************************************************************

unsigned char ReadI2C( void )
{
if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
  SSPCON2bits.RCEN = 1;           // enable master for 1 byte reception
	Delay1KTCYx(1);
  while ( !SSPSTATbits.BF );      // wait until byte received  
  return ( SSPBUF );              // return with read byte 
}

signed char WriteI2C( unsigned char data_out )
{
  SSPBUF = data_out;           // write single byte to SSPBUF
  if ( SSPCON1bits.WCOL )      // test if write collision occurred
   return ( -1 );              // if WCOL bit is set return negative #
  else
  {
	if( ((SSPCON1&0x0F)!=0x08) && ((SSPCON1&0x0F)!=0x0B) )	//Slave mode only
	{
	      SSPCON1bits.CKP = 1;        // release clock line 
	      while ( !PIR1bits.SSPIF );  // wait until ninth clock pulse received

	      if ( ( !SSPSTATbits.R_W ) && ( !SSPSTATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
	      {
	        return ( -2 );           //return NACK
	      }
		  else
		  {
			return ( 0 );				//return ACK
		  }	
	}
	else if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
	{ 
		Delay1KTCYx(1);
	    while( SSPSTATbits.BF );   // wait until write cycle is complete   
	    IdleI2C();                 // ensure module is idle
	    if ( SSPCON2bits.ACKSTAT ) // test for ACK condition received
	    	 return ( -2 );			// return NACK
		else return ( 0 );              //return ACK
	}
	
  }
}

void OpenI2C( unsigned char sync_mode, unsigned char slew )
{
  SSPSTAT &= 0x3F;                // power on state 
  SSPCON1 = 0x00;                 // power on state
  SSPCON2 = 0x00;                 // power on state
  SSPCON1 |= sync_mode;           // select serial mode 
  SSPSTAT |= slew;                // slew rate on/off 

  I2C_SCL = 1;
  I2C_SDA = 1;
  SSPCON1 |= SSPENB;              // enable synchronous serial port 

}

//#########################################################################################


enum {							// Global error numbers
	GERNONE, 					// No error
	GERWCOL,					// I2C write collision
	GERFINT,					// Could not initialize FM module
	GERFMID						// Could not read chip ID (0x1010)
};

void Init();								// Processor initialisation.
void dly(int d);
unsigned char FMread(unsigned char regAddr, unsigned int *data);
unsigned char FMwrite(unsigned char adr);				// Write a new value to a register
unsigned char FMinit();									// Initialise the chip
unsigned char FMready(unsigned int *rdy);				// Status is ready or busy
unsigned char FMid(unsigned int *id);					// Obtain ID number
void Tune_Up(unsigned int);
void load_fave(unsigned int fave);
void set_fave(unsigned int fave);
void Seek_Up(unsigned int up);
unsigned char nextChan(unsigned char up);
void print_to_screen(unsigned char *text_to_print, unsigned int line_to_print_to);
void cleardt(void);
void serial(char, char, char);
void IntToArray(int, int*, int*);
void LCDdisplay(int freqInput, int deci);

//
// end receiveFM.h ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//
