#include <Bounce.h>
#include <SPI.h>


// Only need to specify the digital pin used for ChipSelect
// all other pins (CLK,MOSI, MISO) are fixed by the SPI library
const int SPI_CS=10;

// scrolling speed analog input pin
const int INPUT_SCROLLSPEED=0;

// push button (digital) input pin
const int SWITCH_MODE_PIN=2;

Bounce bouncer = Bounce(SWITCH_MODE_PIN,50);

enum DisplayModes { 
  DISPLAY_MODE_MESSAGESCROLL=0,
  DISPLAY_MODE2,
  NB_DISPLAY_MODES
};

enum ScrollPhase {
  SCROLL_MESSAGE_NORMAL=0,
  SCROLL_MESSAGE_HOLD,
  SCROLL_MESSAGE_UP
};

#define MESSAGE_HOLD_TIME 1000

// Number of daisy-chained 8x8 matrices
#define NB_MODULES 16

// buffer for SPI communication
byte spidata[16*2];

// buffer for LED states
byte ledData[16*8];

//the opcodes for the MAX7219 SPI commands
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

// Buffer for text message to be displayed
#define MAX_MESSAGE_LENGTH 64
char message[16+MAX_MESSAGE_LENGTH+1];

// actual delay used between display refresh
int delaytime=20; // 20ms

// specific delay value when scrolling the text up and out of the display
#define DELAYTIME_SCROLLUP 100

// specific delay value when scrolling some text 
#define DELAYTIME_SCROLL 20

// Data buffers and state for Bluetooth communication
String commandString = ""; 
String remoteDataString = ""; 
boolean commandStarted = false;
boolean remoteDataAvailable = false;  
boolean commandComplete = false; 

// display mode state
int displayMode = DISPLAY_MODE_MESSAGESCROLL;

// to keep track of how long message has been held on display
long messageHoldTimeStart=0;

int loopindex = 0;
int scroll_mode = SCROLL_MESSAGE_NORMAL;
int verticalscroll_loop=0;
int current_messagestartpixel=0;

// 8x8 font definition : data is given line by line for each character
// 1 byte (8 values...) per character line
byte alphabet[][8] = {
{// Space
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
}, {// !
0x30,0x78,0x78,0x30,0x30,0x00,0x30,0x00
}, {// "
0x6c,0x6c,0x6c,0x00,0x00,0x00,0x00,0x00
}, {// #
0x6c,0x6c,0xfe,0x6c,0xfe,0x6c,0x6c,0x00
}, {// $
0x30,0x7c,0xc0,0x78,0x0c,0xf8,0x30,0x00
}, {// %
0x00,0xc6,0xcc,0x18,0x30,0x66,0xc6,0x00
}, {// &
0x38,0x6c,0x38,0x76,0xdc,0xcc,0x76,0x00
}, {// '
0x60,0x60,0xc0,0x00,0x00,0x00,0x00,0x00
}, {// (
0x18,0x30,0x60,0x60,0x60,0x30,0x18,0x00
}, {// )
0x60,0x30,0x18,0x18,0x18,0x30,0x60,0x00
}, {// *
0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00
}, {// +
0x00,0x30,0x30,0xfc,0x30,0x30,0x00,0x00
}, {// ,
0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x60
}, {// -
0x00,0x00,0x00,0xfc,0x00,0x00,0x00,0x00
}, {// .
0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00
}, {// /
0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00
}, {// 0
0x7c,0xc6,0xce,0xde,0xf6,0xe6,0x7c,0x00
}, {// 1
0x30,0x70,0x30,0x30,0x30,0x30,0xfc,0x00
}, {// 2
0x78,0xcc,0x0c,0x38,0x60,0xcc,0xfc,0x00
}, {// 3
0x78,0xcc,0x0c,0x38,0x0c,0xcc,0x78,0x00
}, {// 4
0x1c,0x3c,0x6c,0xcc,0xfe,0x0c,0x1e,0x00
}, {// 5
0xfc,0xc0,0xf8,0x0c,0x0c,0xcc,0x78,0x00
}, {// 6
0x38,0x60,0xc0,0xf8,0xcc,0xcc,0x78,0x00
}, {// 7
0xfc,0xcc,0x0c,0x18,0x30,0x30,0x30,0x00
}, {// 8 
0x78,0xcc,0xcc,0x78,0xcc,0xcc,0x78,0x00
}, {// 9
0x78,0xcc,0xcc,0x7c,0x0c,0x18,0x70,0x00
}, {// :
0x00,0x30,0x30,0x00,0x00,0x30,0x30,0x00
}, {// ;
0x00,0x30,0x30,0x00,0x00,0x30,0x30,0x60
}, {// <
0x18,0x30,0x60,0xc0,0x60,0x30,0x18,0x00
}, {// =
0x00,0x00,0xfc,0x00,0x00,0xfc,0x00,0x00
}, {// > 
0x60,0x30,0x18,0x0c,0x18,0x30,0x60,0x00
}, {// ?
0x78,0xcc,0x0c,0x18,0x30,0x00,0x30,0x00
}, {// @
0x7c,0xc6,0xde,0xde,0xde,0xc0,0x78,0x00
}, {// A
0x30,0x78,0xcc,0xcc,0xfc,0xcc,0xcc,0x00
}, {// B
0xfc,0x66,0x66,0x7c,0x66,0x66,0xfc,0x00
}, {// C
0x3c,0x66,0xc0,0xc0,0xc0,0x66,0x3c,0x00
}, {// D
0xf8,0x6c,0x66,0x66,0x66,0x6c,0xf8,0x00
}, {// E
0xfe,0x62,0x68,0x78,0x68,0x62,0xfe,0x00
}, {// F
0xfe,0x62,0x68,0x78,0x68,0x60,0xf0,0x00
}, {// G
0x3c,0x66,0xc0,0xc0,0xce,0x66,0x3e,0x00
}, {// H
0xcc,0xcc,0xcc,0xfc,0xcc,0xcc,0xcc,0x00
}, {// I
0x78,0x30,0x30,0x30,0x30,0x30,0x78,0x00
}, {// J
0x1e,0x0c,0x0c,0x0c,0xcc,0xcc,0x78,0x00
}, {// K
0xe6,0x66,0x6c,0x78,0x6c,0x66,0xe6,0x00
}, {// L
0xf0,0x60,0x60,0x60,0x62,0x66,0xfe,0x00
}, {// M
0xc6,0xee,0xfe,0xfe,0xd6,0xc6,0xc6,0x00
}, {// N
0xc6,0xe6,0xf6,0xde,0xce,0xc6,0xc6,0x00
}, {// O
0x38,0x6c,0xc6,0xc6,0xc6,0x6c,0x38,0x00
}, {// P
0xfc,0x66,0x66,0x7c,0x60,0x60,0xf0,0x00
}, {// Q
0x78,0xcc,0xcc,0xcc,0xdc,0x78,0x1c,0x00
}, {// R
0xfc,0x66,0x66,0x7c,0x6c,0x66,0xe6,0x00
}, {// S
0x78,0xcc,0xe0,0x70,0x1c,0xcc,0x78,0x00
}, {// T
0xfc,0xb4,0x30,0x30,0x30,0x30,0x78,0x00
}, {// U
0xcc,0xcc,0xcc,0xcc,0xcc,0xcc,0xfc,0x00
}, {// V
0xcc,0xcc,0xcc,0xcc,0xcc,0x78,0x30,0x00
}, {// W
0xc6,0xc6,0xc6,0xd6,0xfe,0xee,0xc6,0x00
}, {// X
0xc6,0x44,0x6c,0x38,0x38,0x6c,0xc6,0x00
}, {// Y
0xcc,0xcc,0xcc,0x78,0x30,0x30,0x78,0x00
}, {// Z
0xfe,0xc6,0x8c,0x18,0x32,0x66,0xfe,0x00
}, {// [
0x78,0x60,0x60,0x60,0x60,0x60,0x78,0x00
}, {// \
0xc0,0x60,0x30,0x18,0x0c,0x06,0x02,0x00
}, {// ]
0x78,0x18,0x18,0x18,0x18,0x18,0x78,0x00
}, {// ^
0x10,0x38,0x6c,0xc6,0x00,0x00,0x00,0x00
}, {// _
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff
}, {// `
0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00
}, {// a
0x00,0x00,0x78,0x0c,0x7c,0xcc,0x7c,0x00
}, {// b
0x60,0x60,0x60,0x7c,0x66,0x66,0x7c,0x00
}, {// c
0x00,0x00,0x78,0xcc,0xc0,0xcc,0x78,0x00
}, {// d
0x0c,0x0c,0x0c,0x7c,0xcc,0xcc,0x7c,0x00
}, {// e
0x00,0x00,0x78,0xcc,0xfc,0xc0,0x78,0x00
}, {// f
0x38,0x6c,0x60,0xf0,0x60,0x60,0xf0,0x00
}, {// g
0x00,0x00,0x7c,0xcc,0xcc,0x7c,0x0c,0xf8
}, {// h
0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x00
}, {// i
0x30,0x00,0x70,0x30,0x30,0x30,0x78,0x00
}, {// j
0x0c,0x00,0x0c,0x0c,0x0c,0x0c,0x6c,0x38
}, {// k
0x60,0x60,0x66,0x6c,0x78,0x6c,0x66,0x00
}, {// l
0x70,0x30,0x30,0x30,0x30,0x30,0x78,0x00
}, {// m
0x00,0x00,0xcc,0xfe,0xfe,0xd6,0xc6,0x00
}, {// n
0x00,0x00,0xf8,0xcc,0xcc,0xcc,0xcc,0x00
}, {// o
0x00,0x00,0x78,0xcc,0xcc,0xcc,0x78,0x00
}, {// p
0x00,0x00,0x7c,0x66,0x66,0x7c,0x60,0x60
}, {// q
0x00,0x00,0x7c,0xcc,0xcc,0x7c,0x0c,0x0c
}, {// r
0x00,0x00,0xdc,0x76,0x66,0x60,0xf0,0x00
}, {// s
0x00,0x00,0x7c,0xc0,0x78,0x0c,0xf8,0x00
}, {// t
0x10,0x30,0x7c,0x30,0x30,0x34,0x18,0x00
}, {// u
0x00,0x00,0xcc,0xcc,0xcc,0xcc,0x7c,0x00
}, {// v
0x00,0x00,0xcc,0xcc,0xcc,0x78,0x30,0x00
}, {// w
0x00,0x00,0xc6,0xd6,0xfe,0xfe,0x6c,0x00
}, {// x
0x00,0x00,0xc6,0x6c,0x38,0x6c,0xc6,0x00
}, {// y
0x00,0x00,0xcc,0xcc,0xcc,0x7c,0x0c,0xf8
}, {// z
0x00,0x00,0xfc,0x98,0x30,0x64,0xfc,0x00
}, {// {
0x1c,0x30,0x30,0xe0,0x30,0x30,0x1c,0x00
}, {// |
0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00
}, {// }
0xe0,0x30,0x30,0x1c,0x30,0x30,0xe0,0x00
}, {// ~
0x76,0xdc,0x00,0x00,0x00,0x00,0x00,0x00
}, {// DEL
0x00,0x10,0x38,0x6c,0xc6,0xc6,0xfe,0x00
}
};

// Swap the font rows/columss.
void rotateFont()
{
  byte newval[8];
  
  for (int letter=0; letter < sizeof(alphabet)/sizeof(byte[8]); letter++)
  {
   for (int column=0; column<8; column++)
   {
     newval[column] = 0;
     for (int bit=0; bit<8; bit++) 
     {
	newval[column] += ((alphabet[letter][7-bit] & (1 << 7-column)) << column)>> bit;
     }   
   } 
   
   // Store rotated character back into original buffer
   for (int k=0; k<8; k++)
   {
     alphabet[letter][k] = newval[k];
   }   
    
  }
}

// Send the shutdown register value to all modules
void shutdown(bool b) {

  byte val = b ? 0 : 1; 
  digitalWrite(SPI_CS,LOW);

   for (int i = 0; i<NB_MODULES;i++)
   {
     SPI.transfer(OP_SHUTDOWN);
     SPI.transfer(val);
   }

  digitalWrite(SPI_CS,HIGH);
}

// Send the scan limit register value to all modules
void setScanLimit(byte limit) {

  digitalWrite(SPI_CS,LOW);
   for (int i = 0; i<NB_MODULES;i++)
   {
     SPI.transfer(OP_SCANLIMIT);  
     SPI.transfer(limit);
   }

  digitalWrite(SPI_CS,HIGH);
}

// Send the intensity register value to all modules
void setIntensity(byte intensity) {

  digitalWrite(SPI_CS,LOW);

   for (int i = 0; i<NB_MODULES;i++)
   {
     SPI.transfer(OP_INTENSITY);
     SPI.transfer(intensity);
   }

  digitalWrite(SPI_CS,HIGH);
}

// Send the decode mode register value to all modules
void setDecodeMode(byte mode) {

  digitalWrite(SPI_CS,LOW);

   for (int i = 0; i<NB_MODULES;i++)
   {
     SPI.transfer(OP_DECODEMODE);
     SPI.transfer(mode);
   }

  digitalWrite(SPI_CS,HIGH);
}

// Send the displayTest register value to all modules
void setDisplayTest(bool b) {

  byte val = b ? 1 : 0;
  digitalWrite(SPI_CS,LOW);

   for (int i = 0; i<NB_MODULES;i++)
   {
     SPI.transfer(OP_DISPLAYTEST);
     SPI.transfer(val);
   }

  digitalWrite(SPI_CS,HIGH);
}

void setup() {
  
  // Bluetooth communication setup
  Serial.begin(9600);
  remoteDataString.reserve(64);    
  commandString.reserve(64); 
  
  // Put spaces in the NB_MODULES first characters, so that when scrolling, 
  // the actual text starts to appear from the right
  strcpy(message, ""); 
  for(int k=0; k<NB_MODULES;k++)
    strcat(message, " "); 
  
  // the pin to which the press button is attached is an input
  pinMode(SWITCH_MODE_PIN, INPUT);  
  
  // the ChipSelect pin is an output
  pinMode (SPI_CS, OUTPUT);
  
  // initialize SPI library
  SPI.begin();

  // Select fastest possible speed: 8 MHz (16Mhz main frequency, divided by 2)
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // make sure the display is not is test mode
  setDisplayTest(false);

  // set the minimum intensity (bright enough, and minimizes current draw)
  setIntensity(0);
  
  // Initialize LED state buffer to all off.
  for(int i=0; i< 16*8; i++)
    ledData[i] = 0;
  
  // and perform an initial setup of these LED states.
  customTransfer(ledData);

  // Activate all 8 rows of each matrix
  setScanLimit(7);
  
  // Turn off any data decoding
  setDecodeMode(0);
  
  // transpose rows/columns of all characters of the in-ram font, to better match
  // the way data will be sent to the modules.
  rotateFont();
  
  // Need to explicitly tell the modules to wake-up from the sleep state they were in at power-up.
  shutdown(false);
}
 
void customTransfer(byte* data) {

   // one byte for command + one byte for data for each module.
   int bytesToSend=NB_MODULES*2;
   
   // Update column x for each module as part of a single data transfer, 
   // and iterate over 8 columns to cover the whole display 
   for (int column=0; column<8; column++)
   {       
      for(int matrix=0;matrix<NB_MODULES;matrix++)
      { 
         // First fill the opcode : the index of the column to be updated + 1
         spidata[matrix*2+1]=column+1;
         // Then fill the value for this column  
         spidata[matrix*2]=ledData[8*matrix+column];
      }
      
      //enable the module data reception by driving chip select low 
      digitalWrite(SPI_CS,LOW);
      
      //Now shift out the data over the MOSI line, using the SPI library call
      for(int i=bytesToSend;i>0;i--)
        SPI.transfer(spidata[i-1]);

      //drive the chip select line back to high, so that modules take the received data into account.
      digitalWrite(SPI_CS,HIGH);
    }
}    
 
// Turn on an individual pixel at coordinates (x,y)
void setPixel(int x, int y, bool state)
{
  // boundary checks
  if (x<0 || x>127 || y<0 || y>7) return;
  
  // figure out where to put the '1' in the column
  byte val = 0b10000000 >> (7-y);
  
  // then turn on/off this bit in column x
  if (state)
  {
    ledData[x]  |= val;  
  }
  else
  {
    val = ~val;
    ledData[x]  &= val;  
  }
}

// Draw a single character over 8 columns, starting from column x
void drawChar(int x, byte character)
{
  if (x > 127) return;
  for(int column=0; column<8;column++)
  {
    if((x+column>=0) && (x+column < 128))
      ledData[x+column] = alphabet[character][column];
  }
}  

void nextDisplayMode()
{
  // If leaving message scroll mode, reset some parameters for a 
  // clean restart once we switch back to this mode
  if (displayMode == DISPLAY_MODE_MESSAGESCROLL)
  {
    scroll_mode = SCROLL_MESSAGE_NORMAL;
    current_messagestartpixel = 0;
  }
  // switch to next mode, and loop to first mode if needed.
  displayMode = (displayMode+1)%NB_DISPLAY_MODES;
       

  Serial.print("Display mode is now "); 
  Serial.println(displayMode);   
}

void loop() { 
     
    if(bouncer.update())
    {
      if(bouncer.read()==0)
      {
        nextDisplayMode();
      }
    }
  
//////////////////////////////////////////////////////////////////////
// Prepare new data to be displayed, depending on current display mode
//////////////////////////////////////////////////////////////////////
  switch(displayMode) {
    
    case DISPLAY_MODE_MESSAGESCROLL:
     
       switch(scroll_mode) {
         
         // scrolling message
         case SCROLL_MESSAGE_NORMAL:
           for(int i=0; i< 16*8; i++)
             ledData[i] = 0;
             
           for (int letter=0; letter<NB_MODULES+1; letter++)
           {
              drawChar(-current_messagestartpixel%8+8*letter, message[current_messagestartpixel/8+letter]-' ');
           }
           delaytime = DELAYTIME_SCROLL;
           
           // scroll by 1 pixel
           current_messagestartpixel++;
           
           // if we reached the end of the message (i.e. the NB_MODULES last characters
           // of the message have been displayed), switch to hold mode to keep the message
           // displayed as is for a while
           if (current_messagestartpixel > (strlen(message)-NB_MODULES)*8) 
           {
              scroll_mode = SCROLL_MESSAGE_HOLD;
              messageHoldTimeStart = millis();
           }
           break;
         
         // just keep current message as is
         case SCROLL_MESSAGE_HOLD:
           
           // If we reached the end of the hold time, go to next mode: 
           // scrolling the message up & out of the display
           if(millis() > messageHoldTimeStart + MESSAGE_HOLD_TIME) 
           {
              scroll_mode = SCROLL_MESSAGE_UP;
              
              // reset the start point of scrolling
              current_messagestartpixel = 0;
              
              // reset the vertical scroll index
              verticalscroll_loop = 0;
           }
           break;
         
         // scrolling message up and out of the display
         case SCROLL_MESSAGE_UP:
           
           // Just shift the data bits "vertically" (i.e. to the right inside each column)
           for(int i=0; i< 16*8; i++)
             ledData[i] = ledData[i] >> 1;
             
           verticalscroll_loop++;
           
           // Once all 8 rows have been shifted up, the message has completely disappeared,
           // and we can go back to normal display mode.
           if (verticalscroll_loop > 7) scroll_mode = SCROLL_MESSAGE_NORMAL;
           
           delaytime = DELAYTIME_SCROLLUP;
           break;
       }   

       break;
    case DISPLAY_MODE2:    
 
     delaytime = DELAYTIME_SCROLL;
     for(int i=0; i< 16*8; i++)
       ledData[i] = 0;
       
     // Draw a little wavy snake
     for (int p=0; p<10;p++)
     {
       int x = loopindex%128-p;
       if (x < 0) break;
       setPixel(x, 4+3.5*sin(5*2*3.14159*(x)/128), true);   
     }
     break;
    default:
     break;
  }

/////////////////////////////
// Send out the data over SPI
/////////////////////////////
   customTransfer(ledData);
 
///////////////////////////////////////////////////////
// Read analog input to adjust scroll speed dynamically
///////////////////////////////////////////////////////
   int speed = analogRead(INPUT_SCROLLSPEED);
   int actualdelay = delaytime*(0.1+9.9*speed/1023);
  
/////////////////////////////////////////////
// Wait a bit before the next display refresh
/////////////////////////////////////////////  
  delay(actualdelay);
 
/////////////////////////////////////////////////////////////
// Keep track of the current loop index. Allow it to rollover
/////////////////////////////////////////////////////////////
  loopindex++;
  
/////////////////////////////////////////////////////////
// Check if new cmd/data has been received over Bluetooth 
/////////////////////////////////////////////////////////    
  if (remoteDataAvailable) {
//    Serial.print("Got the command:"); 
//    Serial.println(commandString); 
//    Serial.print("Got the data:"); 
//    Serial.println(remoteDataString); 

    if(commandString.compareTo("update_message")==0)
    {
      Serial.println("Updating message.");
      // Put the received string into the actual message buffer.
      // Note: toCharArray(buf,len) only copies up len-1 bytes, since this
      // is usually intended to work passing sizeof(<target char array>). 
      // It also puts a trailing \0 char in buf[len]
      // So request to write string length + 1 to get the full message into our
      // message buffer.
      // Also, start writing in message buffer at offset NB_MODULES to not overwrite
      // the NB_Modules first "space" characters that are used to get a clean start of scroll
      int size = remoteDataString.length();
      if (size > MAX_MESSAGE_LENGTH) size = MAX_MESSAGE_LENGTH;
      remoteDataString.toCharArray(message+NB_MODULES, size+1);
      
      // We updated the message, therefore let's restart scrolling from the beginning 
      // of the message
      current_messagestartpixel = 0;
  
    //Serial.print("updateText="); 
    //Serial.println(message);
    }
    else if(commandString.compareTo("next_mode")==0)
    {
      Serial.println("Switching to next mode");
      nextDisplayMode();
    }
    // else just ignore this command and wait for a new one.
    else 
    {
      Serial.println("Unknown command");
    }
    
    // clear the string:
    remoteDataString = "";
    commandString = "";
    commandStarted = false;
    remoteDataAvailable = false;
    commandComplete = false;
  }   
}

/*
called automatically everytime loop() runs.
checks if data is available on the serial input, to which the bluetooth module 
is connected. If so, store the input in a buffer and interpret incoming characters 
as they come : a properly formatted message is of the form : @<cmd>@<data>@
 */
void serialEvent() {
         
  while (Serial.available()) {
    
    char inChar = (char)Serial.read();
    
    // If we receive the first @, flag that reception of a new command is starting 
    if (inChar == '@' && !commandStarted)
    {
      commandStarted = true;
    } 
    // If this is the second @, then the command part of the message is complete
    else if (inChar == '@' && commandStarted && !commandComplete)
    {
      commandComplete = true;
    }
    // if this is the third @, then the data has been received completely too
    // we can then notify the main loop
    else if (inChar == '@' && commandStarted && commandComplete)
    {
      remoteDataAvailable = true;
    }
    // in all other cases, just append the received character into a buffer
    // (the command buffer or the data buffer depending on where we are in the
    // reception sequence.
    else
    {
      if (!commandComplete)
        commandString += inChar;
      else
        remoteDataString += inChar;
    }
  }
}
