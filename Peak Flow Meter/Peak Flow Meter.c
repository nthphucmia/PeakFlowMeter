#include <mega32a.h>
#include <delay.h>


// Declare your global variables here
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define ADC_VREF_TYPE 0x00


unsigned char High_byte, Low_byte;
unsigned char USART_packet[4]={0};
volatile unsigned int ADC[102]={0};
unsigned int ADC_read(unsigned char ADC_CHANNEL);
unsigned int max;
unsigned int  voltage;
unsigned int index=0;
volatile unsigned int Byte_flag,ADC_analog;

// Function Prototypes USART
void UART_putChar(char c);
void Init_UART(void);
     
//Declare LCD
#define   LCD_RS  PORTB.0
#define   LCD_RW  PORTB.1 //PORTA1
#define   LCD_E   PORTB.2 //PORTA2   // enable LCD 

#define   LCD_B4  PORTB.4 //PORTA4
#define   LCD_B5  PORTB.5 //PORTA5
#define   LCD_B6  PORTB.6 //PORTA6
#define   LCD_B7  PORTB.7 //PORTA7

#define   LCD_data_out(data) (PORTB = (PORTB&0x0F)|((data<<4)&0xF0))

#define   MODE_4_BIT     0x28
#define   CLR_SCR        0x01
#define   DISP_ON        0x0C
#define   CURSOR_ON      0x0E
#define   CURSOR_HOME    0x80
#define   CURSOR_LINE1   0x80
#define   CURSOR_LINE2   0xC0

// Function Prototypes LCD
//=============================================================
void LCD_Write_Nibble(char byte);
void LCD_Wait_Busy();
void LCD_Write_Cmd(char cmd);
void LCD_Write_Data(char chr);
void LCD_Putc(char c);
void LCD_GotoXY(unsigned char x, unsigned char y);
void LCD_Init();
void LCD_Write_Int(unsigned int integer );
void LCD_Printf(char *str);
void LCD_Write_Int_analog(unsigned int integer );
void LCD_Printf(char *str);

interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
TCNT1H=0xFF;    // sample rate 100Hz: 
TCNT1L=0x94;    // first number 108 
ADC_analog=ADC_read(0);
Byte_flag=1;
} 


// Declare your global variables here

void main(void)
{
unsigned char i;
//analog signal, INPUT
PORTA=0x00;
DDRA=0x00;

// Declare LCD 
PORTB=0x00;
DDRB=0x00;

// Port D initialization, RX and Tx UART
PORTD=0x00;
DDRD=0x00;

LCD_Init(); 
//LCD_Printf("PEAK FLOW:"); 
Init_UART();
// Timer/Counter 1 initialization
// Clock source: System Clock
//// Clock value: 10.800 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x05;
TCNT1H=0xff;
TCNT1L=0x94;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
SFIOR=0x00;

// ADC initialization
// ADC Clock frequency: 691.200 kHz
// ADC Voltage Reference: AREF pin
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x84;

// Timer(s)/Counter(s ) Interrupt(s) initialization
TIMSK=0x04;

// Global enable interrupts
#asm("sei")
LCD_Printf("PEAK FLOW VALUE:"); 

while(1)
{       
        // luu du lieu vao mang
        do
        {
              if(Byte_flag==1)
                {
                    ADC[index]=ADC_analog; 
                    LCD_GotoXY(1,1);  
                    LCD_Write_Int_analog(ADC[index]); 
                    Byte_flag=0; 
                } 
                index++;
        }
        while(index<102);    
                   
       //tim gia tri     
        for (index=0;index<102;index++)
        {
    
            if (ADC[index]>=max) 
                max=ADC[index];   
        } 
       index=0; 
       //truyen gia tri max 
       LCD_GotoXY(6,1); 
       LCD_Write_Int_analog(max); 
       delay_ms(1);  
            
     // voltage=(unsigned int)(((1023*500)/1024));
     // voltage=(unsigned int)((max*5)/1024);
      voltage=(unsigned int)((max*49)/100);    
      LCD_GotoXY(11,1); 
      LCD_Write_Int(voltage);  
      delay_ms(10); 
      LCD_GotoXY(15,1); 
      LCD_Putc('V');
      //NUT RESET   
      
             
      //USART TRANSMIT, each time transmit 8 bits
        USART_packet[0]=0x55;
        USART_packet[1]=0xaa; 
        High_byte=(max&0xff00)>>8; 
        Low_byte=(max&0x00ff);
        USART_packet[2]=High_byte;
        USART_packet[3]=Low_byte; 
            
      for (i=0;i<4;i++)
          {
                 UART_putChar(USART_packet[i]);
          }
}          
}             

// READ ADC VALUE
unsigned int ADC_read(unsigned char ADC_CHANNEL)
{
//  chon dien ap tham chieu va kenh can doc
ADMUX= ADC_VREF_TYPE|ADC_CHANNEL;
delay_us(10);
// Start conversation
ADCSRA|=0x40;    
// wait for conversation end  (ADIF bit=1)   
while ((ADCSRA&0x10)==0);
//   ADC  WORD = ADCH+ADCL 
ADCSRA|=0x10;
return ADCW ; 
}
//=============================================================
//Define Function
//=============================================================
void Init_UART(void)
{
// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: Off
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600
UCSRA=0x00;
UCSRB=0x08;
UCSRC=0x06;
UBRRH=0x00;
UBRRL=0x47;
}
//=================================================================
// Get a character from the USART Receiver

//=================================================================
//#pragma used+ is mean the following function after that will 
//overwrite standard printf etc. basic routine (redirect to UART)
// Write a character to the USART1 Transmitter
//  UART_putChar is already declared in library of this complier
//  #pragma used+ and #pragma used-  overwrite standard in macr0
#pragma used+
void UART_putChar(char c)
{
    while ((UCSRA & DATA_REGISTER_EMPTY)==0);
    UDR=c;
}
#pragma used-
//=================================================================

void LCD_Putc(char c)
{
   switch (c)
   
        //without "break" can't continue next step
   {  
      case '\f'   :  LCD_Write_Cmd(CLR_SCR);
                     delay_us(2);
                     break;   
                     
      // xuong dong               
      case '\n'   :  LCD_Write_Cmd(CURSOR_LINE2); 
                     break;
      
      // dich con tro
      case '\b'   :  LCD_Write_Cmd(0x10);  
                     break;
     
      default     :  LCD_Write_Data(c);     
                     break;
   }
}

void LCD_Printf(char *str)
{
    int i=0;
    while(str[i]!='\0')      // loop will go on till the NULL character in the string
    {
        delay_us(50);
        LCD_Putc(str[i]);    // sending data on LCD byte by byte
        i++;
    }
}

void LCD_Write_Int(unsigned int integer )
{
    unsigned char thousands,hundreds,tens,ones; 
    
//    thousands = integer / 1000;
//    LCD_Write_Data(thousands + 0x30);    
    
    
    hundreds = (integer) / 100;
    LCD_Write_Data(hundreds + 0x30);   
    
    LCD_Write_Data('.'); 
    
    tens=(integer%100)/10;
    LCD_Write_Data(tens + 0x30);
    
    ones=integer%10;
    LCD_Write_Data(ones + 0x30);
}

void LCD_Write_Int_analog(unsigned int integer )
{
    unsigned char thousands,hundreds,tens,ones; 
    
    thousands = integer / 1000;
    LCD_Write_Data(thousands + 0x30);    
    
    
    hundreds = ((integer - thousands*1000)) / 100;
    LCD_Write_Data(hundreds + 0x30);   
     
    
    tens=(integer%100)/10;
    LCD_Write_Data(tens + 0x30);
    
    ones=integer%10;
    LCD_Write_Data(ones + 0x30);
}


void LCD_Write_Nibble(char byte)
{
    LCD_E = 1;
    LCD_data_out(byte);
    LCD_E = 0;
}
// wait + read LCD
void LCD_Wait_Busy()
{
    unsigned char tempH,tempL;
    PORTB |= 0xF0;          // SET 4 BIT DATA (thap) duoc su dung
    LCD_RS = 0;             // 0 = Instruction input, 1 = Data input        
    LCD_RW = 1;             // 0 = Write to LCD module,1 = Read from LCD module       

    do 
    {   // che do 4 bit se truyen va nhan nibble cao truoc (thap sau)
        LCD_E = 1;          //che do 4 bit phai doc 2 lan 
        delay_us(2);
        DDRB  = 0x0f;        //set 4 bit cao cua PORTD LAM input     (doc ve)
        tempH = PINB;       //read in upper nybble  
        DDRB  = 0xff;       //out put (xuat ra LCD)
        LCD_E = 0;          //busy_flag
        delay_us(2);  
       // ==============================
       //không có phan nay khong hien LCD
        LCD_E = 1;                  
        delay_us(2);
        DDRB  = 0x0f;
        tempL = PINB;       //read in lower nybble.  
        DDRB  = 0xff;                    
        LCD_E = 0;          //BF = 1 is busy
        delay_us(2);
    } while (tempH&0x80);   //bit cuoi la busy
}

void LCD_Write_Cmd(char cmd)
{
    LCD_Wait_Busy();
    LCD_RS = 0;               // 0 = Instruction input
    LCD_RW = 0;               // 0 = Write to LCD module
    LCD_Write_Nibble(cmd>>4);  // send high byte first
    LCD_Write_Nibble(cmd);
}

void LCD_Write_Data(char chr)
{ 
    LCD_Wait_Busy();
    LCD_RS = 1;                    // 1 = Data input
    LCD_RW = 0;                    // 0 = Write to LCD module
    LCD_Write_Nibble(chr>>4);     // nibble cao truoc
    LCD_Write_Nibble(chr);    
}

void LCD_Init()
{
    LCD_RS = 0;
    LCD_RW = 0;
    LCD_Write_Nibble(MODE_4_BIT>>4);   //lan dau tien truyen du lieu, LCD mac dinh la` 8 bit
    LCD_Write_Cmd(MODE_4_BIT);         //nen phai set function 2 lan de dieu chinh theo che do
    LCD_Write_Cmd(DISP_ON);            //mong muon: 4 bit, 2 line, 5x7 dot
    //LCD_Write_Cmd(CURSOR_ON);        // tat con 
    LCD_Write_Cmd(CLR_SCR);
}


void LCD_GotoXY(unsigned char x, unsigned char y)
{
    if(x<40) 
    
    {   
    //Sets the specified value (AAAAAA) into the address counter
      if(y) x |= 0x40;    //0x40:  64D
      // di chuyen toi vi tri mong muon neu y=0 (DDRAM)
      x |=0x80; 
      LCD_Write_Cmd(x);
    }
}


