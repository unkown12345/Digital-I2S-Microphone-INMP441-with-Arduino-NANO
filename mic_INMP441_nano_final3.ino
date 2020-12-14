/* 10.12.2020 NANO with Vcc=3.3V : Digital Mic INMP441 (24Bit)   with DAC MCP4921 12-Bit OUTPUT
 *  
 *  Attention: Take care to put NANO on 3.3V (see picture)  to connect  Mic INMP441 ( dont like U>3.3V ) 
 *              - 
 *  This example reads audio data from an I2S microphone breakout board and OUTPUT samples to  DAC MCP4921.
 *  
 *  Most people means UNO/Nano is to slow for I2S_MIC INMP441
 *  
 *  but there is a Trick toggle PB3 with Timer2 and Timer 0 to save CPU-Time
 *  
 *  
 Feature:
 
 *  NANO-LED signals overdrive 
 *  
 *  The value "gain" reduce or amplifie the Mic-Loudness. 
 *  
 *  gain=1  high sensitive
 *  
 *  gain=6  less sensitive
 *  
 *  I fixed it on gain=3
 *  
 *  Notice: The Mic gives 24-Bit but for me 12 Bit are usefull enough
 *  
 *  There is only one Mic-channel fixed on L/R=Ground ->Left
 *  
 *  Sample-Rate is 25Khz and Clk-Output-PIN is fixed on PORT PB3 to use  NANO-Board.
 *  
 *  There is 12µs you can work with a Sample , here I send it to DAC .
 *  
 *  After SETUP the MIc needs 20000 Samples nearly 1 second to stabilized the I2S-Flow.

     
Circuit:
       * PORTD -MIC 
       * GND,L/R  connected GND
       * mic_Vcc connected 3.3V 
       * mic_WS connected to pin   PD3=D3
       * mic_SCK connected to pin  PB3=D11
       * mic_SD connected to pin   PD4=D4 
       * 
       * PORTC -DAC4921 
       * GND,AVss,LDAC connected GND
       * Vdd, connected 3.3V 
       * VrefA (2µF/1K) connected 3.3V 
       * dac_cs  connected to pin  PC0=A0
       * dac_clk connected to pin  PC1=A1
       * dac_sda connected to pin  PC2=A2 

     
     created 14 December 2020
     by Reini
 
     *  
     Der Sketch verwendet 1478 Bytes (4%) des Programmspeicherplatzes. Das Maximum sind 32256 Bytes.
Globale Variablen verwenden 12 Bytes (0%) des dynamischen Speichers, 2036 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.

*/

byte sample0,sample1,sample2;

void setup() {
//  Serial.begin(115200);
//  Serial.println("Setup I2S Simulation");
 
//------INIT DAC MCP4921-----------------------------

#define dac_sda  0x04     //SDA Data
#define dac_clk  0x02     //Clk Datenübernahme
#define dac_cs   0x01     //CS  Spannung erscheint am Uout-Pin

DDRC= dac_sda|dac_clk|dac_cs; 
PORTC=dac_sda|dac_clk|dac_cs;
//----------------------------------------------


//----------INIT MIC INMP441 -------------------------------------------------
#define mic_WS   0x08     //PD3 PD3
#define mic_SD   0x10     //PD4 PD4
#define mic_SCK  0x08     //PB3 PD11  ClockPin  ! do not change
#define mic_Vcc  0x40     //PD6 PD6   or fixed at 3.3V

     DDRD |=mic_WS|mic_Vcc;

     //Toggle PB3 and Set Start-condition High  (I2S Simulation)
     DDRB |=mic_SCK;
     OCR2A=2;
     pdiEnableTimerClock_H();
     delay(1);
     TCCR2B=(0<<CS22)|(0<<CS21)|(0<<CS20);TCNT2=0; //Timer2 stop ,  Counter 0
     OCR2A=2;                                      //gives 32 Periodes in 17µs

//--------------------------------------------------

//Loudness overdrive
#define LED  0x20     //PB5=D13  LED on Board
DDRB |=LED; PORTB |=LED;

}//----------------------------setup END -----------------------------------------------------------

void loop() {
//byte TCCR0B_=TCCR0B;//save settings delay-Timer0 if needed


byte     umax;     //Loudness overdrive >0
byte     gain=3;   //adjust Loudness 1-6
uint32_t sample;   //32Bit Raw-Data
uint16_t ux;       //16Bit faster

                       PORTD |=mic_Vcc|mic_WS;                           //init start MIC                     

                       TCCR0B=1;                                         //start Timer0 ,  dont use delay() 
                       cli();                                            //reduce Jitter                            

            while(1) { // Read 24Bit-Mic-Data                            Sampling 25Khz

                       READ_mic();                                       // (22,5 µs/mic_WS low)
                       PIND=mic_WS;                                      // (17,5 µs/mic_WS HIGH)
                       pdiEnableTimerClock32();                          // Toggle PB3 
                       
                       //4µs CPU needed
                       umax=0;
                       
                       //12 Bit Data DAC     : 2048 +- 2048 Amplitude  ( 1.6V +- 1.6V )
                       
                       ux=sample1;ux=(ux<<8)|sample2;

                       sample=sample0; sample=(sample<<16)|ux; sample >>=gain;  ux=sample;              
                       
                       if(sample0&0x20)  {if(ux<=0xf800) {ux=0x801;umax=1;} else PORTB &=~LED;}     //neg.Limit
                       
                                         else {if(ux>=0x800) {ux=0x7ff;umax=2;} else PORTB &=~LED;} //pos.Limit
                       
                       ux=ux+0x800;                      //Normalization
                       
                       if(umax) PORTB |=LED;             //signal Loudness overdrive
                   
                       dac(ux);                          //12µs needed MCP4921  12-Bit    
 
                       //PINB =LED;
                       while(TIFR0==0);                  //Sampling-Rate constant with Timer0
                       //PINB =LED;
                       TCCR2B=0;                         //Timer2 OFF
                       pdiDisableTimerClock();           //toggle OFF leave PORT PB3=HIGH
                       
                     }


//TCCR0B=TCCR0B_;sei();

}//----------------------------loop END --------------------------------------------------------------------


            void dac(int k) {                                               //12bit-Voltage needs 12µs
                               byte n=16;
                               PORTC |=dac_sda|dac_clk|dac_cs;              //DAC-default all High
                               k=k&0x0fff;
                               k=k|0x3000;
                               while(n) {
                                         PORTC &=~(dac_sda|dac_clk|dac_cs); //Start with LOW
                                         if(k&0x8000)  
                                         PINC=dac_sda;                      //SDA High Data
                                         PINC=dac_clk;                      //Clk High Clock
                                         k=k << 1;
                                         n--;
                                        }
                               PINC=dac_cs;                                 //CS HIgh   Uout-Pin
                            }


void pdiEnableTimerClock_H()
{
TCCR2A=(1<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(0<<WGM20);//|(1<<WGM22); //SET PB3 High
TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20)|(1<<WGM22);
}

void pdiEnableTimerClock32()
{
TCCR2A=(0<<COM2A1)|(1<<COM2A0)|(1<<WGM21)|(0<<WGM20),//|(1<<WGM22); //START Toggle PB3
TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20)|(1<<WGM22);
TCNT0=0;
TIFR0=0xff;
}

void pdiDisableTimerClock()
{
TCCR2A=(0<<COM2A1)|(0<<COM2A0)|(1<<WGM21)|(0<<WGM20);//|(1<<WGM22);
TCCR2B=(0<<CS22)|(0<<CS21)|(1<<CS20);
PORTB |=mic_SCK;                                                    //STOP Toggle PB3 ,leave CLOCK High
}


void  READ_mic() {                                                  //without loops save CPU-Time
          PIND=mic_WS;
          PINB=mic_SCK;TIFR0=0xff;PINB=mic_SCK;TCNT0=144; 
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;  
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK; 
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK;
          sample0 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample0 |=1; PINB=mic_SCK; 
          while(TIFR0==0);TIFR0=0xff;TCNT0=24;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;  
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;
          sample1 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample1 |=1; PINB=mic_SCK;         
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;  
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;
     
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;      
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK;
          sample2 <<=1;PINB=mic_SCK;if(PIND&mic_SD) sample2 |=1; PINB=mic_SCK; 
          while(TIFR0==0);
/*        I2S Standart but works without , save CPU time   
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
          PINB=mic_SCK;asm("NOP");PINB=mic_SCK;asm("NOP");//PINB=mic_SCK;PINB=mic_SCK; 
*/
}
