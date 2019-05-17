#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

#define SET_BIT(PORT,PIN)  PORT |= (1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT &= ~(1<<PIN)

#define START_ADC  ADCSRA |= (1<<ADSC)
#define ENABLE_ADC ADCSRA |= (1<<ADEN)

#define SEAT_BELT_NOT_REQUIRED 0

#define Parking_SW PD7
#define output PD6
#define Adc_Ch0 PC0
#define Adc_Ch1 PC1
#define Adc_Ch2 PC2
#define Adc_Ch4 PC4

#define Low_beam OCR0B=128
#define High_beam OCR0B=250

#define setSpeed(value) OCR0B=value/4

volatile uint8_t EngineFlag=0;
volatile uint8_t Flag2=0;

unsigned int ADC1_data, ADC2_data,ADC4_data;
volatile uint8_t parking_flag=0;

volatile unsigned int SEAT_BELT_FLAG=0; //initializing the flag for switch2_seatbelt
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

int speed=0;
uint8_t check = 1;
uint8_t check1 = 1;
uint8_t autolock = 1;
uint8_t indicator=0;
uint8_t seatbeltcount=0;

void init_LCD()
{

    SET_BIT(DDRB,PB2);    //set as output port-pins
    SET_BIT(DDRB,PB3);    //connected to the lcd
    SET_BIT(DDRB,PB4);
    SET_BIT(DDRB,PB5);

    lcd.begin(16, 2);
}
void ADC_init()
{
    CLR_BIT(PORTC,Adc_Ch0);
    CLR_BIT(PORTC,Adc_Ch1);
    CLR_BIT(PORTC,Adc_Ch2);
    CLR_BIT(PORTC,Adc_Ch4);

    ENABLE_ADC;
    ADMUX |= ((1<<REFS0)); // reference voltage and left adjust
    ADCSRA |= ((1<<ADPS1)|(1<<ADPS2)); // division factor
}

int ADC_Read(char ch)
{
    ADMUX = 0x40 | (ch & 0x07);
    ADMUX|=ch;
    ADCSRA|=(1<<ADSC);
    while((ADCSRA  & 1<<ADSC));
    return ADC;
}

void DoorsLocked()
{
    SET_BIT(PORTD,PD4);
}

void DoorsOpened()
{
    CLR_BIT(PORTD,PD4);
}

void Init_Seatbelt()
{
    CLR_BIT(DDRD,PD0); //Seat Switch
    SET_BIT(PORTD,PD0); //PULLUP Enabled

    SET_BIT(DDRD,PD1);

    PCICR |=(1<<PCIE2);
    PCMSK2 |=(1<<PCINT16);
}
void Init_CentralLockingSystem()
{
    ADC_init();
    SET_BIT(DDRD,PD4); //Right Front

    CLR_BIT(DDRD,PD2);  // Engine Switch
    SET_BIT(PORTD,PD2);  // PULL-UP enabled

    CLR_BIT(DDRD,PD3);  // doorLocking Switch
    SET_BIT(PORTD,PD3);  // PULL-UP enabled

    SET_BIT(DDRD,PD7); //PD6 (OC0A) as output
  
    SREG |= (1<<7); //Global inetrrupt


    EICRA |= ((1<<ISC10)|(1<<ISC00)); //the falling edge will trigger  an interrupt
    EIMSK |= ((1<<INT1)|(1<<INT0));   //PD2=INT0,PD3=INT1;
}
void init_Lights()
{
    CLR_BIT(DDRD,Parking_SW); //Parking_SW
    SET_BIT(PORTD,Parking_SW);//pull up Parking_SW

    SET_BIT(DDRD,output); //PD6 (OC0A) as output


}
void pwm_init()
{

    OCR0A = (255);
    OCR0B = 255;
    TCCR0A|= (1<<WGM01);
    TCCR0B&=~ ((1<<CS01)|(1<<CS00));
    TCCR0B|=  (1<<CS02);
    TIMSK0|= (1<<OCIE0A)|(1<<OCIE0B);
}

int checkTypePressure()
{
    ADC1_data=ADC_Read(1);
    ADC2_data=ADC_Read(2);
    ADC1_data=ADC1_data/20;
    ADC2_data=ADC2_data/20;

    if((ADC1_data>30) && (ADC1_data<35))
    {
        PORTD = 0x00;
        lcd.setCursor(0,0);
        lcd.print(ADC1_data);
        lcd.setCursor(0,1);
        lcd.print("N");

    }
    else if(ADC1_data>35)
    {
        PORTD = 0x00;
        lcd.setCursor(0,0);
        lcd.print(ADC1_data);
        lcd.setCursor(0,1);
        lcd.print("H");
    }
    else
    {
        PORTD = 0x00;
        lcd.setCursor(0,0);
        lcd.print(ADC1_data);
        lcd.setCursor(0,1);
        lcd.print("L");

    }

    if((ADC2_data>30) && (ADC2_data<35))
    {
        PORTD = 0x00;
        lcd.setCursor(4,0);
        lcd.print(ADC2_data);
        lcd.setCursor(4,1);
        lcd.print("N");

    }
    else if(ADC2_data>35)
    {
        PORTD = 0x00;
        lcd.setCursor(4,0);
        lcd.print(ADC2_data);
        lcd.setCursor(4,1);
        lcd.print("H");
    }
    else
    {
        PORTD = 0x00;
        lcd.setCursor(4,0);
        lcd.print(ADC2_data);
        lcd.setCursor(4,1);
        lcd.print("L");

    }
}
uint8_t SteeringIndication()
{
    if(ADC4_data<1024 && ADC4_data>720)
    {
        return 0;
    }
    else if(ADC4_data<360 && ADC4_data>0)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}
int main()
{
    Init_Seatbelt();
    Init_CentralLockingSystem();
    init_LCD();
    init_Lights();
    pwm_init();
    lcd.setCursor(0,0);
    lcd.print("Welcome");
   CLR_BIT(PORTD,PD7);

    while(1)
    {
        if(SEAT_BELT_FLAG==1 || SEAT_BELT_NOT_REQUIRED)
        {
            SET_BIT(PORTD,PD1);
          //_delay_ms(1000);
            if(EngineFlag)
            {      setSpeed(speed);
                 SET_BIT(PORTD,PD7);
                 if(check==1)
                {
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("IGNITION is ON");
                      _delay_ms(1000);      
                    lcd.clear();
                    checkTypePressure();
                   SET_BIT(PORTD,PD7);
                    _delay_ms(1000);
                    check++;
                }
                _delay_ms(1);
                if(Flag2==1 )                      //|| speedFlag==1)
                {
                    DoorsLocked();
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Doors are Locked");
                    lcd.setCursor(0,1);
                    lcd.print("Manually");
                    _delay_ms(1000);
                  autolock=1;

                }
                else
                {
                    if(EngineFlag==1  && Flag2==0)
                    {
                        speed = ADC_Read(0);
                        if(speed>500)
                        {  
                          if(check1==1)
                          {
                            DoorsLocked();
                            lcd.clear();
                            lcd.setCursor(0,0);
                            lcd.print("Doors are Locked");
                            lcd.setCursor(0,1);
                            lcd.print("Automatically");
                            _delay_ms(1000);
                            check1++;
                            autolock=0;
                          } 
                        }
                      if(autolock==1)
                      {
                        lcd.clear();
                        lcd.setCursor(0,0);
                        lcd.print("Doors are unLocked");
                        lcd.setCursor(0,1);
                        lcd.print("parking is ON");
                         _delay_ms(1000);
                        check1=1;
                      }
                    }
                    DoorsOpened();
                }

                speed = ADC_Read(0);
                //lcd.clear();
                //lcd.print(speed);
                setSpeed(speed);

                ADC4_data=ADC_Read(4);
                indicator=SteeringIndication();
                if(indicator==0)
                {
                  lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("LEFT LED ON");
                    _delay_ms(1000);
                }
                else if(indicator==1)
                {
                  lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("RIGHT LED ON");
                    _delay_ms(1000);
                }
                else
                {lcd.clear();
                   // lcd.setCursor(0,0);
                    //lcd.print("LIGHTS ARE OFF");
                    //_delay_ms(1000);
                }
               
            }
            else
            {   CLR_BIT(PORTD,PD7);
             setSpeed(0);
                check=1;
                check1=1;
                autolock=1;
                DoorsOpened();
              lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Engine is OFF");
              _delay_ms(1000);
              Flag2=0;
            }
        }
        else
        { if(EngineFlag==1)
        {
          
               lcd.clear();
               lcd.setCursor(0,0);
               lcd.print("Seat Belt is");
               lcd.setCursor(0,1);
               lcd.print(" Removed");
               CLR_BIT(PORTD,PD1);
               _delay_ms(1000);
               SET_BIT(PORTD,PD1);
               _delay_ms(1000);
          seatbeltcount++;
          if(seatbeltcount==5)
          {
            SEAT_BELT_FLAG=1;
          }
          
          
        }
          CLR_BIT(PORTD,PD1);
        }
            
    }
}
ISR(INT0_vect)                          //engine switch
{
    EngineFlag=!EngineFlag;
}
ISR(INT1_vect) //
{
    if(EngineFlag==1)
        Flag2=!Flag2;
}
ISR(PCINT2_vect)                       //seatbelt
{
    SEAT_BELT_FLAG = !SEAT_BELT_FLAG;
    if( SEAT_BELT_FLAG==1)
    {
        lcd.setCursor(0,0);
        lcd.print("Seatbelt is Worn");
      _delay_ms(1000);
    }
}
ISR(TIMER0_COMPA_vect)
{
    SET_BIT(PORTD,PD6);
}

ISR(TIMER0_COMPB_vect)
{
    CLR_BIT(PORTD,PD6);
}

