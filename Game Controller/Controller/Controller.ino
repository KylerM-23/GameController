#include <XInput.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

#define BIP 7                               //The button interrupt pin is at 7
#define MuxEN 5                             //Mux Enable pin is 5
#define BSP1 12                             //Button Select Pin 1
#define BSP0 11                             //Button Select Pin 0
#define LBR 10                              //Left (DPAD) Button Read
#define RBR 9                               //Right (Face) Button Read
#define GLED 4                              //Green LED of RGB
#define BLED 13                             //Blue LED of RGB

const boolean InvertLeftYAxis   = false;    //Do not invert the two analog Y axis
const boolean InvertRightYAxis  = false;  
const int ADC_Max = 1023;                   //10 bit ADC
const int Pin_LeftJoyX  = A1;               //Define the locations of the Joy Stick Pins 
const int Pin_LeftJoyY  = A0;
const int Pin_RightJoyX = A4;
const int Pin_RightJoyY = A3;
const int Pin_ButtonBack  = A2;
const int Pin_ButtonStart = A5;

int LEDB = 0;                               //Variables for the values of the
int LEDG = 0;                               //Blue and Green LEDs
unsigned long long LEDTime = 0;             //Cooldown time for LEDs

void Buttons(int, int, int);                //Declare Button Read & Press function
void ButtonDetect(void);                    //button press ISR
void StartSelect();                         //read start & select prototype
void JoyStickRead();                        //read joystick prototype
void updateLEDs(int, int);                  //update the color on the RGB LED

char Active = 0;                            //variable saying that a button is actively being pressed
int Over = 0;                               //variable that keeps the state of button press, if over = 1, it means no more buttons are pressed
int results[4] = {0, 0, 0, 0};              //array to store the results of button reads
int buttons[8] = {DPAD_DOWN, DPAD_UP, DPAD_RIGHT, DPAD_LEFT, BUTTON_Y, BUTTON_B, BUTTON_X, BUTTON_A };  //array of all the DPAD and Face buttons
char temp = 0;                              //temporary variable
int leftJoyX, leftJoyY, rightJoyX, rightJoyY;//Variable for joystick values
boolean invert, buttonBack, buttonStart;    //Variable for inverting, and Start & Back Buttons

//Fan Stuff
const int PWM = 6;                          //PWM Signal Pin
const int FC = 8;                           //Fan Control Pin for MOSFET
float TempF = 0.0;                          //variable for temperature
const word PWM_FREQ = 25000;                //Frequency for PWM
const word TCNT1_TOP = 16000000/(2*PWM_FREQ);//Set register for proper PWM Freq.
Adafruit_MLX90614 mlx = Adafruit_MLX90614();//Create instance of Temp sensor

void setup() {
  noInterrupts();                           //disable interrupts
  pinMode(BSP1, OUTPUT);                    //set up pins to inputs or outputs
  pinMode(BSP0, OUTPUT);
  pinMode(LBR, INPUT);
  pinMode(RBR, INPUT);
  pinMode(BIP, INPUT);
  pinMode(MuxEN, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(FC, OUTPUT);
  pinMode(Pin_ButtonBack, INPUT_PULLUP);
  pinMode(Pin_ButtonStart, INPUT_PULLUP);
  interrupts();                           //Enable interrupts
  mlx.begin();                            //Start temp sensor
  analogWrite(PWM, 0);                    //Set Fan Speed To Low
  digitalWrite(FC, LOW);                  //Turn Off Fan

  XInput.setJoystickRange(0, ADC_Max);    //Set joystick range to the ADC
  attachInterrupt(digitalPinToInterrupt(BIP), ButtonDetect, RISING);  //set the button interrupt ISR to ButtonDetect and trigger at rising edge
  XInput.begin();                         //Begin XINPUT
  Active = 0;                             //Clear Active
  Over = 0;                               //Clear Over
}

void loop()
{
  StartSelect();                          //Read Start & Select Buttons
  JoyStickRead();                         //Read Joysticks
  
  if (Active | Over)                      //Check if we are in the active or ending state
  {
    if (Over) Over = 0;                   //if we ending the button press, clear the over var
    Buttons(5, 4, 1);                     //Read DPAD output at PORTB, Bit 5 (LBR), and Disable Right MUX
    Buttons(6, 0, 0);                     //Read Face output at PORTB, Bit 6 (RBR), and Enable Right MUX
  }
  TempF = mlx.readObjectTempF();          //read Temperature

  //Nonie
  if (TempF < 80.0)                       //if temp is less than 80F
  {
    analogWrite (PWM, 0);                 //Fan Speed At Slowest
    digitalWrite (FC, LOW);               //Turn Off Fan
  }
  else if (TempF < 100)                   //if temp is greater than or equal to 80 but less than 100
  {
    analogWrite (PWM, 255*(TempF-80)/20); //scale speed between 0 and 255 depending on the temperature
    digitalWrite (FC, HIGH);              //turn on fan
  }
  else                                    //otherwise
  {
    analogWrite (PWM, 255);               //max fan speed
    digitalWrite (FC, HIGH);              //turn on fan
  }
}

void StartSelect()
{
  buttonBack  = !digitalRead(Pin_ButtonBack);                           //Read the logic state of the back button
  buttonStart = !digitalRead(Pin_ButtonStart);                          //Read the logic state of the start button
  XInput.setButton(BUTTON_BACK, buttonBack);                            //Update controller logic states
  XInput.setButton(BUTTON_START, buttonStart);
  if(buttonBack | buttonStart) updateLEDs(buttonBack, buttonStart);     //Update LEDs
}

void updateLEDs(int st, int sl)
{
  if (millis() > LEDTime + 750)           //If the time since last update has been 750ms
  {
    LEDTime = millis();                   //Reset LEDTime
    if (st) LEDG = ~LEDG;                 //If Back Pressed, invert the Green Color
    if (sl) LEDB = ~LEDB;                 //If Start Pressed, invert the Blue Color
    digitalWrite (GLED, LEDG);            //Write the updated color values
    digitalWrite (BLED, LEDB);            
  }
}

void JoyStickRead()
{
  leftJoyX = analogRead(Pin_LeftJoyX);                                    //Read ADC Value of left X & left Y
  leftJoyY = analogRead(Pin_LeftJoyY);                                    
  XInput.setJoystickX(JOY_LEFT, leftJoyX);                                //Update X & Y  Left Joystick
  XInput.setJoystickY(JOY_LEFT, leftJoyY, invert);                        

  rightJoyX = analogRead(Pin_RightJoyX);                                  //Read ADC Value of right X & right Y
  rightJoyY = analogRead(Pin_RightJoyY);                                  
  XInput.setJoystickX(JOY_RIGHT, rightJoyX);                              //Update X & Y  Right Joystick
  XInput.setJoystickY(JOY_RIGHT, rightJoyY, invert);                      
}

void ButtonDetect()
{
  detachInterrupt(digitalPinToInterrupt(BIP));                            //disable interrupt
  if (Active)                                                             //if we are in the active mode
  {
    attachInterrupt(digitalPinToInterrupt(BIP), ButtonDetect, RISING);    //Reenable interrupt with rising
    Over = 1;                                                             //Enter the over mode
  }
  else attachInterrupt(digitalPinToInterrupt(BIP), ButtonDetect, FALLING);//Enable interrupt with falling to detect all buttons not pressed
  Active = ~Active;                                                       //Invert active
}


void Buttons(int BRB, int offsetNum, int EN)
{
  temp = 0;                                                   //clear temp
  bitWrite(PORTC, 6, EN);                                     //write to portC bit 6, MuxEN, to enable the mux
  delay(5);                                                   //small delay
  for (int i = 0; i < 2; i++)                                 //loop through all input select combinations on mux
  {
    for (int j = 0; j < 2; j++)
    {
      bitWrite(PORTD, 6, i);                                  //write to PORTD bit 6, BSP1
      bitWrite(PORTB, 7, j);                                  //write to PORTB bit 7, BSP0
      delay(5);                                               //small delay
      results[temp] = bitRead(PINB, BRB);                     //save the logic state at PINB, bit BRB to results
      temp++;                                                 //increase temp var
    }
  }

  for (int k = 0; k < 4; k++)                                 //Loop through all buttons
  {
    if (results[k]) XInput.press(buttons[k + offsetNum]);     //if logic 1 at pin, get the button from the array and press
    else XInput.release(buttons[k + offsetNum]);              //other wise let go of the button
  }
}
