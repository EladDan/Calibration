#include <MegunoLink.h>
#include <CommandHandler.h>
#include <CommandProcessor.h>
#include <SoftwareSerial.h>

// Sensors Pins definition
# define Lout_Pin 2                     // OUT of TSL235R out-connected to Digital pin 2 (interrupt 0)
# define Lin_Pin 3                      // OUT of TSL235R in -connected to Digital pin 3 (interrupt 1)

// DIM Pins (PWM) definition
# define RedPin 5            // PWM pin 5 for DIM Red LEDS 665 nm
# define BluePin 6           // PWM pin 6 for DIM Blue LEDS 452 nm

const int PWMmin = 20;       // Minimal Power for lighting up

boolean IsSetup;          // used in Cmd_UpdateIndicators() & ConPWMCalc()
String comment = "zero algae";     // added algal volume
String ColMix;

// Constants
unsigned long period_out = 10000;      // Milliseconds of each light frequency measurement
unsigned long period_in = 2000;       // Milliseconds of each light frequency measurement
// period_x type changed to 'unsigned long' to avoid warning in comparison between
// (currentMillis-MillisStart) with period_x in pulse_x counting
// const float factor = 1.5;          // Proportion factor uW/cm2/Hz for white;
// for red (635 nm) =1.72 ; for blue (450 nm) = 1.12

unsigned long currentMillis ;
unsigned long MillisStart ;
volatile unsigned long pulses_out ;    // Counter of measurements of the TSL235R out
volatile unsigned long pulses_in ;     // Counter of measurements of the TSL235R in

float Lo;            // the readings from the TSL light out in Hz
float Losu;          // sum of 5 replicates
float Lom;           // mean of 5 replicates

float Li;            // the readings from the TSL light in in Hz
float Lisu;          // sum of 10 replicates
float Lim;           // mean of 10 replicates

int j;
int i;

int RedPWM;
int BluePWM;

// ***************************************************************************************************

// INTERFACE
// Communication with the Interface Panel (Panel to Arduino)
CommandHandler <10> SerialCommandHandler; // memory reserved for up to 10 Commands

InterfacePanel Panel;                     // used for Monitor & Control

//"LogFile" = "CSV" = the target message channel (remember to select this in megunolink)
Message MyCSVMessage("CSV");


// ****************** format for the TimePlot ; value: Light outputs 1 and 2, Temp, pH **************
void TimePlot(float value, String seriesName, String channelName, int digits)
{
  Serial.print("{TIMEPLOT:");
  Serial.print(channelName);
  Serial.print("|data|");
  Serial.print(seriesName);
  Serial.print("|T|");
  Serial.print(value, digits);
  Serial.println("}");
}
//**************************************************************************************************

void setup()
{
  Serial.begin(9600);   // Enable the hardware serial port
  setupLout();          // Set up Lightout
  setupLin();           // Set up Lightin
  setupPWMLEDS();       // Set up PWM pins
  delay(1000);

  IsSetup = true;

  //Initialize TimePlots, CommandHandler(Interface Panel Functions) & Graphic Indicator for New Run
  SetupMLP();
  delay(500);

  PrintCSV();                           // using IsSetup
  comment = "";

  IsSetup = false;

}

// ***************************** Setup Light Intensity out ******************************************
void setupLout()
{
  pinMode(Lout_Pin, INPUT);                  // Declare the TSL pin as input
  digitalWrite(Lout_Pin, HIGH);
  attachInterrupt(0, PulseCount_out, RISING);
}
// **************************************************************************************************

// ***************************** Setup Light Intensity in *******************************************
void setupLin()
{
  pinMode(Lin_Pin, INPUT);                    // Declare the TSL pin as input
  digitalWrite(Lin_Pin, HIGH);
  attachInterrupt(1, PulseCount_in, RISING);  // Interrupt 1 for L_in
}
// **************************************************************************************************

void setupPWMLEDS()
{
  pinMode(RedPin, OUTPUT);    // Declare the Red DIM pin as output
  pinMode(BluePin, OUTPUT);   // Declare the Blue DIM pin as output
}
// ***************************************************************************************************
// ************************ SUB: SetupMLP() ******************************************
void SetupMLP()
{
  //Interface Panel Functions
  // E.D: added, invoke from the MLP when we send the right command from setup
  SerialCommandHandler.AddCommand(F("SetComment"), Cmd_SetComment);

}  // End of SetupMLP()
// *************************************************************************************

void loop()
{
  SerialCommandHandler.Process();  // Read from SerialCommandHandler, receive commands

  for (j = 1; j < 3; j++) // number of variations
  {
    for (i = PWMmin; i < 251; i = i + 10) // Light intensity change
    {
      if (j == 1)// Variation 1, RED ONLY, FrR = 1
      {
        ColMix = "FrR = 1.0";
        RedPWM = constrain(i, PWMmin, 250);
        BluePWM = 0;
      }
      if (j == 2) // Blue only
      {
        ColMix = "FrR = 0";
        RedPWM = 0;
        BluePWM = constrain(i, PWMmin, 250);
      }
      //      if (j == 2)// 1RED AND 1BLUE
      //      {
      //        ColMix = "FrR = 0.75";
      //        RedPWM = constrain(i, PWMmin, 250);
      //        BluePWM = constrain(i, PWMmin, 250);
      //      }
      //      if (j == 3)// FrR = 0.50
      //      {
      //      ColMix = "FrR = 0.50";
      //      RedPWM = constrain(0.2823*i+47.5, PWMmin, 250);
      //      BluePWM = constrain(i, PWMmin, 250);
      //      }
      //      if (j == 4)// FrR = 0.25
      //      {
      //      ColMix = "FrR = 0.25";
      //      RedPWM = constrain(0.0941*i+63.74, PWMmin, 250);
      //      BluePWM = constrain(i, PWMmin, 250);
      //      }
      //      if (j == 5)// BLUE ONLY
      //      {
      //        ColMix = "FrR = 0";
      //        RedPWM = 0;
      //        BluePWM = constrain(i, PWMmin, 250);
      //      }

      analogWrite(RedPin, RedPWM);   // send DIM value for Red
      analogWrite(BluePin, BluePWM); // send DIM value for Blue

      // Period conditioning routine
      if ((RedPWM + BluePWM) < (2 * PWMmin + 15)) // Low light intensity (on LI sensor) Li < ~30 Hz: Lo < ~1 Hz
      {
        period_out = 10000;
        period_in = 5000;
      }
      else if ((RedPWM + BluePWM) < (2 * PWMmin + 45)) // Medium light intensity: Li < ~300 Hz: Lo < ~30 Hz
      {
        period_out = 5000;
        period_in = 2000;
      }
      else                   //High light intensity
      {
        period_out = 3000;
        period_in = 500;
      }


      TimePlot(RedPWM, "RedPWM", "chName", 0);
      delay(100);                       //just here to slow down the output
      TimePlot(BluePWM, "BluePWM", "chName", 0);
      delay(100);                       //just here to slow down the output


      // *********** read and plot the instantaneous values from the light sensors *****************
      GetLight();

      delay(200);               // wait 0.2 second until the next measurement


      // *********************** Send data to MegunoLink TimePlot *****************************************
      TimePlot(Lom, "Lo", "chName", 2); // lightLevel,seriesName,channelName,nb digits
      delay(100);                       //just here to slow down the output
      TimePlot(Lim / 1000.0, "Li", "chName", 5); // lightLevel,seriesName,channelName,nb digits
      delay(100);                          //just here to slow down the output

      // Send data to Serial monitor
      PrintCSV();                           // using !IsSetup
    }    // close for i

    analogWrite(RedPin, 0);   // turn off Red
    analogWrite(BluePin, 0);  // turn off Blue
    delay(30000);             // Allow 30 seconds (decay) for separating the [R, B, R+B] data sets
  }    // close for j
}    // close loop
// **************************************************************************************************

void PulseCount_out()
{
  pulses_out ++;
  currentMillis = millis();    // update currentMillis
}

void PulseCount_in()
{
  pulses_in ++;
  currentMillis = millis();    // update currentMillis
}

void getLight_out()
{
  if (pulses_out < 1)     // Error handling
  {
    Serial.print("Error: pulses_out = ");
    Serial.println(pulses_out);
    pulses_out = 1;
  }
  unsigned long TempPulseO = pulses_out;
  unsigned long MillisDelayed = millis() - MillisStart;
  Lo = TempPulseO / ( MillisDelayed / 1000.0); // Calculate the frequency (pulses/second)
}

void getLight_in()
{
  if (pulses_in < 1)     // Error handling
  {
    Serial.print("Error: pulses_in = ");
    Serial.println(pulses_in);
    pulses_in = 1;
  }
  unsigned long TempPulseI = pulses_in;
  unsigned long MillisDelayed = millis() - MillisStart;
  Li = TempPulseI / ( MillisDelayed / 1000.0); // Calculate the frequency (pulses/second)
}

// *********************** SUB: get Light intensities from 4 sensors ***************************
void GetLight()
{
  Losu = 0.0;
  Lisu = 0.0;

  for (int k = 1; k < 4; k++)
  {
    MillisStart = millis();
    pulses_out = 0;            // reset the pulses_out counter

    delay(period_out);

    getLight_out();          // Request to measure the frequency

    Losu = Losu + Lo;
    delay(200);
  }
  Lom = Losu / 3;

  delay(200);

  for (int k = 1; k < 4; k++)
  {
    MillisStart = millis();
    pulses_in = 0;            // reset the pulses_out counter

    delay(period_in);

    getLight_in();          // Request to measure the frequency

    Lisu = Lisu + Li;
    delay(200);
  }

  Lim = Lisu / 3;
  delay(200);
}

void Cmd_SetComment(CommandParameter & Parameter)
{
  comment = Parameter.NextParameter();

  Serial.print("## comment set to ");
  Serial.println(comment);
  Panel.SetText("CommentTextBox", "");
}
// *********************************************************************************************


// *************************** SUB: PrintCSV() *****************************************
void PrintCSV()  //CSV File setup:(ColMix, dRed , dBlue, Lom, Po, Lim, Pi, Lo/Li, comment)
{
  MyCSVMessage.Begin();
  delay(5);

  if (IsSetup)
  {
    Serial.println("Program NEW3LCALIB.ino");
    Serial.print("Added Algal volume = ");
    Serial.println(comment);
    Serial.println("ColMix, dRed , dBlue, Lom, Po, Lim, Pi, Lo/Li, comment");    // CSV file headers for Initial Parameters
  }
  else
  {
    Serial.print(ColMix);
    Serial.print(", ");
    Serial.print(RedPWM);
    Serial.print(", ");
    Serial.print(BluePWM);
    Serial.print(", ");
    Serial.print(Lom, 3);
    Serial.print(", ");
    Serial.print(period_out);
    Serial.print(", ");
    Serial.print(Lim, 3);
    Serial.print(", ");
    Serial.print(period_in);
    Serial.print(", ");
    Serial.print(Lom / Lim, 9);
    Serial.print(", ");
    Serial.print(comment);
  }
  MyCSVMessage.End();
} // End PrintCSV()
// ******************************************************************************





