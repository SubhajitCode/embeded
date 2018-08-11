/*
   Magentic Levitation
  
   1) connect the digital pin PIN_COIL to the MOSFET gate for electromagnet activation.                                                                  
   2) connect the analog input pin PIN_HALL_SENSOR to the hall sensor signal pin (the other
      two pins shall be wired to gnd and +5V).
   3) (optional) connect the digital pin PIN_CTRL_INDICATOR to the LED that indicates when the
      controller loop is running.
   4) run Arduino, on the serial port the following commands are available:
      - press 's' or 'S' to decrease or increase the setpoint,
      - press 'p' or 'P' to decrease or increase the P gain,
      - press 'd' or 'D' to decrease or increase the D gain,
      - press 'i' or 'I' to decrease or increase the I gain,      
      - press 'x' or 'X' to show the current settings
   5) the system enters in IDLE mode, with a minimum PWM command that allows the orientation of the
      permanent magnet, when in range the system goes in CONTROL mode. When the magnet is too close to
      the coil, the system switches in OFF mode by turning off the PWM command.  

   version 1.0: 23/12/2011 -  E.Assenza   
*/

const int PIN_COIL = 11; // Pins 3 and 11 are connected to Timer 2.
const int PIN_HALL_SENSOR = 1;
const int PIN_CTRL_INDICATOR = 8;
// Coil resistance 3 ohm, max 1 A = 3 Volt max.
const int COIL_MAX_PWM_VALUE = 160;
// Minimum magnetic field when in idle mode (needed to properly orientate the magnet).
const int COIL_IDLE_VALUE = 50;
// Proximity thresholds, when the magnetic field is in this range enters in control mode.
const int MAG_PROX_THRESHOLD_MIN = 585;
const int MAG_PROX_THRESHOLD_MAX = 615;
// Magnetic field limits, when outside this range enters in idle mode.
const int MAG_LIMIT_MIN = 560;
const int MAG_LIMIT_MAX = 700;
// Control loop frequency.
const int CTRL_LOOP_PERIOD_MS = 1;
// Control loop bias value (PWM midpoint)
const int CONTROL_BIAS = 65;
// Set point corresponding to the control bias
const int CONTROL_SETPOINT = 597;
const int MAX_ERR_INTEGRAL = 10000;
// PID controller parameters
const float PGain = 2.1;
const float DGain = 23.6;
const float IGain = 0.001;

const float P_INCR = 0.1; 
const float D_INCR = 0.1; 
const float I_INCR = 0.001; 

// State machine constants.
const byte MODE_OFF = 0; 
const byte MODE_IDLE = 1; 
const byte MODE_CONTROL = 2;

const char     *gAppName = "Arduino Magnetic Levitation";
byte            gMode;
int             gSetPoint;
int             gSensorReadout;
int             gPrevSensorReadout;
int             gPwmCommand;
int             gIntegral;
float           gBias;
float           gP;
float           gD;
float           gI;
unsigned long   gMillisCounter;

void setupCoilPWM()
{  
   // Setup the timer 2 as Phase Correct PWM, 3921 Hz.
   pinMode(3, OUTPUT);
   pinMode(11, OUTPUT);
   // Timer 2 register: WGM20 sets PWM phase correct mode, COM2x1 sets the PWM out to channels A and B.
   TCCR2A = 0;
   TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
   // Set the prescaler to 8, the PWM freq is 16MHz/255/2/<prescaler>
   TCCR2B = 0;
   TCCR2B = _BV(CS21);
}

void writeCoilPWM(uint8_t pin, int val)
{
   if( pin == 3 )
   {
      OCR2B = val;
   }
   else if( pin == 11 )
   {
      OCR2A = val;
   }
   else
   {
      OCR2A = 0;
      OCR2B = 0;
   }
}

void processCommand( int cmd ) 
{ 
   char tmp[64]; 

   switch(cmd) 
   { 
      case 'p': 
         gP -= P_INCR; 
         if( gP < 0 ) gP = 0; 
         break; 
      case 'P': 
         gP += P_INCR; 
         break; 
      case 'd': 
         gD -= D_INCR; 
         if( gD < 0 ) gD = 0; 
         break; 
      case 'D': 
         gD += D_INCR; 
         break; 
      case 'i': 
         gI -= I_INCR; 
         if( gI < 0 ) gI = 0; 
         break; 
      case 'I': 
         gI += I_INCR; 
         break; 
      case 's': 
         gSetPoint --; 
         break; 
      case 'S': 
         gSetPoint ++; 
         break; 
      case 'x': 
      case 'X': 
         sprintf(tmp, "s(%d) c(%d) h(%d) P(%u) D(%u) I(%u,%d)\n", gSetPoint, gPwmCommand, gSensorReadout, (uint16_t)(gP*1000.0), (uint16_t)(gD*1000.0), (uint16_t)(gI*1000.0), gIntegral ); 
         Serial.print(tmp); 
         break; 
      default: 
         break; 
   } 
} 

byte executeOffMode()
{
   digitalWrite(PIN_CTRL_INDICATOR, LOW);
   
   // turn off magnetic field
   gPwmCommand = 0;
   writeCoilPWM(PIN_COIL, gPwmCommand);

   gSensorReadout = analogRead(PIN_HALL_SENSOR);
   
   if( gSensorReadout <= MAG_LIMIT_MIN )
   {
      // Switch to idle mode.
      return MODE_IDLE;
   }
   else
   {
      return MODE_OFF;
   }
}

byte executeIdleMode()
{
   digitalWrite(PIN_CTRL_INDICATOR, LOW);      
  
   // Apply minimum magnetic field.
   gPwmCommand = COIL_IDLE_VALUE;
   writeCoilPWM(PIN_COIL, gPwmCommand);
   
   gSensorReadout = analogRead(PIN_HALL_SENSOR);

   if( (gSensorReadout >= MAG_PROX_THRESHOLD_MIN) && (gSensorReadout <= MAG_PROX_THRESHOLD_MAX) )
   {
      // Switch to control mode.
      gIntegral = 0;
      return MODE_CONTROL;
   }
   else if( gSensorReadout > MAG_LIMIT_MAX )
   {
      return MODE_OFF;
   }
   else
   {
      return MODE_IDLE;   
   }
}

byte checkOutOfLimits(int val)
{  
   if( val <= MAG_LIMIT_MIN ) 
   {
      return MODE_IDLE;
   }
   else if( val >= MAG_LIMIT_MAX )
   {
      return MODE_OFF;
   }
   
   return MODE_CONTROL;
}

byte executeControlMode()
{
   int err;
   int der;
   
   byte mode = MODE_CONTROL;
   
   // Execute control loop at frequency = 1/CTRL_LOOP_PERIOD_MS.
   if( (signed long)( millis() - gMillisCounter ) >= 0)
   {
      gMillisCounter = millis() + CTRL_LOOP_PERIOD_MS;
    
      digitalWrite(PIN_CTRL_INDICATOR, HIGH);
    
      gPrevSensorReadout = gSensorReadout;

      // Read hall sensor.
      gSensorReadout = analogRead(PIN_HALL_SENSOR);
    
      // Check if out of limits (the magnet is out of coil range).
      if( (mode = checkOutOfLimits(gSensorReadout)) != MODE_CONTROL )
      {
         return mode;
      }
        
      err = (gSetPoint - gSensorReadout);
      der = (gPrevSensorReadout - gSensorReadout);
      gIntegral += err;
      gIntegral = constrain( gIntegral, -MAX_ERR_INTEGRAL, MAX_ERR_INTEGRAL);
      	  
      gPwmCommand = gBias + (int)(gP*err) + (int)(gD*der) + (int)(gI*gIntegral);
    
      gPwmCommand = constrain( gPwmCommand, 0, COIL_MAX_PWM_VALUE);

      // Apply output.
      writeCoilPWM(PIN_COIL, gPwmCommand);

      // Write data to serial port.            
      /*
      Serial.write( (uint8_t)0xAF );
      Serial.write( (uint8_t)gSetPoint );
      Serial.write( (uint8_t)(gSetPoint >> 8) );
      Serial.write( (uint8_t)gSensorReadout );
      Serial.write( (uint8_t)(gSensorReadout >> 8) );
      Serial.write( gPwmCommand );
      */
   }
   
   return mode;
}

void setup()
{
   setupCoilPWM();
   
   pinMode(PIN_CTRL_INDICATOR, OUTPUT);
   
   Serial.begin(115200);

   gMillisCounter = 0;
   gMode = MODE_IDLE;
   gSetPoint = CONTROL_SETPOINT;
   gBias = CONTROL_BIAS;
   gSensorReadout = 0;
   gIntegral = 0;
   gP = PGain;
   gD = DGain;
   gI = IGain;
}

void loop()
{
   // User commands.
   if( Serial.available() ) 
   { 
      processCommand( Serial.read() ); 
   } 
   
   // State machine.
   switch( gMode )
   {
      case MODE_OFF:
         gMode = executeOffMode();
         break;
         
      case MODE_IDLE:
         gMode = executeIdleMode();
         break;
         
      case MODE_CONTROL:
         gMode = executeControlMode();
         break;
      
      default:
         break;
   }   
}
