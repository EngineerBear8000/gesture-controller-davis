#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <BleGamepad.h>
#include <HardwareSerial.h>
#include "sbus.h"
#include "math.h"
#include "types.h"

//#define TRAINER_MODE_SBUS
#define TRAINER_MODE_PPM

#define SERIAL1_RX -1
#define SERIAL1_TX 39

#define PIN_THUMB_JOYSTICK_Y 6
#define PIN_II 13
#define PIN_I 12
#define PIN_DROP 11


#ifdef TRAINER_MODE_SBUS
#define SBUS_UPDATE_TASK_MS 15
uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
uint32_t nextSbusTaskMs = 0;
#endif

#ifdef TRAINER_MODE_PPM

#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 8

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum ppmState_e {
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};

void IRAM_ATTR onPpmTimer() {

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = LOW;
    static int usedFrameLength = 0;
    int currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE) {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
    }

    if (ppmState == PPM_STATE_PULSE) {
        ppmOutput = HIGH;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    } else if (ppmState == PPM_STATE_FILL) {
        ppmOutput = LOW;
        currentChannelValue = getRcChannel_wrapper(ppmChannel);
        
        ppmChannel++;
        ppmState = PPM_STATE_PULSE;

        if (ppmChannel > PPM_CHANNELS) {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        } else {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(SERIAL1_TX, ppmOutput);
}
#endif

// BleGamepad bleGamepad;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
dataOutput_t output;
int counter = 0;
float formattedX = 0;
float formattedY = 0;
float formattedZ = 0;

bool isOffset = false;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

float throttleRaw = 0.0;
int lastT[] = {0,0,0,0};

int three_pos = -500;
int drop = -500;


int angleToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 5.0f), -45.0f, 45.0f); //5 deg deadband
    return (int)fscalef(value, -45.0f, 45.0f, -500, 500);
  // return map(angle, -90f, 90f, 500, 500);
}
int joystickToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 0.3f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, 1000, 2000);
}

// void processJoystickAxis(uint8_t axis, uint8_t pin)
// {
//     thumbJoystick.raw[axis] = analogRead(pin);
//     thumbJoystick.zeroed[axis] = thumbJoystick.calibration.zero[axis] - thumbJoystick.raw[axis];

//     if (thumbJoystick.calibration.state == CALIBRATION_DONE)
//     {

//         if (thumbJoystick.zeroed[axis] > thumbJoystick.max[axis])
//         {
//             thumbJoystick.max[axis] = thumbJoystick.zeroed[axis];
//         }

//         if (thumbJoystick.zeroed[axis] < thumbJoystick.min[axis])
//         {
//             thumbJoystick.min[axis] = thumbJoystick.zeroed[axis];
//         }

//         if (thumbJoystick.zeroed[axis] > 0)
//         {
//             thumbJoystick.position[axis] = fscalef(thumbJoystick.zeroed[axis], 0, thumbJoystick.max[axis], 0.0f, 1.0f);
//         }
//         else
//         {
//             thumbJoystick.position[axis] = fscalef(thumbJoystick.zeroed[axis], thumbJoystick.min[axis], 0, -1.0f, 0.0f);
//         }
//     }
// }

int getRcChannel_wrapper(uint8_t channel)
{
  if (channel >= 0 && channel < SBUS_CHANNEL_COUNT)
  {
    return output.channels[channel];
  }
  else
  {
    return DEFAULT_CHANNEL_VALUE;
  }
}

float adjustFloat(float cur, float off)
{
  float adjusted = cur - off;
  if (adjusted < 180.0)
  {
    return adjusted;
  }
  return adjusted - 360.0;
}

void setup(void) 
{
  Serial.begin(115200);
  #ifdef TRAINER_MODE_SBUS
    sbusSerial.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
  #endif
  
  #ifdef TRAINER_MODE_PPM
      pinMode(SERIAL1_TX, OUTPUT);
      timer = timerBegin(0, 80, true);
      timerAttachInterrupt(timer, &onPpmTimer, true);
      timerAlarmWrite(timer, 12000, true);
      timerAlarmEnable(timer);
  #endif
  
  //Serial2.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  // bleGamepad.begin();
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    // Set encoder pins as inputs  
   

  pinMode(PIN_THUMB_JOYSTICK_Y,INPUT);
  pinMode(PIN_II,INPUT_PULLUP);
  pinMode(PIN_I,INPUT_PULLUP);
  pinMode(PIN_DROP,INPUT_PULLUP);

  delay(1000);
    
  bno.setExtCrystalUse(true);
  
}


void loop(void) 
{

    /* Get a new sensor event */ 
    sensors_event_t event;
    bno.getEvent(&event); 
    
    if (!isOffset or counter < 1000)
    {
      counter ++;
      offsetX = event.orientation.x;
      // offsetY = event.orientation.y;
      // offsetZ = event.orientation.z;
      isOffset = true;
    }
    // throttleRaw = analogRead(PIN_THUMB_JOYSTICK_Y);
    if(!digitalRead(11)){
     drop = 500;
    }

    else{
      drop = -500;
    }

    if(!digitalRead(13)){
      Serial.println("II");
      three_pos = 500;
    }
    else if(!digitalRead(12)){
      Serial.println("I");
      three_pos = -500;
    }
    else{
      Serial.println("O");
      three_pos = 0;
    }

    for(int i=3 ; i >= 0; i--)
    {
      lastT[i] = lastT[i-1];
    }
    lastT[0] = analogRead(PIN_THUMB_JOYSTICK_Y);

    throttleRaw = (lastT[0] + lastT[1] +lastT[2]+ lastT[3])/40;

    formattedX = adjustFloat(event.orientation.x, offsetX);
    formattedY = event.orientation.y;
    formattedZ = event.orientation.z;
    // formattedY = adjustFloat(event.orientation.y, offsetY);
    // formattedZ = adjustFloat(event.orientation.z, offsetZ);

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(formattedX, 4);
    // Serial.print("\tY: ");
    // Serial.print(formattedY, 4);
    // Serial.print("\tZ: ");
    // Serial.print(formattedZ, 4);
    // Serial.println("");

//    f
      float opX = formattedX; //+ to - 90
      float opY = formattedY; //roll 
      float opZ = formattedZ;  //pitch
      // float opX = map(formattedX, -180, 180,90,-90);// yaw
      // float opY = map(formattedY, -180, 180,90,-90);// roll
      // float opZ = map(formattedZ, -180, 180, -90, 90); // pitch
      float opT = fscalef(throttleRaw, 130.0f, 409.0f, -1.0, 1.0); // throttle
      
    //bleGamepad.setAxes(formattedX, formattedY, formattedZ, 0, 0, 0, 0, 0);
    // (x axis, y axis, z axis, rx axis, ry axis, rz axis, slider 1, slider 2)
    
    // bleGamepad.setHat1(HAT_DOWN_RIGHT);
    // All axes, sliders, hats etc can also be set independently. See the IndividualAxes.ino example

    output.channels[ROLL] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(opY);
    output.channels[PITCH] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(opZ);
    output.channels[YAW] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(opX); // why is it minus?
    output.channels[THROTTLE_T] = joystickToRcChannel(opT);
    output.channels[4] = 2000; //arm
    output.channels[6] = 1500; //alt hold
    // output.channels[6] = three_pos + DEFAULT_CHANNEL_VALUE;  //
    output.channels[5] = DEFAULT_CHANNEL_VALUE + drop; //drop
    
    // Serial.println();
    /* Display the formatted floating point data */
   Serial.print("X: ");
   Serial.print(angleToRcChannel(opX));
   Serial.print("\tY: ");
   Serial.print(angleToRcChannel(opY));
   Serial.print("\tZ: ");
   Serial.print(angleToRcChannel(opZ));
   Serial.print("\tT: ");
  //  Serial.print(throttleRaw);
   Serial.println(joystickToRcChannel(opT));
   
   Serial.print("\toffset: ");
   Serial.print(offsetX);
   Serial.print("\tizz_offset: ");
   Serial.print(isOffset);
   Serial.println("");                                                                                                       


    for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
      output.channels[i] = constrain(output.channels[i], 1000, 2000);
    }
  #ifdef TRAINER_MODE_SBUS
    sbusPreparePacket(sbusPacket, false, false, getRcChannel_wrapper);
    sbusSerial.write(sbusPacket, SBUS_PACKET_LENGTH);

    nextSbusTaskMs = millis() + SBUS_UPDATE_TASK_MS;
  
  #endif
  
    delay(10);
  // }
  
  
}
