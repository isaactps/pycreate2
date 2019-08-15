#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMegaPi.h>
#include <PN532_HSU.h>
#include <PN532.h>

//Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeUltrasonicSensor ultrasonic_10(8);
MeLineFollower linefollower_9(7);

//MeRGBLed rgbled_0(0, 12);
//Me7SegmentDisplay seg7_6(6);

//MeSerial SWSerial(PORT_6);

PN532_HSU pn532hsu(Serial3);
PN532 nfc_reader(pn532hsu);

long currentTime = 0;
long lastTime = 0;
long Line_Fail_Timer = 0;
long prev_tagTime = 0;
long curr_tagTime = 0;
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
void EncoderStop();
double rotation;
double degreesPerInch;
double rpm;
void Initialize();
double turnDegreeL;
double turnDegreeR;
void LeftSquare();
void RightSquare();
void Reverse(double inches, double speed);
void forward(double inches, double speed);
void TurnLeft(double degrees, double speed);
void TurnRight(double degrees, double speed);
float factor = 0.996;
float factor1 = 0.950; //for right motor forward motion correction
float factor2 = 0.950; //for right motor backward motion correction
bool RxFlag;
bool LineFlag;
bool ForwardFlag;
bool ReverseFlag;
bool RightFlag;
bool LeftFlag;
bool Line_Fail_Flag;
bool Ckpt_Flag;
long KeepAliveTime = 0;
short low_speed = 0;
short high_speed = 0;
int velocity = 0;
short low_distance = 0;
short high_distance = 0;
short distance = 0;
short low_radius = 0;
short high_radius = 0;
short radius = 0;
short low_angle = 0;
short high_angle = 0;
short angle = 0;
short low_ckpt = 0xFF;
short high_ckpt = 0xFF;
short cmd_ckpt = 0xFFFF;
unsigned short Obstacle_ID = 0;
unsigned char OBSTACLE;
uint8_t tag_read_success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 }; // Buffer to store the returned UID
uint8_t uidLength = 4; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
union input_data 
{
   byte command[8];
   byte indat[8];
}inp;

enum LF_State {
  OFF_TRACK, // Both IR detectors on the track
  RIGHT_OFF, // Right IR detector is off the track
  LEFT_OFF,  // Left IR detector is off the track
  ON_TRACK   // Both IR receiver are off the track
};

enum LF_State LF_status;

uint8_t ckpt_uid[][4] = {
  {0xF0, 0xFA, 0xFB, 0xFC},
  {0xE7, 0xE7, 0x08, 0xB6},
  {0xE7, 0x3B, 0x09, 0xB6},
  {0xF7, 0xEC, 0x09, 0xB6},
  {0xA7, 0x37, 0x09, 0xB6},
  {0x13, 0x32, 0x5F, 0x1B},
  {0x13, 0x26, 0x7F, 0x1B},
  {0x5D, 0x52, 0x62, 0x5B},
  {0xED, 0x0B, 0xF9, 0xA3},
  {0x5D, 0x17, 0xBF, 0xA3},
  {0x3D, 0xFD, 0xF8, 0xA3},
  {0xAD, 0x6C, 0xF1, 0xA3},
  {0x31, 0xDE, 0x48, 0x1E},
  {0xCD, 0xEC, 0x9D, 0x5B},
  {0x7D, 0x28, 0x18, 0x75},
  {0xED, 0x54, 0x63, 0x5B},
  {0x11, 0xD1, 0x54, 0x1E},
  {0x29, 0xF8, 0xF2, 0x62},
  {0x79, 0x2B, 0x3B, 0xEA},
  {0x89, 0xCE, 0x3C, 0xEA},
  {0x82, 0x04, 0x67, 0x6F},
  {0xF2, 0x8B, 0x5C, 0x6F},
  {0x02, 0x7F, 0x5F, 0x6F},
  {0x99, 0x18, 0x39, 0xEA},
  {0x02, 0xBF, 0x5B, 0x6F},
  {0x69, 0x83, 0xCD, 0xEB} 
  
};

void isr_process_encoder1(void)
{
      if(digitalRead(Encoder_1.getPortB()) == 0){
            Encoder_1.pulsePosMinus();
      }else{
            Encoder_1.pulsePosPlus();
      }
}

void isr_process_encoder2(void)
{
      if(digitalRead(Encoder_2.getPortB()) == 0){
            Encoder_2.pulsePosMinus();
      }else{
            Encoder_2.pulsePosPlus();
      }
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void move(int direction, int speed)
{
      int leftSpeed = 0;
      int rightSpeed = 0;
      
      if(direction == 1){
            leftSpeed = -speed*factor1;
            rightSpeed = speed;
      }else if(direction == 2){
            leftSpeed = speed*factor2;
            rightSpeed = -speed;
      }else if(direction == 3){
            leftSpeed = -speed;
            rightSpeed = -speed;
      }else if(direction == 4){
            leftSpeed = speed;
            rightSpeed = speed;
      }
      Encoder_1.setTarPWM(leftSpeed);
      Encoder_2.setTarPWM(rightSpeed);
}

void moveDegrees(int direction,long degrees, int speed_temp)
{
      speed_temp = abs(speed_temp);
      
      if(direction == 1)
      {
            Encoder_1.move(-degrees*factor,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
      else if(direction == 2)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
      }
      else if(direction == 3)
      {
            Encoder_1.move(-degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
      }
      else if(direction == 4)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
}

void EncoderStop(void)
{
    _delay(0.2);
    while(!((((Encoder_1.getCurrentSpeed())==(0))) && (((Encoder_2.getCurrentSpeed())==(0)))))
    {
        _loop();
    }
}

void Reverse(double inches, double speed = 150)
{
    rotation = (degreesPerInch) * (inches);
    moveDegrees(2,rotation,speed);
    EncoderStop();
}

void Reverse_NonStop(double speed = 150)
{
    move(2,speed);
}

void Forward(double inches, double speed = 150)
{
    rotation = (degreesPerInch) * (inches);
    moveDegrees(1,rotation,speed);
    EncoderStop();
}

void Forward_NonStop(double speed = 150)
{
    move(1,speed);
}

void Initialize(void)
{
    rpm = 150;
    degreesPerInch = 70.73;
    turnDegreeL = 5.450; //5.7;
    turnDegreeR = 5.450; //5.7;
}

void LeftSquare(void)
{
    for(int __i__=0;__i__<4;++__i__)
    {
        Forward(20,rpm);
        _delay(0.5);
        TurnLeft(90,rpm);
    }
}

void RightSquare(void)
{
    for(int __i__=0;__i__<4;++__i__)
    {
        Forward(20,rpm);
        _delay(0.5);
        TurnRight(90,rpm);
    }
}

void LineFollowing(void)
{
  if(linefollower_9.readSensors() == 3.000000){
    LF_status = ON_TRACK;
    move(1,50/100.0*255);
    Line_Fail_Flag = false;
  }

  if(linefollower_9.readSensors() == 2.000000){
    LF_status = RIGHT_OFF;
    Encoder_1.setTarPWM(-1*50/100.0*255);
    Encoder_2.setTarPWM(0/100.0*255);
    Line_Fail_Flag = false;
  }

  if(linefollower_9.readSensors() == 1.000000){
    LF_status = LEFT_OFF;
    Encoder_1.setTarPWM(-1*0/100.0*255);
    Encoder_2.setTarPWM(50/100.0*255);
    Line_Fail_Flag = false;
  }

  if(linefollower_9.readSensors() == 0.000000){
    LF_status = OFF_TRACK;
    move(2,35/100.0*255);
    if(Line_Fail_Flag == false){
      Line_Fail_Flag = true;
      Line_Fail_Timer = millis()/1000; 
    }
  }
}

void TurnRight(double degrees, double speed = 150)
{
  rotation = (degrees) * (turnDegreeR);
  Encoder_1.move(rotation,abs(speed));
  Encoder_2.move(rotation,abs(speed));
  EncoderStop();
}

void TurnRight_NonStop(double speed = 150)
{
  move(4,speed);
}

void TurnLeft(double degrees, double speed = 150)
{
  rotation = (degrees) * (turnDegreeL);
  Encoder_1.move((rotation) * (-1),abs(speed));
  Encoder_2.move((rotation) * (-1),abs(speed));
  EncoderStop();
}

void TurnLeft_NonStop(double speed = 150)
{
  move(3,speed);
}

void StopMoving(void)
{
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    EncoderStop();   
}

void Obstacle_Avoidance(void)
{
    TurnRight(90,rpm);
    Forward(16,rpm);
    TurnLeft(90,rpm);
    Forward(32,rpm);
    TurnLeft(90,rpm);
    Forward(16,rpm);
    TurnRight(45,rpm);
    StopMoving();
}

void _loop(){
    Encoder_1.loop();
    Encoder_2.loop();
}

void setup(){
    
    Serial.begin(115200);
    //SWSerial.begin(115200);
    Serial2.begin(115200);
    
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    Encoder_1.setPulse(9);
    Encoder_1.setRatio(39.267);
    Encoder_1.setPosPid(1.8,0,1.2);
    Encoder_1.setSpeedPid(0.18,0,0);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    Encoder_2.setPulse(9);
    Encoder_2.setRatio(39.267);
    Encoder_2.setPosPid(1.8,0,1.2);
    Encoder_2.setSpeedPid(0.18,0,0);
    //Set Pwm 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    
    //rgbled_0.setpin(44);
    RxFlag = false;
    LineFlag = false; 
    ForwardFlag = false;
    ReverseFlag = false;
    RightFlag = false;
    LeftFlag = false;
    Line_Fail_Flag = false;
    Ckpt_Flag = false;  
    KeepAliveTime = 0;
    OBSTACLE = 0;
    Obstacle_ID = 0;
    LF_status = ON_TRACK;
    Initialize();
    
    nfc_reader.begin();
    uint32_t versiondata = nfc_reader.getFirmwareVersion();
    
    if (! versiondata) {
      Serial.print("Didn't find PN53x board");
      while (1); // halt
    }

    // Got version data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
        
    nfc_reader.setPassiveActivationRetries(0xFF);
    nfc_reader.SAMConfig();
   
    /* 
    if (nfc_reader.tagPresent())
    {
      rgbled_0.setColor(0,101,255,0);
      rgbled_0.show();
      _delay(2);
      rgbled_0.setColor(0,0,0,0);
      rgbled_0.show();

      Serial.println("Found NFC Tag!");
      NfcTag tag = nfc_reader.read();
      Serial.println(tag.getTagType());
      Serial.print("UID: ");
      Serial.println(tag.getUidString());
    }
    else
    {
      Serial.println("No NFC Tag Present!"); 
    }
    */    
    
    //rgbled_0.setColor(0,101,255,40);
    //rgbled_0.show();
    //_delay(2);
    //rgbled_0.setColor(0,0,0,0);
    //rgbled_0.show();

    lastTime = millis()/1000;
    prev_tagTime = millis();
    curr_tagTime = prev_tagTime;
    Serial.println("Robot Test Start at relative time 0 sec !");    
}

void loop(){

    bool KeepAliveFlag = false;
    int display_currentTime_no_overflow = 0;
    static int prev_display_currentTime_no_overflow = 0;
           
    currentTime = millis()/1000 - lastTime;
      
    if((currentTime % 5) == 0){
      display_currentTime_no_overflow = currentTime % 10000;
      if (display_currentTime_no_overflow != prev_display_currentTime_no_overflow) {
        //seg7_6.display((long)(display_currentTime_no_overflow));
        Serial.println(display_currentTime_no_overflow);
        prev_display_currentTime_no_overflow = display_currentTime_no_overflow;        
      }
    }

    if((currentTime % 10) == 0){
      if (KeepAliveTime != currentTime){
        KeepAliveTime = currentTime;
        KeepAliveFlag = true;        
      }
    }

    if(KeepAliveFlag == true){
      Serial2.print("Link Alive");
      KeepAliveFlag = false;
    }

    curr_tagTime = millis();
    if (((curr_tagTime - prev_tagTime) >= 40) && (LF_status != OFF_TRACK)) {
      prev_tagTime = curr_tagTime;
      //StopMoving();
      tag_read_success = 0;
      for (int i=0; i < uidLength; i++){
        uid[i] = 0x00;
      }

      tag_read_success = nfc_reader.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 25);
      if (tag_read_success) {

        StopMoving();
        Serial2.print("Tag ID");
        for (uint8_t i=0; i < uidLength; i++) 
        {
          Serial2.print(" 0x");Serial2.print(uid[i], HEX); 
        }
        Serial2.print(" at time ");
        Serial2.print(currentTime);
        Forward(5,rpm);

        int count_1 = 1;
        double swing_step = 10.0;
        double swing_angle = 0.0;

        while(true) {
          if(linefollower_9.readSensors() == 3.000000){
            break;
          }
          else if(linefollower_9.readSensors() == 2.000000){
            TurnLeft(swing_step,rpm);
          }
          else if(linefollower_9.readSensors() == 1.000000){
            TurnRight(swing_step,rpm);
          }
          else if(linefollower_9.readSensors() == 0.000000){
            swing_angle = pow(-1,count_1)*(count_1*swing_step);
            if (swing_angle >= 0){
              TurnLeft(swing_angle,rpm);
            }
            else{
              TurnRight(swing_angle*-1.0,rpm);
            }
            count_1 = count_1 + 1;
          }
        }

        if(Ckpt_Flag == true){
          //if(memcmp( (const void *)ckpt_uid[cmd_ckpt], (const void *)uid, sizeof(ckpt_uid[cmd_ckpt])) == 0)
          if(memcmp( (const void *)ckpt_uid[cmd_ckpt], (const void *)uid, 4) == 0)
          {
            StopMoving();
            LineFlag = false;
            Ckpt_Flag = false;

            _delay(1.0);
            
            Serial2.print("Checkpoint arrived :");
            for (uint8_t i=0; i < uidLength; i++) 
            {
              Serial2.print(" 0x");Serial2.print(uid[i], HEX); 
            }
          }
        }
      }
    }
    
    if(ultrasonic_10.distanceCm() < 20.32){
      OBSTACLE = 1;
      
      //Obstacle_ID = Obstacle_ID + 1;
      //if (Obstacle_ID == 11) {
      //  Obstacle_ID = 1;
      //}
      currentTime = millis()/1000 - lastTime;
      Serial2.print("Obstacle detected");
      //Serial2.print(Obstacle_ID);
      Serial2.print(" at time ");
      Serial2.print(currentTime);
      StopMoving();
      _delay(2.0);
    }

    if(OBSTACLE == 1) {
      if(ultrasonic_10.distanceCm() < 20.32){
        Obstacle_Avoidance();
      }
      OBSTACLE = 0;      
    }
    
    if(Serial2.available())
    {
      int index = 0;

      for (int i=0; i < 8; i++){
        inp.indat[i] = 0x00;
      }
   
      while (Serial2.available() > 0){
            inp.indat[index] = Serial2.read();
            index ++;
            if (index == 8) {
              break;
            }
      }
      
      while (Serial2.available() > 0){
        Serial2.read();
        _delay(0.0015);
      }
      
      RxFlag = true;
    }

    if (RxFlag == true) { //Uplink Command is received from Test Station
      switch(inp.command[0]){

        case(0xC9): low_speed = (inp.command[2] & 0x00FF); //Drive Distance command 201
                    high_speed = (inp.command[1] & 0x00FF) << 8;
                    velocity = low_speed | high_speed;
                    
                    if ((velocity > 255) || (velocity <= 0)) {
                        velocity = rpm;
                    }
                    
                    low_distance = (inp.command[4] & 0x00FF); 
                    high_distance = (inp.command[3] & 0x00FF) << 8;
                    distance = low_distance | high_distance;
                    
                    if (distance > 0) {
                        Forward(distance, velocity);
                    }
                    else {
                        Reverse(-1.0*distance, velocity);
                    }
                    ForwardFlag = false;
                    ReverseFlag = false;
                    RightFlag = false;
                    LeftFlag = false;                      
                    LineFlag = false;
                    Ckpt_Flag = false;                                          
                    Serial2.print("AcK RX ");
                    Serial2.print(inp.command[0],HEX);
                    break;

        case(0xCA): low_speed = (inp.command[2] & 0x00FF); //Turn Angle command 202
                    high_speed = (inp.command[1] & 0x00FF) << 8;
                    velocity = low_speed | high_speed;
                    
                    if ((velocity > 255) || (velocity <= 0)) {
                        velocity = rpm;
                    }
                    
                    low_angle = (inp.command[4] & 0x00FF); 
                    high_angle = (inp.command[3] & 0x00FF) << 8;
                    angle = low_angle | high_angle;
                    
                    if (angle > 0) { //CCW rotation correpsonds tp positive rotation angle
                        TurnLeft(angle,velocity);
                    }
                    else { //CW rotation correpsonds to negative rotation angle
                        angle = abs(angle);
                        TurnRight(angle,velocity);
                    }
                    ForwardFlag = false;
                    ReverseFlag = false;
                    RightFlag = false;
                    LeftFlag = false;                      
                    LineFlag = false;
                    Ckpt_Flag = false;                                          
                    Serial2.print("AcK RX ");
                    Serial2.print(inp.command[0],HEX);
                    break;
    
        case(0x89): low_speed = (inp.command[2] & 0x00FF); //Drive command 137
                    high_speed = (inp.command[1] & 0x00FF) << 8;
                    velocity = low_speed | high_speed;
                    
                    if (velocity > 255) {
                        velocity = rpm;
                    }
                    else if (velocity < -255) {
                        velocity = -rpm;
                    }
                    
                    low_radius = (inp.command[4] & 0x00FF);
                    high_radius = (inp.command[3] & 0x00FF) << 8;
                    radius = low_radius | high_radius;
        
                    if (radius == 0x8000){ //Drive straight
                      if (velocity > 0) { //Move forward 
                        Forward_NonStop(velocity);
                        ForwardFlag = true;
                        ReverseFlag = false;
                        RightFlag = false;
                        LeftFlag = false;                      
                        LineFlag = false;
                        Ckpt_Flag = false;                                          
                      }
                      else if (velocity < 0) { //Move backwards
                        velocity = abs(velocity);
                        Reverse_NonStop(velocity);
                        ForwardFlag = false;
                        ReverseFlag = true;
                        RightFlag = false;
                        LeftFlag = false;                                            
                        LineFlag = false;                        
                        Ckpt_Flag = false;                                          
                      }
                      else { //Stop Moving as velocity = 0
                        StopMoving();
                        ForwardFlag = false;
                        ReverseFlag = false;
                        RightFlag = false;
                        LeftFlag = false;                                            
                        LineFlag = false;
                        Ckpt_Flag = false;                                          
                      }
                    }
                    else if (radius == 0xFFFF) { //Turn in place clockwise
                        velocity = abs(velocity);
                        TurnRight_NonStop(velocity);
                        ForwardFlag = false;
                        ReverseFlag = false;
                        RightFlag = true;
                        LeftFlag = false;                                            
                        LineFlag = false;
                        Ckpt_Flag = false;                                          
                    }
                    else if (radius == 0x0001) { //Turn in place counter-clockwise
                        velocity = abs(velocity);
                        TurnLeft_NonStop(velocity);
                        ForwardFlag = false;
                        ReverseFlag = false;
                        RightFlag = false;
                        LeftFlag = true;                                            
                        LineFlag = false;
                        Ckpt_Flag = false;                                          
                    }
                    
                    Serial2.print("AcK RX ");
                    Serial2.print(inp.command[0],HEX);
                    break;

        case(0xCB): LineFlag = true; //Line Following command 203
                    Line_Fail_Flag = false;
                    Ckpt_Flag = false;                                          
                    ForwardFlag = false;
                    ReverseFlag = false;
                    RightFlag = false;
                    LeftFlag = false;                                            
                    Serial2.print("AcK RX ");
                    Serial2.print(inp.command[0],HEX);
                    break;
                           
        case(0xCC): low_ckpt = (inp.command[2] & 0x00FF); //Move Checkpoint command 204
                    high_ckpt = (inp.command[1] & 0x00FF) << 8;
                    cmd_ckpt = low_ckpt | high_ckpt;
        
                    if ((cmd_ckpt > 255) || (cmd_ckpt < 0)) {
                        cmd_ckpt = 0;
                    }

                    LineFlag = true;
                    Line_Fail_Flag = false;                                            
                    Ckpt_Flag = true;                                          
                    ForwardFlag = false;
                    ReverseFlag = false;
                    RightFlag = false;
                    LeftFlag = false;
                    Serial2.print("AcK RX ");
                    Serial2.print(inp.command[0],HEX);
                    Serial2.print(" Ckpt ");
                    Serial2.print(cmd_ckpt,DEC);
                    break;
                           
        default:    break;
      }
      for (int i=0; i < 8; i++){
        inp.indat[i] = 0x00;
      }      
      RxFlag = false;
    }

    if (ForwardFlag == true){
      Forward_NonStop(velocity);
    }
    
    if (ReverseFlag == true){
      Reverse_NonStop(velocity);
    }
    
    if (RightFlag == true){
      TurnRight_NonStop(velocity);
    }
    
    if (LeftFlag == true){
      TurnLeft_NonStop(velocity);
    }
    
    if (LineFlag == true){
      LineFollowing();
    
      if(Line_Fail_Flag == true){
        if ((millis()/1000 - Line_Fail_Timer) >= 10.0){
          Line_Fail_Timer = millis()/1000;
          Serial2.print("Line Following lost track");
          TurnRight(5,rpm);
        }
      }
    }
       
    _loop();
}
