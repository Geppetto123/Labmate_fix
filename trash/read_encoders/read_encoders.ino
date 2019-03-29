#include <Encoder.h>
#include <DueTimer.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS // Definition for the library Encoder.h
#define READ_PERIOD 100               // timer period in microseconds


#define ENC_R_A 3
#define ENC_R_B 2
#define ENC_L_A 5
#define ENC_L_B 4

#define CPR 40000      // Counts Per Rotation
#define RADIUS_MM 72.5 // Wheel radius
#define PI 3.1415926535897932384626433832795
#define STEP_LENGTH (2 * PI * RADIUS_MM / (CPR))
#define WHEELS_DISTANCE_MM 330

Encoder R_ENC(ENC_R_A, ENC_R_B);
Encoder L_ENC(ENC_L_A, ENC_L_B);

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10); // Without this it runs painfully slowly

  Timer3.attachInterrupt(readEncoders).setPeriod(READ_PERIOD).start();
}

volatile double distanceL_cpy = 0;
volatile double distanceR_cpy = 0;
volatile double impulseR_cpy = 0;
volatile double impulseL_cpy = 0;
volatile double theta_cpy = 0;
volatile double coordinates_cpy[] = {0, 0};

volatile double distanceL = 0;
volatile double distanceR = 0;
volatile double old_impulseR = 0;
volatile double old_impulseL = 0;
volatile double impulseR = 0;
volatile double impulseL = 0;
volatile double theta = 0;
volatile double coordinates[] = {0, 0};
char b[200];

volatile double cnt = 0;
volatile double cnt2 = 0;
unsigned long t = millis();

void loop()
{
  
  if(timeFrom(t) > 1000) {
    send_impulses();
    t = millis();
  }
}

void sendSerial() {
  noInterrupts();

  String str;

  str.concat(coordinates[0]);
  str.concat(" ");
  str.concat(coordinates[1]);
  str.toCharArray(b, 200);
  Serial.write(b);
  Serial.write("\n");

  interrupts();
}

void readEncoders()
{
  count();

  impulseR = R_ENC.read();
  impulseL = L_ENC.read();

}

void updateCoordinates()
{

  distanceR = (impulseR - old_impulseR) * STEP_LENGTH;
  distanceL = (impulseL - old_impulseL) * STEP_LENGTH;
  
  // Calculate new angle and coordinates
  theta = theta + (distanceR - distanceL) / double(WHEELS_DISTANCE_MM);
  coordinates[0] = coordinates[0] + double(distanceR) * sin(theta);

  old_impulseR = impulseR;
  old_impulseL = impulseL;
}

long timeFrom(long t)
{
  return millis() - t;
}

void count()
{
  if (cnt >= 10)
  {
    updateCoordinates();
    // Serial.print("Angle: ");
    // Serial.print(theta*180/PI);
    // Serial.print(", X: ");
    // Serial.print(coordinates[0]);
    // Serial.print(", Y:");
    // Serial.print(coordinates[1]);
    // Serial.print(", Imp. L: ");
    // Serial.print(impulseL);
    // Serial.print(", Imp. R: ");
    // Serial.println(impulseR);

    cnt = 0;
  }
  else
  {
    cnt++;
  }
}

void send_impulses() {
//  noInterrupts();

  impulseR_cpy = impulseR;
  impulseL_cpy = impulseL;
  coordinates_cpy[0] = coordinates[0];
  coordinates_cpy[0] = coordinates[0];
  theta_cpy = theta;
  
  String str;

  str.concat(impulseR_cpy);
  str.concat(" ");
  str.concat(impulseL_cpy);
  str.concat(" ");
  str.concat(coordinates_cpy[0]);
  str.concat(" ");
  str.concat(coordinates_cpy[1]);
  str.concat(" ");
  str.concat(theta_cpy);
  str.toCharArray(b, 200);
  Serial.write(b);
  Serial.write("\n");

//  interrupts();
}

String readCommand() {
  String str;
  str = Serial.readString();
  return str;
}
