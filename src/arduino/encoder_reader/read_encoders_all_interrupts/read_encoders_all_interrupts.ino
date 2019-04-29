#include <Encoder.h>
#include <DueTimer.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS // Definition for the library Encoder.h
#define READ_PERIOD 100               // timer period in microseconds

#define ENC_R_A 3
#define ENC_R_B 5
#define ENC_L_A 2
#define ENC_L_B 4

#define CPR 40000      // Counts Per Rotation
#define WHEELS_DISTANCE_MM 335
#define L_WHEEL_MM 470
#define R_WHEEL_MM 470
#define L_STEP_LENGTH (L_WHEEL_MM /(double) (CPR))
#define R_STEP_LENGTH (R_WHEEL_MM /(double) (CPR))
#define CPR_ERROR 440
#define CPR_CORRECTION (1 + CPR_ERROR/CPR)

Encoder R_ENC(ENC_R_A, ENC_R_B);
Encoder L_ENC(ENC_L_A, ENC_L_B);

void setup()
{
  Serial.begin(115200);
  Timer3.attachInterrupt(read_encoders).setPeriod(READ_PERIOD).start();
}

volatile double l_enc = 0;
volatile double r_enc = 0;

volatile double l_enc_old = 0;
volatile double r_enc_old = 0;

volatile double l_enc_cpy = 0;
volatile double r_enc_cpy = 0;

volatile double delta_l_enc = 0;
volatile double delta_r_enc = 0;

volatile bool odometry = false;
volatile bool odometry_done = true;

volatile double theta = 0;
volatile double coordinates[] = {0, 0};
volatile double l_distance = 0;
volatile double r_distance = 0;

char b[200];

volatile double cnt = 0;
volatile double odometry_cnt = 0;

void loop()
{
  if (odometry == true)
  {
    odometry == false;
    compute_odometry();
    delta_l_enc = 0;
    delta_r_enc = 0;
    odometry_cnt++;
    if (odometry_cnt >= 1000) {
      // send_coordinates();
      // send_impulses();
      send_all();
    }
    odometry_done = true;
  }
}

void send_coordinates()
{
  String str;
  str.concat(coordinates[0]);
  str.concat(" ");
  str.concat(coordinates[1]);
  str.toCharArray(b, 200);
  Serial.write(b);
  Serial.write("\n");
}

void send_impulses() {
  String str;
  str.concat(l_enc_cpy);
  str.concat(" ");
  str.concat(r_enc_cpy);
  str.toCharArray(b, 200);
  Serial.write(b);
  Serial.write("\n");
}

void send_all() {
  String str;
  str.concat(l_enc_cpy);
  str.concat(" ");
  str.concat(r_enc_cpy);
  str.concat(" ");
  str.concat(coordinates[0]);
  str.concat(" ");
  str.concat(coordinates[1]);
  str.concat(" ");
  str.concat(theta*180/PI);
  str.toCharArray(b, 200);
  Serial.write(b);
  Serial.write("\n");
}

void read_encoders()
{
  r_enc = R_ENC.read();
  l_enc = L_ENC.read();

  cnt++;

  if (cnt >= 100 && odometry_done)
  {
    l_enc_cpy = l_enc * CPR_CORRECTION; 
    r_enc_cpy = r_enc * CPR_CORRECTION;

    delta_l_enc = l_enc_cpy - l_enc_old;
    delta_r_enc = r_enc_cpy - r_enc_old;
    odometry = true;
    odometry_done = false;
    cnt = 0;
    
    l_enc_old = l_enc_cpy;
    r_enc_old = r_enc_cpy;
  }
}

void compute_odometry()
{
  r_distance = delta_r_enc * R_STEP_LENGTH;
  l_distance = delta_l_enc * L_STEP_LENGTH;
  
  theta = theta + (l_distance - r_distance) / double(WHEELS_DISTANCE_MM);
  coordinates[0] = coordinates[0] + double(r_distance) * sin(theta);
  coordinates[1] = coordinates[1] + double(l_distance) * cos(theta);
}
