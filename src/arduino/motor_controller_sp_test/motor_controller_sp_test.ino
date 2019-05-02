#include <PID_v1.h>
#include <DuePWM.h>
#include <DueTimer.h>
#include <Encoder.h>

#define ENCODER_DO_NOT_USE_INTERRUPTS // Definition for the library Encoder.h
#define READ_PERIOD 100               // Encoder reading peirod in microseconds
#define COMPUTATION_PERIOD 80000      // Computation period in microseconds

#define PWM_FREQ1 5000
#define PWM_FREQ2 5000

#define ENC_R_A 2
#define ENC_R_B 3

#define ENC_L_A 4
#define ENC_L_B 5

#define DIR_M_R 10
#define DIR_M_L 11

#define R_M 8
#define L_M 9

DuePWM pwm(PWM_FREQ1, PWM_FREQ2);

Encoder R_ENC(ENC_R_A, ENC_R_B);
Encoder L_ENC(ENC_L_A, ENC_L_B);

double l_kp = 0.013, l_ki = 0.045, l_kd = 0;
double r_kp = 0.007, r_ki = 0.0525, r_kd = 0;

volatile double l_impulses = 0;
volatile double r_impulses = 0;
volatile double l_last_impulses;
volatile double r_last_impulses;

double l_duty = 0;
double r_duty = 0;

double l_set_point = 0;
double r_set_point = 0;

double l_set_point_abs = 0;
double r_set_point_abs = 0;

double l_input = 0;
double r_input = 0;

PID l_pid(&l_input, &l_duty, &l_set_point_abs, l_kp, l_ki, l_kd, DIRECT);
PID r_pid(&r_input, &r_duty, &r_set_point_abs, r_kp, r_ki, r_kd, DIRECT);

char b[200];

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10); // In order to get the command over serial immediatly, waits only 10ms

  l_pid.SetMode(AUTOMATIC);
  r_pid.SetMode(AUTOMATIC);

  Timer3.attachInterrupt(control_motors).setPeriod(COMPUTATION_PERIOD).start();
  Timer2.attachInterrupt(read_encoders).setPeriod(READ_PERIOD).start();

  pinMode(DIR_M_L, OUTPUT);
  pinMode(DIR_M_R, OUTPUT);
  digitalWrite(DIR_M_L, HIGH);
  digitalWrite(DIR_M_R, HIGH);

  pwm.pinFreq1(L_M); // Pin freq set to "pwm_freq1" on clock A
  pwm.pinFreq1(R_M); // Pin freq set to "pwm_freq1" on clock B
}

void loop()
{
  if (Serial.available() > 0) {
    receive_commands();
    parse_commands();
  }
}

void receive_commands() {
  String str;
  str = Serial.readString();
  str.toCharArray(b, 200);
}

void parse_commands() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(b, " "); // this continues where the previous call left off
  l_set_point = atof(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(b, " ");
  r_set_point = atof(strtokIndx);     // convert this part to an integer
}

void read_encoders()
{
  r_impulses = R_ENC.read();
  l_impulses = L_ENC.read();
}

void control_motors()
{
  l_input = (l_impulses - l_last_impulses);
  r_input = (r_impulses - r_last_impulses);

  //     Set motor direction
  if (l_set_point > 0)
    digitalWrite(DIR_M_L, LOW);
  else
  {
    digitalWrite(DIR_M_L, HIGH);
  }

  if (r_set_point > 0)
    digitalWrite(DIR_M_R, LOW);
  else
  {
    digitalWrite(DIR_M_R, HIGH);
  }

  l_input = abs(l_input);
  r_input = abs(r_input);

  r_set_point_abs = abs(r_set_point);
  l_set_point_abs = abs(l_set_point);

  (void)l_pid.Compute();
  (void)r_pid.Compute();

  pwm.pinDuty(L_M, l_duty);
  pwm.pinDuty(R_M, r_duty);

  l_last_impulses = l_impulses;
  r_last_impulses = r_impulses;
}