#include <Servo.h>
Servo s1, s2;

const int P1 = 9, P2 = 10;
int active = 1;                 // 1 or 2: which servo you're moving
int stepUs = 200;                 // jog size (µs)
int us1 = 1500, us2 = 1500;     // current commands
int min1 = 1500, max1 = 1500;   // discovered bounds
int min2 = 1500, max2 = 1500;

int CLAMP_MIN = 850;           // start conservative
int CLAMP_MAX = 2100;           // widen later only if safe

void writeCmds() {
  s1.writeMicroseconds(constrain(us1, CLAMP_MIN, CLAMP_MAX));
  s2.writeMicroseconds(constrain(us2, CLAMP_MIN, CLAMP_MAX));
}

void printState() {
  Serial.print("Active S"); Serial.print(active);
  Serial.print("  S1:"); Serial.print(us1);
  Serial.print(" ["); Serial.print(min1); Serial.print(","); Serial.print(max1); Serial.print("]");
  Serial.print("  S2:"); Serial.print(us2);
  Serial.print(" ["); Serial.print(min2); Serial.print(","); Serial.print(max2); Serial.print("]");
  Serial.print("  step="); Serial.println(stepUs);
}

void setup() {
  Serial.begin(115200);
  s1.attach(P1); s2.attach(P2);
  delay(300);
  writeCmds(); printState();
  Serial.println("Keys: '1'/'2' select servo | 'a' left, 'd' right | '[' smaller step, ']' bigger step | 'c' center | 'm' save MIN | 'M' save MAX");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  int &u = (active==1 ? us1 : us2);
  int &mn = (active==1 ? min1 : min2);
  int &mx = (active==1 ? max1 : max2);

  if (c=='1') active = 1;
  else if (c=='2') active = 2;
  else if (c=='a') u -= stepUs;
  else if (c=='d') u += stepUs;
  else if (c=='[') stepUs = max(1, stepUs/2);
  else if (c==']') stepUs = min(100, stepUs*2);
  else if (c=='c') { us1 = us2 = 1500; }
  else if (c=='m') mn = u;    // record current as MIN
  else if (c=='M') mx = u;    // record current as MAX
  else if (c=='t') { us1 = CLAMP_MIN; us2 = CLAMP_MAX; }
  else if (c=='b') { us1 = CLAMP_MAX; us2 = CLAMP_MIN; }
  writeCmds();
  printState();
}
