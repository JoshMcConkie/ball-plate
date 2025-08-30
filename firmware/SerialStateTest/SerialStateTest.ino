int x=0, y=0, vx=0, vy=0;
String line;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);                // give USB time to enumerate
  Serial.println("READY");   // Python can wait for this

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    line = Serial.readStringUntil('\n');
    line.trim();  // removes \r and spaces
    if (line.length() == 0) return;  // ignore empty lines

    if (sscanf(line.c_str(), "%d,%d,%d,%d", &x, &y, &vx, &vy) == 4) {
      Serial.print("OK,");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.print(',');
      Serial.print(vx); Serial.print(',');
      Serial.println(vy);
    } else {
      Serial.print("BAD,"); Serial.println(line);  // helps debug malformed packets
    }
  }
}



