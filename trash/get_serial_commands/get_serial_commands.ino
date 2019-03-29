



int ledPin = 13;


void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10); 

  pinMode(ledPin, OUTPUT);
}

int forward = 0;

void loop()
{
  String str = read_command();

  if(str == "o") {
    digitalWrite(ledPin, HIGH);
  }
  if(str == "f") {
    digitalWrite(ledPin, LOW);
  }
}

String read_command() {
  String str;
  str = Serial.readString();
  return str;
}

void add_commands(String str) {
  if((str == "w" || str == "W") && forward < 6000) {
    forward += 200;
  }
  if((str == "w" || str == "W") && forward > -6000) {
    forward += 200;
  }
  if((str == "p" || str == "P") && forward < 6000) {
    forward = 0;
  }
}
