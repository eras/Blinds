// -*- mode: c++ -*-
int port = 3;

void setup()
{
  Serial.begin(115200);
  Serial.println("NexaGen ready");
  pinMode(port, OUTPUT);
}

void td()
{
  //  delayMicroseconds(350);
  //delayMicroseconds(337);
  delayMicroseconds(350 * 10);
}

void send0()
{
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
}

void send1()
{
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
}

void sendX()
{
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
}

void sendStop()
{
  digitalWrite(port, HIGH); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  digitalWrite(port, LOW); td();
  for (int c = 0; c < 7; ++c) {
    digitalWrite(port, LOW); td();
    digitalWrite(port, LOW); td();
    digitalWrite(port, LOW); td();
    digitalWrite(port, LOW); td();
  }
}

void send_bits(unsigned long data, char n)
{
  while (n > 0) {
    --n;
    if (data & (1 << n)) {
      sendX();
    } else {
      send0();
    }
  }
  sendStop();
}

void send()
{
  // 1000 1000 0110
  send_bits(04206ul, 12);
  //send_bits(02222211ul, 12);
}

void loop()
{
  if (Serial.available()) {
    Serial.read();
    Serial.println("Sending sequence");
    send();
  }
}
