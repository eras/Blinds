int led2 = 2;
int led3 = 3;

void setup()
{
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
}

void loop()
{
  digitalWrite(led2, HIGH);
  digitalWrite(led3, LOW);
  delay(1000);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
  delay(1000);
}

     
