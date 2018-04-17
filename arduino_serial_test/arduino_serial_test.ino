void setup() {
  Serial.begin(9600);
}

String received;

void loop() {
  Serial.println("Hi");
  if (Serial.available() > 0) {
          received = Serial.readString();
          Serial.print("I received: ");
          Serial.println(received);       
  }
}
