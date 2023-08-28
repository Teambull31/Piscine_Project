
// Complete Instructions: https://RandomNerdTutorials.com/esp32-digital-inputs-outputs-arduino/

// set pin numbers
const int Levelsensor = 4;  // the number of the pushbutton pin
const int electrovanne =  15;    // the number of the LED pin
unsigned long dynamique_interval = 60000;
const long interval = dynamique_interval;
unsigned long previousMillis = 0;
// variable for storing the pushbutton status 
int WaterState = 0;

void setup() {
  Serial.begin(115200);  
  // initialize the pushbutton pin as an input
  pinMode(Levelsensor, INPUT);
  // initialize the LED pin as an output
  pinMode(electrovanne, OUTPUT);
}

void loop() {
  // read the state of the pushbutton value
  WaterState = digitalRead(Levelsensor);
  unsigned long currentMillis = millis();
  Serial.println(WaterState);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  if (WaterState == HIGH) {
    // turn LED on
    digitalWrite(electrovanne, HIGH);
  } else if ( WaterState == LOW && (currentMillis - previousMillis > interval) )  {
    // turn LED off
     previousMillis = currentMillis;
    
    digitalWrite(electrovanne, LOW); 
    
  }
  //else if ()// if received 0x24 via Lora.
}
