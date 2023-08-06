// Complete Instructions: https://RandomNerdTutorials.com/esp32-digital-inputs-outputs-arduino/

// set pin numbers
const int Levelsensor = 4;  // the number of the pushbutton pin
const int electrovanne =  15;    // the number of the LED pin

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
  Serial.println(WaterState);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  if (WaterState == HIGH) {
    // turn LED on
    digitalWrite(electrovanne, HIGH);
  } else if ( WaterState == LOW  )  {
    // turn LED off
    delay (600000); 
    digitalWrite(electrovanne, LOW); 
    
  }
  //else if ()// if received 0x24 via Lora.
}
