//gpio pin numbers
const int buzzerPin = 5; 
const int PushButton = 4;
unsigned long buzzStartTime = 0;  //tracks when the buzzing starts
const unsigned long BuzzDuration = 5000;  //5 seconds in milliseconds (can be changed)
bool isBuzzing = false;  //tracks if we're currently in a buzz cycle


void setup(){
  //Serial.begin(115200); //to see the button state
  pinMode(buzzerPin, OUTPUT); //makes pin5 output
  pinMode(PushButton, INPUT); //makes pin4 input
}

void loop(){
// Read push button state
  int ButtonState = digitalRead(PushButton);
  //Serial.print(ButtonState);
  //Serial.print("\n");
  // If button is pressed, generate tone
  
  if (ButtonState == true && !isBuzzing){ //this if checks if button is pressed and buzzer is not buzzing
    buzzStartTime = millis();
    isBuzzing = true;
  }

  if (isBuzzing) { 
    if ((millis() - buzzStartTime) < BuzzDuration){ //this if statement checks if it's still in the 5second window
      // Create a square wave for buzzer tone
      digitalWrite(buzzerPin, HIGH);
      delayMicroseconds(250); // For ~500Hz tone
      digitalWrite(buzzerPin, LOW);
      delayMicroseconds(1000);
    }
    else {
      digitalWrite(buzzerPin, LOW);
      isBuzzing = false;
    }
  }
}