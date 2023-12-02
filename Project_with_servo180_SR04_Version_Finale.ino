/* A start version for a surrounding scanner mini project.
 * We learn to use and understand this first.
 * Then we edit the code further in order to make a surrounding scanner.
 * 
 * This Arduino program is ment for a small project where we are scanning surrounding 
 * with ultrasound sensor HC-SR04 and send data in order to make "a map view" in Excel.
 * 
 * Draw xy scatter with points without line between.
 * Servomotor: Bought from SP electronics in Oulu: SG90 servo 0 - 180deg.
 * Follow coordinate system: 0 deg = x-axis; 90deg = y-axis. Thus, servo can turn
 * from -90 degree ... 0deg ...+90deg with corresponding control messages 0... 90 ... 180.
 * 29.11.2023 / Jaakko Kaski
 * Wires: Yellow to Arduino control pin, check the const int determinations. Dark: GND. Red: +5VDC in Arduino.
 */
#include <Servo.h>
#include <math.h>
Servo servo;

int control = 0; // Control number for servo motor: integer number between 0 and 180.
int control_old = 0;


// Pins in Arduino for ultrasound sensor HC-SR04
 const int GNDPin = 11; // GND pin
 const int echoPin = 10; // Echo Pin gets the voice reflection back.
 const int trigPin = 9; // Trigger Pin wakes the sensor to send ulrasound.
 const int VccPin = 8; // Operating voltage +5V 

 // Pins in Arduino for servo motor: GND <-> Dark wire; +5V pin <-> red wire
 const int servopin = 5; // Control message from Arduino to yellow wire.
 
 float maximumRange = 100.0; // Maximum distance, accepted (cm)
 float minimumRange = 1.2;   // Minimum distance, accepted (cm)
 unsigned long duration = 0; // Duration of ultrasound to go there and back (us)
 float distance = 0.0;       // Measured distance to the surface (cm)
 unsigned long time_ms = 0;  // Time if needed to print out (ms)
 float xvalue = 0.0;
 float yvalue = 0.0;
 float anglevalue = 0.0;

void setup() {
  servo.attach(servopin); //Electrical control message to servo Motor (yellow fire)
  servo.write(control);
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  // Ultrasound sensor pin determinations:
  pinMode(GNDPin, OUTPUT); // GND is 0V voltage source!
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(VccPin, OUTPUT); // Operating voltage
  // Set pin voltages:
  digitalWrite(VccPin, HIGH);
  delayMicroseconds(2); 
  digitalWrite(GNDPin, LOW); 
  delayMicroseconds(2); 
  delay(10); //10ms delay
}
void loop() 
{ 
    for (control = 0; control <= 180; control++) {
      anglevalue = control / 180.0 * 3.141592; // Angle in radians.
    /*
    // Move the servo continuously.
    servo.write(0);
    delay(15);
    servo.write(45);
    delay(15);
    servo.write(90);
    delay(5000);
    servo.write(180);
    delay(5000);
    */

  
 // Test for servo motor: you can write control message to servo with SerialMonitor input line!
 //   if (Serial.available() > 0) {
 //     control = Serial.parseInt(); //If there is input, it will be read.
 //   }
    
    // Write the message to the motor if the value is accepted:
    if(control >= 0 && control <=180){                         
      servo.write(control);   
      control_old = control;       //Update also 'old' variable   
    }
    else {
        Serial.print("Your input is not an accepted, you wrote: ");
        Serial.println(control); 
        control = control_old;     //In the case of bad input, use old message.
    } 

    //Test use of ultrasound sensor HC-SR04
    // See datasheet of the sensor to understand these pulses:
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH); // Time of flight of ultrasound... there and back.

    // Time stamp (ms) if needed:
    time_ms = millis();

    // Calculate distanse from sensor to the surface.
    // Check the equation from physical basic equations!
    // Speed of voice is typically 340m/s.
    distance =(duration * 0.0146 - 0.3063);

    //Check the quality of distance:
    if (distance >= maximumRange || distance <= minimumRange ){
      distance = maximumRange + 50.0; // Non realistic result if not accepted
    }

    // Calculate x and y values.
    xvalue = distance * cos(anglevalue);
    yvalue = distance * sin(anglevalue);

    Serial.print("control = ");
    Serial.print(control);
    Serial.print("  distance = ");
    Serial.print(distance);
    //Serial.print("time_ms "); //take into use if needed
    //Serial.print(time_ms);
    Serial.print("  x value = ");    //Print the situation in any case
    Serial.print(xvalue);  
    Serial.print("  y value = ");
    Serial.println(yvalue);
  
    delay(300);                    //Delay gives time...
    }
}
