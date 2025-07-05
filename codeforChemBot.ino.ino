#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN A0 
#define ECHO_PIN A1
#define MAX_DISTANCE 200 
#define MAX_SPEED 190 // sets speed of DC motors
#define MAX_SPEED_OFFSET 20

#define sensorDigital 28  // Define the digital pin where the MQ-3 digital output is connected
#define pump 24            // Define the pin for the pump
#define brushServoPin 26   // Define the pin for the brush servo

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

AF_DCMotor motor3(3, MOTOR34_1KHZ); // Front left and back left
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Front right and back right
Servo myservo;   
Servo brushServo; // Brush servo

boolean goesForward = false;
int distance = 100;
int speedSet = 0;

void setup() {
  Serial.begin(9600);  // Initialize the serial communication
  myservo.attach(22);  
  myservo.write(115); 
  delay(2000);

  brushServo.attach(brushServoPin); // Attach brush servo
  brushServo.write(0); // Set brush to rest position
  
  pinMode(sensorDigital, INPUT);  // Set the MQ-3 digital output pin as input
  pinMode(pump, OUTPUT);  // Set pump pin as an output
}

void loop() {
  int distanceR = 0;
  int distanceL = 0;
  int alcoholDetected = digitalRead(sensorDigital);  // Read the digital value from the MQ-3 sensor
  delay(40);
  
  distance = readPing();  // Read the distance from the ultrasonic sensor
  Serial.print("Distance: ");
  Serial.print(distance);  // Print the distance value to the Serial Monitor
  Serial.println(" cm");

  // Check if alcohol is detected by the MQ-3 sensor
  if (alcoholDetected == 0) {
    Serial.println("Alcohol detected");
    moveStop();  // Stop the car if alcohol is detected
    startCleaning(); // Start pump and brush rotation
  } else {
    if (distance <= 15) {
      moveStop();
      delay(100);
      moveBackward();
      delay(300);
      moveStop();
      delay(200);
      distanceR = lookRight();
      delay(200);
      distanceL = lookLeft();
      delay(200);

      if(distanceR >= distanceL) {
        turnRight();
        moveStop();
      } else {
        turnLeft();
        moveStop();
      }
    } else {
      moveForward();
    }
  }
  delay(100);  // Small delay before the next reading
}

int lookRight() {
  myservo.write(50); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115); 
  return distance;
}

int lookLeft() {
  myservo.write(170); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115); 
  return distance;
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm == 0) {
    cm = 250;  // If the sensor doesn't detect anything, set distance to 250 cm
  }
  return cm;
}

void moveStop() {
  motor3.run(RELEASE); 
  motor4.run(RELEASE);
} 
  
void moveForward() {
  if(!goesForward) {
    goesForward = true;
    motor3.run(FORWARD); // Front left and back left
    motor4.run(FORWARD); // Front right and back right
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor3.run(BACKWARD); // Front left and back left
  motor4.run(BACKWARD); // Front right and back right
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) {
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}  

void turnRight() {
  motor3.run(FORWARD); // Front left and back left
  motor4.run(BACKWARD); // Front right and back right     
  delay(500);
  motor3.run(FORWARD); // Front left and back left
  motor4.run(FORWARD); // Front right and back right
} 

void turnLeft() {
  motor3.run(BACKWARD); // Front left and back left   
  motor4.run(FORWARD); // Front right and back right   
  delay(500);
  motor3.run(FORWARD); // Front left and back left
  motor4.run(FORWARD); // Front right and back right
}

void startCleaning() {
  digitalWrite(pump, HIGH);  // Activate the pump (turn on)
  brushServo.write(180);      // Rotate the brush to 180 degrees
  delay(500);                 // Wait for half a second to complete rotation
  brushServo.write(0);        // Rotate the brush to 0 degrees
  delay(500);                 // Wait for half a second to complete rotation
  
  // Rotate brush continuously from 0 to 180 to 0
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // Run for 10 seconds
    brushServo.write(180);   // Rotate to 180 degrees
    delay(500);              // Wait for half a second
    brushServo.write(0);     // Rotate back to 0 degrees
    delay(500);              // Wait for half a second
  }
  
  digitalWrite(pump, LOW);   // Deactivate the pump (turn off)
  brushServo.write(0);       // Return brush to rest position
}
