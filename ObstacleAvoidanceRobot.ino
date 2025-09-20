#include <AFMotor.h>
#include <Servo.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 25 // Maximum distance to detect obstacles (in cm)
#define SERVO_PIN 10

AF_DCMotor motor1(1); // Motor 1 connected to M1
AF_DCMotor motor2(2); // Motor 2 connected to M2
AF_DCMotor motor3(3); // Motor 3 connected to M3
AF_DCMotor motor4(4); // Motor 4 connected to M4

Servo myservo;  // Declare servo globally

// Function prototypes
void moveForward();
void stopMotors();
long measureDistance();
void avoidObstacle();
void turnLeft();
void turnRight();

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  myservo.write(90); // Point servo forward
}

void loop() {
  long distance = measureDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= MAX_DISTANCE) {
    stopMotors();
    delay(50); // Reduce delay for quicker response
    avoidObstacle(); // Trigger obstacle avoidance
  } else {
    moveForward();
  }

  delay(100); // Short delay to prevent unnecessary rapid loop iterations
}

void moveForward() {
  motor1.setSpeed(100); // Reduced speed for slower motion
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

long measureDistance() {
  // Trigger the ultrasonic sensor to send a pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the echo duration and calculate the distance
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration / 29 / 2; // Convert time to distance in cm

  return distance;
}

void avoidObstacle() {
  // Look to the left
  myservo.write(0); // Move servo to far left
  delay(200); // Shorten delay
  long distanceLeft = measureDistance();
  Serial.print("Left Distance: ");
  Serial.println(distanceLeft);

  // Look to the right
  myservo.write(180); // Move servo to far right
  delay(200); // Shorten delay
  long distanceRight = measureDistance();
  Serial.print("Right Distance: ");
  Serial.println(distanceRight);

  // Look forward again
  myservo.write(90); // Move servo back to center
  delay(200); // Shorten delay

  // Compare distances and decide which direction to turn
  if (distanceLeft > distanceRight && distanceLeft > MAX_DISTANCE) {
    turnLeft();
  } else if (distanceRight > MAX_DISTANCE) {
    turnRight();
  } else {
    // If no clear path, turn in a default direction (e.g., left)
    turnLeft();
  }
}

void turnLeft() {
  Serial.println("Turning Left");
  motor1.setSpeed(100); // Reduced speed for controlled turning
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  delay(1000); // Adjust delay for a proper turn
  stopMotors();
}

void turnRight() {
  Serial.println("Turning Right");
  motor1.setSpeed(100); // Reduced speed for controlled turning
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  delay(1000); // Adjust delay for a proper turn
  stopMotors();
}
