#include <Arduino.h>
#include <zirconLib.h>
#include <cmath>

// Function declarations
void lineAvoidance();
void moveTowardsVector();

struct Vector {
  float X;
  float Y;
};
Vector getBallDirection() {
  const int numSensors = 8;  // Assuming you have 8 ball sensors
  const int numSamples = 10; // Number of samples to take for averaging
  int sensorReadings[numSensors];

  // Read ball sensor values and accumulate readings
  for (int i = 0; i < numSensors; i++) {
    int sum = 0;
    for (int j = 0; j < numSamples; j++) {
      sum += readBall(i + 1);
    }
    sensorReadings[i] = sum / numSamples;
  }

  int maxBallValue = sensorReadings[0];
  int maxDirection = 0;

  for (int i = 1; i < numSensors; i++) {
    if (sensorReadings[i] > maxBallValue) {
      maxBallValue = sensorReadings[i];
      maxDirection = i;
    }
  }

  int directionVectors[numSensors] = {0, 0.7071067812, 1, 0.7071067812, 0, -0.7071067812, -1, -0.7071067812};

  Serial.println(maxDirection);
  Vector ballVector;
  ballVector.X = directionVectors[maxDirection];  // Assuming direction corresponds to sensor index
  ballVector.Y = directionVectors[(maxDirection + 2) % 8];
  return ballVector;
}



float vectorToAngle(const Vector& v) {
  return atan2(v.Y, v.X);
}

void moveTowardsVector(const Vector& ball_vector, int power) {
  // Assuming you have two motors at 45 degrees and 135 degrees, you can adjust these angles accordingly
  float motorAngle1 = 150.0 * (M_PI / 180.0);  // Convert degrees to radians
  float motorAngle2 = 30.0 * (M_PI / 180.0);
  float motorAngle3 = 270.0 * (M_PI / 180.0);

  // Convert the ball vector to an angle
  float ballAngle = vectorToAngle(ball_vector);

  
  // Calculate the difference in angles
  float angleDifference1 = motorAngle1 - ballAngle;
  float angleDifference2 = motorAngle2 - ballAngle;
  float angleDifference3 = motorAngle3 - ballAngle;

  // Calculate motor powers based on sin(angle difference)
  int motorPower1 = static_cast<int>(power * sin(angleDifference1));
  int motorPower2 = static_cast<int>(power * sin(angleDifference2));
  int motorPower3 = static_cast<int>(power * sin(angleDifference3));

  // Serial.println(String(angleDifference3) + " " + String(sin(angleDifference1)) + " " + String(sin(angleDifference2)) + " " + String(sin(angleDifference3)) + " ");


  int sign1 = (motorPower1 < 0) ? 1 : 0;
  int sign2 = (motorPower2 < 0) ? 1 : 0;
  int sign3 = (motorPower3 < 0) ? 1 : 0;

  motor1(abs(motorPower1), sign1);
  motor2(abs(motorPower2), sign2);
  motor3(abs(motorPower3), sign3);
  
}


void moveTowardsHighestBall() {
  // Implement your code to move towards the highest ball sensor while correcting with compass
  // This is a placeholder; replace it with your actual code
  Serial.println("Moving towards the highest ball sensor while correcting with compass...");
  // Your code to move towards the highest ball sensor goes here
  // You can use readCompass() for compass direction correction
  while (true) {
    Vector ball_vector = getBallDirection();
    // Serial.println("X:" + String(ball_vector.X) + " Y:" + String(ball_vector.Y));

    moveTowardsVector(ball_vector, 100);

    if (readButton(1)) {
      Serial.println("Button 1 pressed. Initiating line avoidance program.");
      lineAvoidance();
    }

  }
}

void setup(void)
{
  Serial.begin(115200);
  InitializeZircon();
}

void loop(void)
{
  Serial.println("Press button 1 or button 2 to initiate the corresponding action.");
  
  // Wait for button 1 or button 2 to be pressed
  while (!readButton(1) && !readButton(2)) {
    // You can add additional logic or actions here if needed
    delay(100);  // Adjust the delay as needed to control the loop speed
  }

  // Check which button is pressed
  if (readButton(1)) {
    Serial.println("Button 1 pressed. Initiating line avoidance program.");
    lineAvoidance();
  } else if (readButton(2)) {
    Serial.println("Button 2 pressed. Moving towards the highest ball sensor.");
    moveTowardsHighestBall();
  }
}

void lineAvoidance() {
  // Implement your line avoidance program logic here
  // This is a placeholder; replace it with your actual code
  Serial.println("Executing line avoidance program...");
  // Your line avoidance code goes here
  while (true) {
    Serial.printf("Line values: %d %d %d\n", readLine(1), readLine(2), readLine(3));
    if (300 < readLine(1)) {
      motor1(100, 1);
      motor2(100, 0);
    }
    else if (300 < readLine(2)) {
      motor2(100, 1);
      motor3(100, 0);
    }
    else if (300 < readLine(3)) {
      motor1(100, 0);
      motor3(100, 1);
    }
    else {
      motor2(0, 0);
      motor3(0, 0);
      motor1(0, 0);
    }
    if (readButton(2)) {
      Serial.println("Button 2 pressed. Moving towards the highest ball sensor.");
      moveTowardsHighestBall();
    }
  }
}

