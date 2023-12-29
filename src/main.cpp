#include <Arduino.h>
#include <zirconLib.h>

// Function declarations
void lineAvoidance();
void moveTowardsVector();

struct Vector {
  float X;
  float Y;
};

Vector getBallDirection() {
  const int numSensors = 8;  // Assuming you have 8 ball sensors
  int sensorReadings[numSensors];

  // Read ball sensor values
  for (int i = 0; i < numSensors; i++) {
    sensorReadings[i] = readBall(i + 1);
  }

  // Define direction vectors (adjust these based on your sensor positions)
  int directionVectors[numSensors] = {0, 0.7071067812, 1, 0.7071067812, 0, -0.7071067812, -1, -0.7071067812};

  // Calculate net direction based on sensor readings and direction vectors
  int netDirectionX = 0;
  int netDirectionY = 0;

  for (int i = 0; i < numSensors; ++i) {
    netDirectionX += sensorReadings[i] * directionVectors[i];
    netDirectionY += sensorReadings[i] * directionVectors[(i + numSensors / 4) % numSensors];
  }

  // Now netDirectionX and netDirectionY represent the net direction of the ball
  // You can use this information to adjust the robot's movement
  Serial.println("Net Direction X: " + String(netDirectionX));
  Serial.println("Net Direction Y: " + String(netDirectionY));

  Vector ball_vector;
  ball_vector.X = netDirectionX;
  ball_vector.Y = netDirectionY;
  return ball_vector;
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
  float angleDifference3 = motorAngle2 - ballAngle;

  // Calculate motor powers based on sin(angle difference)
  int motorPower1 = static_cast<int>(power * sin(angleDifference1));
  int motorPower2 = static_cast<int>(power * sin(angleDifference2));
  int motorPower3 = static_cast<int>(power * sin(angleDifference3));

  int sign1 = (motorPower1 < 0) ? 1 : 0;
  int sign2 = (motorPower2 < 0) ? 1 : 0;
  int sign3 = (motorPower3 < 0) ? 1 : 0;

  motor1(motorPower1, sign1);
  motor1(motorPower2, sign2);
  motor1(motorPower3, sign3);
  
}


void moveTowardsHighestBall() {
  // Implement your code to move towards the highest ball sensor while correcting with compass
  // This is a placeholder; replace it with your actual code
  Serial.println("Moving towards the highest ball sensor while correcting with compass...");
  // Your code to move towards the highest ball sensor goes here
  // You can use readCompass() for compass direction correction
  while (true) {
    Vector ball_vector = getBallDirection();
    moveTowardsVector(ball_vector, 100);


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
  }
}

