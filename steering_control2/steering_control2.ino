const int stepPin = 9;
const int dirPin = 8;
const int enPin = 5;

float speed = 0; // Initial speed setting (0 means stopped)
int direction = 0; // Initial direction: 0 for stop, 1 for counterclockwise, -1 for clockwise

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  Serial.begin(38400); // Set the baud rate to 38400
  Serial.setTimeout(100); // Set a timeout for parsing input
}

void loop() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n'); // Read the input until a newline character is encountered
    inputStr.trim(); // Remove leading/trailing whitespace
    float angular_velocity = inputStr.toFloat(); // Convert the input string to a floating-point number
    
    // Determine direction based on the sign of the input
    if (angular_velocity < 0) {
      direction = -1; // Clockwise for negative input
    } else if (angular_velocity > 0) {
      direction = 1; // Counterclockwise for positive input
    } else {
      direction = 0; // Stop for input of 0
    }
    
    // Calculate the speed based on the absolute value of angular velocity, steps per revolution, and microstepping
    speed = abs(angular_velocity) * 200 * 4 / (2 * PI);
    
    Serial.print("Input (Angular Velocity): ");
    Serial.println(angular_velocity); // Print the input value to the Serial Monitor
  }

  if (speed > 0) {
    digitalWrite(enPin, LOW); // Enable the motor

    digitalWrite(dirPin, direction == 1 ? LOW : HIGH); // Set direction based on the 'direction' variable

    // Step signal ON
    digitalWrite(stepPin, HIGH);
    delayMicroseconds((int)(1.0 / speed * 1000000)); // Convert the speed to an appropriate delay
    
    // Step signal OFF
    digitalWrite(stepPin, LOW);
    delayMicroseconds((int)(1.0 / speed * 1000000)); // Convert the speed to an appropriate delay
  } else {
    digitalWrite(enPin, HIGH); // Disable the motor (stop)
  }
}