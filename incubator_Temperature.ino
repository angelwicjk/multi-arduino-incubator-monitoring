#include <PID_v1.h>
#include <DFRobot_SHT20.h>
#include <SoftwareSerial.h>

unsigned long startTime = 0;  // Variable to store the start time

int randominteger;  // random integer for debugging and monitoring


// Define PID constants
double Kp = 50.0;
// Initialize variables for tracking past temperature and integral gain adjustment
double past_temp = 0.0;
double Ki = 0.0;  // Initial value for Ki_temp
double Kd = 0.0;


const int fan = 13;

// Define PID variables
double setpoint = 38.0;  // Set your desired temperature in degrees Celsius
double input, output;

const int enablePin = 6;   // Enable B pin (ENB)
const int in4Pin = 7;     // Input 4 pin (IN4)


unsigned long lastControlUpdate = 0;
const unsigned long controlUpdateInterval = 100;  // Delay for control loop (adjust as needed)

unsigned long lastPrintUpdate = 0;
const unsigned long printUpdateInterval = 1000;  // Print PPM data every 1 second (adjust as needed)


int valveOpenCount = 0;  // Count of valve openings

// Create PID object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// Create DFRobot SHT20 object
DFRobot_SHT20 sht20;

void setup() {
  Serial.begin(9600);
  startTime = millis();  // Initialize the start time

  Wire.begin();
  // Initialize the DFRobot SHT20 sensor
  sht20.initSHT20();
  // Set up the serial communication for debugging
  delay(100);
  Serial.println("Sensor init finish!");

  sht20.checkSHT20();


  pinMode(enablePin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  pinMode(fan, OUTPUT);

  digitalWrite(fan, HIGH);
  digitalWrite(in4Pin, LOW);



  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // PWM range for the L298N (0-255)
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = (currentMillis - startTime) / 1000;  // Calculate elapsed time in seconds

  float temperature = sht20.readTemperature();
  // Set the input value for the PID controller
  input = temperature;
  // Control loop update at a slower interval
  if (currentMillis - lastControlUpdate >= controlUpdateInterval) {
    lastControlUpdate = currentMillis;

    //
    // Compute PID control output
    myPID.Compute();

    // Adjust Ki_temp based on past temperature
    if (past_temp >= input && input >= (setpoint - 0.2)) {
      Ki += 0.06;  // Increase Ki_temp
    } else if (past_temp <= input && input <= (setpoint + 0.2)) {
      Ki -= 0.3;  // Decrease Ki_temp
    }

    // Limit Ki within a specific range
    if (Ki > 10) {
      Ki = 10;
    } else if (Ki < -10) {
      Ki = -10;
    }

    // // Output value represents the PWM value for the temperature control
    analogWrite(in4Pin, output);
    analogWrite(enablePin, output);

    // Store the current temperature for the next iteration
    past_temp = input;

    // PPM printing at a faster interval
    if (currentMillis - lastPrintUpdate >= printUpdateInterval) {
      lastPrintUpdate = currentMillis;

      // Serial.print(", ");
      // Serial.print("Elapsed Time (s): ");
      // Serial.print(elapsedTime);
      // Serial.print(", ");
      // Serial.print("Temp is :");
      Serial.print(input);
      // Serial.print(", ");
      // Serial.print("PTC voltage is :");
      // Serial.print(output);
      // Serial.print(", ");
      // Serial.print("Ki value is :");
      // Serial.print(Ki);

      Serial.println();
    }
  }
}
