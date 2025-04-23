#include <PID_v1.h>
#include <DFRobot_SHT20.h>
#include <SoftwareSerial.h>
#include <NDIR_SoftwareSerial.h>

NDIR_SoftwareSerial mySensor(10, 11);
unsigned long startTime = 0;  // Variable to store the start time

int randominteger;  // random integer for debugging and monitoring
int randomcounter = 0;
String PPM;

// Control the CO2 valve with a counter
int co2Counter = 0;  // Initialize the counter

// Define PID constants
double Kp = 49.0;
// Initialize variables for tracking past temperature and integral gain adjustment
double past_temp = 0.0;
double Ki = 0.0;  // Initial value for Ki_temp
double Kd = 0.0;
int powerC = 0;
float error_C = 0;

float Kp_CO2 = 0.1, Ki_CO2 = 0;
const int fan = 26;

// Define PID variables
double setpoint = 38.5;  // Set your desired temperature in degrees Celsius
double input, output;

const int enablePin = 8;   // Enable B pin (ENB)
const int in4Pin = 9;      // Input 4 pin (IN4)
const int enaPin_co2 = 4;  // Connect to L298N ENA
const int in1Pin_co2 = 5;  // Connect to L298N IN1

const int setpoint_temp = 39;     // Set your desired temperature in degrees Celsius
const int co2_threshold = 50000;  // Threshold CO2 concentration in PPM for valve control

bool valveOpen = false;  // Track the valve state

unsigned long lastControlUpdate = 0;
const unsigned long controlUpdateInterval = 100;  // Delay for control loop (adjust as needed)

unsigned long lastPrintUpdate = 0;
const unsigned long printUpdateInterval = 1000;  // Print PPM data every 1 second (adjust as needed)


int co2_counter = 0;  // Count of valve openings

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
  pinMode(enaPin_co2, OUTPUT);
  pinMode(in1Pin_co2, OUTPUT);
  pinMode(fan, OUTPUT);

  digitalWrite(fan, HIGH);
  digitalWrite(in4Pin, LOW);
  digitalWrite(enaPin_co2, LOW);
  digitalWrite(in1Pin_co2, LOW);

  if (mySensor.begin()) {
    Serial.println("Sensor initialized.");
  } else {
    Serial.println("ERROR: Failed to connect to the CO2 sensor.");
    while (1)
      ;
  }
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

    // Your PID control logic and other code can go here
    if (mySensor.measure()) {
      PPM = String(mySensor.ppm);
      long ppmValue = PPM.toInt();
      //Serial.println(ppmValue);
      error_C = co2_threshold - ppmValue;
      // Control the CO2 valve
      // Control the CO2 valve
      if (co2_counter <= 2) {
        if (ppmValue <= 30000) {
          // Open the CO2 valve
          // Serial.println("Valve is open");
          digitalWrite(enaPin_co2, HIGH);
          digitalWrite(in1Pin_co2, HIGH);
          delay(3000);  // Open the valve for 10 seconds
          digitalWrite(in1Pin_co2, LOW);
          co2_counter++;
          randomcounter++;
        } else if (ppmValue < 45000) {
          // Open the CO2 valve with different delay settings
          // Serial.println("Valve is open");
          digitalWrite(enaPin_co2, HIGH);
          digitalWrite(in1Pin_co2, HIGH);
          delay(200);  // Adjust the delay as needed
          digitalWrite(in1Pin_co2, LOW);
          // Serial.println("Valve is closed");
          //delay(500);
          co2_counter++;
          randomcounter++;
          randominteger = 350;
        } else if (ppmValue < 46000) {
          // Open the CO2 valve with different delay settings
          // Serial.println("Valve is open");
          digitalWrite(enaPin_co2, HIGH);
          digitalWrite(in1Pin_co2, HIGH);
          delay(150);  // Adjust the delay as needed
          digitalWrite(in1Pin_co2, LOW);
          // Serial.println("Valve is closed");
          //delay(500);
          co2_counter++;
          randomcounter++;
          randominteger = 150;
        } else if (ppmValue < 48000) {
          // Open the CO2 valve with different delay settings
          // Serial.println("Valve is open");
          digitalWrite(enaPin_co2, HIGH);
          digitalWrite(in1Pin_co2, HIGH);
          delay(90);  // Adjust the delay as needed
          digitalWrite(in1Pin_co2, LOW);
          // Serial.println("Valve is closed");
          //delay(500);
          co2_counter++;
          randomcounter++;
          randominteger = 50;
        } else if (ppmValue < 49000) {
          // Open the CO2 valve with different delay settings
          // Serial.println("Valve is open");
          digitalWrite(enaPin_co2, HIGH);
          digitalWrite(in1Pin_co2, HIGH);
          delay(55);  // Adjust the delay as needed
          digitalWrite(in1Pin_co2, LOW);
          // Serial.println("Valve is closed");
          //delay(500);
          co2_counter++;
          randomcounter++;
          randominteger = 50;
        }
      } else if (co2_counter > 2 && co2_counter < 80) {
        co2_counter++;
      } else if (co2_counter >= 60) {
        co2_counter = 0;  // resetting the counter for the sensor to catch up the released CO2
      }






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

        if (mySensor.measure()) {
          PPM = String(mySensor.ppm);
          long ppmValue = PPM.toInt();
          //Serial.print("CO ppm is :");
          Serial.print(ppmValue);
          Serial.print(",");
          Serial.print("Elapsed Time (s): ");
          Serial.print(elapsedTime);
          Serial.print(", ");
          //Serial.print("Temp is :");
          Serial.print(input);
          // Serial.print(", ");
          // //Serial.print("PTC voltage is :");
          // Serial.print(output);
          // Serial.print(", ");
          // Serial.print("Ki value is :");
          // Serial.print(Ki);
          // Serial.print(", ");
          // Serial.print("System delay number is:");
          // Serial.print(randominteger);
          // Serial.print(", ");
          // Serial.print("PowerC=");
          // Serial.print(powerC);
          // Serial.print(", ");
          // Serial.print("errorC=");
          // Serial.print(error_C);
          Serial.print(", ");
          Serial.print(randomcounter);
          Serial.println();
        }
      }
    }
  }
}
