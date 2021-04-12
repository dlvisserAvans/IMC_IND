/*
   This is the waterSensor SOURCE file.
   Eindopdracht Individueel onderdeel IMC.
   Eigen project gemaakt voor arduino, plant en kamer sensoren met motor voor ventilatie.
   Ontwikkelaar: Dave Visser
*/

#include "moistureSensor.h"

// Global variables.
float  sensor_value = 0;
int  moisture_sensor_pin;

float read_moisture_sensor() {

  for (int i = 0; i <= 100; i++)
  {
    sensor_value  = sensor_value + analogRead(moisture_sensor_pin); // Add the new analog values to the current value in sensor_value.
    delay(10); // Delay for accuracy.
  }

  sensor_value = sensor_value / 100.0;  // Better accuracy because 100 values are taken from the sensor.

  return sensor_value;
}

// Function to set the moisture sensor pin in the main program.
int set_moisture_sensor_pin(int pin) {
  moisture_sensor_pin = pin;
}
