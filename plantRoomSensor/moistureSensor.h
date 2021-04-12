/*
   This is the waterSensor HEADER file.
   Eindopdracht Individueel onderdeel IMC.
   Eigen project gemaakt voor arduino, plant en kamer sensoren met motor voor ventilatie.
   Ontwikkelaar: Dave Visser
*/

#ifndef MOISTURESENSOR_H  // Compiler guards for single compilation
#define MOISTURESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

float read_moisture_sensor();
int set_moisture_sensor_pin(int pin);

#ifdef __cplusplus
}
#endif

#endif /* MOISTURESENSOR_H */
