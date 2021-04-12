/*
   Eindopdracht Individueel onderdeel IMC.
   Eigen project gemaakt voor arduino, plant en kamer sensoren met motor voor ventilatie.
   Ontwikkelaar: Dave Visser
*/

//All the required includes for this project.
#include "moistureSensor.h" //Self made header/source file.
#include "DHT.h" //Uses downloadable library https://github.com/adafruit/DHT-sensor-library
#include "LiquidCrystal_I2C.h" //Uses downloadable library https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

#define INTERRUPTPIN 2 //Pin on which the interrupt is set.
//Device pins.
#define DHTPIN 3
#define FANMOTORPIN 5
#define MOISTURESENSORPIN 0
#define MOISTURELED 9
#define HUMIDLED 10
#define TEMPLED 11

//Value defines.
#define DHTTYPE DHT11
#define PREFERABLEROOMTEMP 22
#define MAX_PWM_VAL 255

//Global variables.
float plant_moisture_value, room_humidity_value, room_temperature_value;
volatile int lcd_menu_display_value;
DHT dht(DHTPIN, DHTTYPE);           // Belongs to the DHT.h file.
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x3F for a 16 chars and 2 line display, Belongs to the DHT.h file.

// For debugging purposes....
int verbosity = 1;

// Function declares.
void read_sensors();
void change_lcd_screen();
void update_value_leds();
void enable_output_pins();
float temphudsensor_read_humidity();
float temphudsensor_read_temperature();

void setup() {
  Serial.begin(9600); //Start the Serial listener.
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), change_lcd_screen, RISING); //Set the interrupt on the pin of INTERRUPTPIN
  dht.begin();  // Starts the dht sensor listening.
  lcd.init();   // Starts the lcd controller.
  lcd.clear();
  lcd.backlight();
  enable_output_pins(); // Function which enables the output on specific pins.
  set_moisture_sensor_pin(MOISTURESENSORPIN); // Set the moisture sensor value in the lib files.
}

// Looped code, I have tried to use FreeRTOS tasks but that did not work well with the single core arduino.
void loop() {
  read_sensors(); // In the loop first read all of the sensors.
  change_motor_speed(); // If necessary change the fan motor speed to cool down room.
  update_lcd_display(lcd_menu_display_value); // // Update the lcd display with the correct values.
  update_value_leds();  // Update the leds with the current values.
}

// Function to adjust all the 3 leds for temp, humid and moisture of ground.
void update_value_leds() {
  analogWrite(TEMPLED, calculate_temperature_led_value(room_temperature_value));
  analogWrite(HUMIDLED, calculate_humidity_led_value(room_humidity_value));
  analogWrite(MOISTURELED, calculate_moisture_led_value(plant_moisture_value));
}

// Calculate a correct step and value for the led PWM signal.
int calculate_temperature_led_value(int current_value) {
  int minimal_temp = -5; // A minimum value for the temp indoors.
  int maximal_temp = 35; // A maximum value for the temp indoors.

  int step_led_size = (MAX_PWM_VAL / (maximal_temp - (minimal_temp)));
  return step_led_size * current_value;
}

// Calculate a correct step and value for the led PWM signal.
int calculate_moisture_led_value(int current_value) {
  int maximal_moist = 200;  // A max value for the moisture sensor.

  float step_led_size = (MAX_PWM_VAL / maximal_moist);
  return step_led_size * current_value;
}

// Calculate a correct step and value for the led PWM signal.
int calculate_humidity_led_value(int current_value) {
  int minimal_hum = 0;  // A minimum value for the indoors humidity.
  int maximal_hum = 100; // A maximum value for the indoors humidity.

  int step_led_size = (MAX_PWM_VAL / (maximal_hum + minimal_hum));
  return step_led_size * current_value;
}

// Function which enables the used pins for output.
void enable_output_pins() {
  pinMode(FANMOTORPIN, OUTPUT);
  pinMode(TEMPLED, OUTPUT);
  pinMode(HUMIDLED, OUTPUT);
  pinMode(MOISTURELED, OUTPUT);
}

// This function is used when the interrupt button is pressed.
void change_lcd_screen() {
  lcd_menu_display_value++;
  if (lcd_menu_display_value >= 3) {
    lcd_menu_display_value = 0;
  }
}

// This function checks the current room temperature and if temp is above preferable value then start the fan.
void change_motor_speed() {
  if (room_temperature_value >= PREFERABLEROOMTEMP) {
    digitalWrite(FANMOTORPIN, HIGH);
  } else {
    digitalWrite(FANMOTORPIN, LOW);
  }
}

//Function which reads all the sensors of the device. Two for the room temperature/humidity and one for a ground moisture of a plant.
void read_sensors() {

  plant_moisture_value = read_moisture_sensor();
  delay(1); //Delay for accuracy.
  room_humidity_value = temphudsensor_read_humidity();
  delay(1); //Delay for accuracy.
  room_temperature_value = temphudsensor_read_temperature();
  delay(1); //Delay for accuracy.

  // If the debugging boolean is checked it will print all the values of the chosen type. This can be changed when the interrupt is triggered.
  if (verbosity) {
    switch (lcd_menu_display_value) {
      case 0:
        Serial.print("Moisture value: ");
        Serial.println(plant_moisture_value);
        Serial.println("=============================================");
        break;
      case 1:
        Serial.print("Humidity: ");
        Serial.print(room_humidity_value);
        Serial.println("%");
        Serial.println("=============================================");
        break;
      case 2:
        Serial.print("Temperature in C : ");
        Serial.println(room_temperature_value);
        Serial.println("=============================================");
        break;
    }
    delay(5); //Delay for accuracy.
  }
}

// Updat the lcd module when after the interrupt button is clicked.
void update_lcd_display(int value) {
  switch (value) {
    case 0:
      lcd.setCursor(0, 0);  //Set cursor to character 2 on line 0
      lcd.print("Moisture value: ");
      lcd.setCursor(0, 1);
      lcd.print(plant_moisture_value);
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(0, 0);  //Set cursor to character 2 on line 0
      lcd.print("Humidity: ");
      lcd.setCursor(0, 1);
      lcd.print(room_humidity_value);
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0, 0);  //Set cursor to character 2 on line 0
      lcd.print("Temperature in C : ");
      lcd.setCursor(0, 1);
      lcd.print(room_temperature_value);
      break;
  }
  delay(5); //Delay for accuracy.
}

// Read the humidity of the dht sensor.
float temphudsensor_read_humidity() {
  float val = dht.readHumidity();
  if (isnan(val)) {
    val = -1;
  }
  return val;
}

// Read the temperature of the dht sensor.
float temphudsensor_read_temperature() {
  float val = dht.readTemperature();
  if (isnan(val)) {
    val = -1;
  }
  return val;
}
