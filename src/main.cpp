#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "esp_task_wdt.h"
//#include "FastLED.h"
//#include "main.h"

// Motor driver pin definitions
#define Pin_frontL_f 18
#define Pin_frontL_b 5
#define Pin_frontR_f 4
#define Pin_frontR_b 15
#define Pin_backL_f 21
#define Pin_backL_b 19
#define Pin_backR_f 16
#define Pin_backR_b 17

#define Intake_motor 13
#define Intake_motor2 27
#define Shooter_Pin 12
#define servoPin 14

#define WDT_TIMEOUT 10  // Watchdog timeout in seconds

#define LED_PIN 23
#define NUM_LEDS 30

// Xbox controller MAC address (replace with your actual address)
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("EC:83:50:05:71:92");

//EC:83:50:05:71:92 for xbox one controller
//0C:35:26:C1:46:6E for xbox series x controller galaxy purple

//CRGB leds[NUM_LEDS];

// PWM Setup: Assign a unique channel per pin
void setupPWM() {
  //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS); 

  Serial.println("Initializing PWM...");

  const int pwmFrequency = 5000;  // 5 kHz
  const int pwmResolution = 8;    // 8-bit resolution

  // Assigning a separate LEDC channel per motor pin
  ledcSetup(0, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_frontL_f, 0);
  ledcSetup(1, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_frontL_b, 1);
  ledcSetup(2, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_frontR_f, 2);
  ledcSetup(3, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_frontR_b, 3);
  ledcSetup(4, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_backL_f, 4);
  ledcSetup(5, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_backL_b, 5);
  ledcSetup(6, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_backR_f, 6);
  ledcSetup(7, pwmFrequency, pwmResolution);  ledcAttachPin(Pin_backR_b, 7);

  Serial.println("PWM Initialized Successfully");
}

// Function to control motor speed and direction
void controlMotor(int channel_f, int channel_b, float speed) {
  int pwmSpeed = abs(speed * 255);  // Convert -1 to 1 range into 0 to 255

  if (speed > 0) {
    ledcWrite(channel_f, pwmSpeed);  // Forward
    ledcWrite(channel_b, 0);
  } else {
    ledcWrite(channel_f, 0);
    ledcWrite(channel_b, pwmSpeed);  // Backward
  }
}

// Xbox controller handling task
void xboxControllerTask(void *pvParameters) {
  while (true) {
    xboxController.onLoop();  // Handle Bluetooth input
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Short delay for stability
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Controller");

  delay(500);

  pinMode(Intake_motor, OUTPUT);
  pinMode(Intake_motor2, OUTPUT);
  pinMode(Shooter_Pin, OUTPUT);

  // Initialize PWM
  setupPWM();

  // Initialize watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // Start Xbox controller
  xboxController.begin();
  Serial.println("Controller initialized");

  // Create a FreeRTOS task for handling Xbox controller
  xTaskCreatePinnedToCore(
    xboxControllerTask,  // Task function
    "XboxControllerTask", // Task name
    4096,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority
    NULL,                 // Task handle
    0                     // Run on Core 0
  );
}

void loop() {
  esp_task_wdt_reset();  // Reset watchdog timer

  if (xboxController.isConnected()) {
    float lx_axis = ((float)xboxController.xboxNotif.joyLHori / 32767.0) - 1 ; // Left stick horizontal
    float ly_axis = ((float)xboxController.xboxNotif.joyLVert / 32767.0) - 1; // Left stick vertical
    float rx_axis = (((float)xboxController.xboxNotif.joyRHori / 32767.0) - 1) * 0.35; // Right stick horizontal (rotation)

    float lt_axis = ((float)xboxController.xboxNotif.trigLT / 255.0); // Left trigger
    float rt_axis = ((float)xboxController.xboxNotif.trigRT / 255.0); // Right trigger

    if (rt_axis > 1) {
      ledcWrite(8, 255);
      digitalWrite(Intake_motor, LOW);
      digitalWrite(Shooter_Pin, HIGH);
    } else {
      if (xboxController.xboxNotif.btnRB) {
        digitalWrite(Shooter_Pin, HIGH);
      } else {
        digitalWrite(Shooter_Pin, LOW);
      }

      if (lt_axis > 1) {
        digitalWrite(Intake_motor, HIGH);
      } else {
        digitalWrite(Intake_motor, LOW);
      }

      if (xboxController.xboxNotif.btnLB) {
        digitalWrite(servoPin, HIGH);
      } else {
        digitalWrite(servoPin, LOW);
      }

      //lightchase_update();
    }

    //deadzones
    if (abs(lx_axis) < 0.15) lx_axis = 0;
    if (abs(ly_axis) < 0.15) ly_axis = 0;
    if (abs(rx_axis) < 0.05) rx_axis = 0;
    
    Serial.print("LX: "); Serial.print(lx_axis);
    Serial.print(", LY: "); Serial.print(ly_axis);
    Serial.print(", RX: "); Serial.print(rx_axis);
    Serial.print(", LT: "); Serial.println(lt_axis);
    Serial.print(", RT: "); Serial.println(rt_axis);

    // X-drive motor calculations
    float frontL = ly_axis - lx_axis - rx_axis; // Front-left wheel
    float frontR = ly_axis + lx_axis + rx_axis; // Front-right wheel
    float backL = ly_axis + lx_axis - rx_axis; // Back-left wheel
    float backR = ly_axis - lx_axis + rx_axis; // Back-right wheel

    // Constrain values to -1.0 to 1.0
    frontL = constrain(frontL, -1.0, 1.0);
    frontR = constrain(frontR, -1.0, 1.0);
    backL = constrain(backL, -1.0, 1.0);
    backR = constrain(backR, -1.0, 1.0);

    // Drive motors using correct channels
    controlMotor(0, 1, frontL);
    controlMotor(2, 3, frontR);
    controlMotor(4, 5, backL);
    controlMotor(6, 7, backR);

  } else {
    // for (int i = 0; i < NUM_LEDS; i++) {
    //   leds[i] = CRGB::Red;
    // } FastLED.show();

    Serial.println("Controller not connected");
    controlMotor(0, 1, 0);
    controlMotor(2, 3, 0);
    controlMotor(4, 5, 0);
    controlMotor(6, 7, 0);
    delay(250);

    // for (int i = 0; i < NUM_LEDS; i++) {
    //   leds[i] = CRGB::Black;
    // } FastLED.show();

    delay(250);
  }
  vTaskDelay(20 / portTICK_PERIOD_MS);  // Short delay for stability
}
