// #include "main.cpp"

// extern CRGB leds[NUM_LEDS];
// float lightchase_last_update = 0;
// int lightchase_update_interval = 50;

// void lightchase_update() {

//     // Update the light chase pattern
//     if (millis() - lightchase_last_update > lightchase_update_interval) {
//         lightchase_last_update = millis();
    
//         // Shift the light chase pattern
//         for (int i = 0; i < NUM_LEDS; i++) {
//             if (i > 0) {
//                 leds[i-1] = CRGB::Black;
//             }
         
//             leds[i] = CRGB::Green;
        
//             if (i == 0) {
//                 leds[0] = CRGB::Black;
//             }
//         }
//     }

// }