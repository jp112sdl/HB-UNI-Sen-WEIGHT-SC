#include <HX711.h>

#define HX711_SCK_PIN               6                 //Clock Pin for all HX711 Boards
const uint8_t HX711_DOUT_PINS[]  = {14, 15, 16, 17};  //DT Pins


HX711 scale[sizeof(HX711_DOUT_PINS)];
long offset[sizeof(HX711_DOUT_PINS)];
float    sc[sizeof(HX711_DOUT_PINS)];

void setup() {
  Serial.begin(57600);
  Serial.println("       HB-UNI-Sen-Weight-SC Calibration Sketch");
  Serial.println("1.) Remove all weight from all scales, then press return");

  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) scale[i].begin(HX711_DOUT_PINS[i], HX711_SCK_PIN);

  while (Serial.available() == 0) { }
  while (Serial.available() > 0)  Serial.readString();

  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
    scale[i].set_scale();
    scale[i].set_offset(0);
    Serial.print("Offset ");
    Serial.print(i);
    Serial.print(" = ");
    offset[i] = scale[i].read_average(10);
    Serial.println(offset[i]);
    scale[i].set_offset(offset[i]);
    scale[i].set_offset(offset[i]);
  }

  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
    Serial.println("\n" + String(i + 2) + ".) Enter known load value, put it on the scale with PIN " + String(HX711_DOUT_PINS[i]) + ", then press return");
    while (Serial.available() == 0) { }
    float m = Serial.parseFloat();
    while (Serial.available() > 0) Serial.readString(); //just to catch CR(LF)
    Serial.print("Input = ");
    Serial.println(m, 2);
    Serial.print("Scale = ");
    sc[i] = scale[i].get_units(10) / m;
    Serial.println(sc[i], 2);
  }

  Serial.println("\nInsert these lines into the HB-UNI-Sen-WEIGHT-SC.ino sketch:");

  Serial.print("const uint8_t HX711_DOUT_PINS[]  = { ");
  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
    Serial.print(HX711_DOUT_PINS[i]);
    if (i < sizeof(HX711_DOUT_PINS) - 1) Serial.print(", ");
  }
  Serial.println(" };");

  Serial.print("const float HX711_CALIBRATIONS[] = { ");
  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
    Serial.print(sc[i], 2); Serial.print("f");
    if (i < sizeof(HX711_DOUT_PINS) - 1) Serial.print(", ");
  }
  Serial.println(" };");

  Serial.print("const int32_t    HX711_OFFSETS[] = { ");
  for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
    Serial.print(offset[i]); Serial.print("L");
    if (i < sizeof(HX711_DOUT_PINS) - 1) Serial.print(", ");
  }
  Serial.println(" };");

}

void loop() {

}

