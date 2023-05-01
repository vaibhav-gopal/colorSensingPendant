#include <TCS34725AutoGain.h>
#include <SoftwareSerial.h>

#define buttonPin 3
#define ledPin 5

TCS34725 tcs;
TCS34725::Color color;
TCS34725::RawData raw;

const String MODENAMES[5] = { "Undefined", "Sleep", "Idle", "RGBC", "RGBC+Wait" };

//Button is set on internal pullup, so connect button to ground
int buttonState;

/*
  Making it better:
  - Either fiddle with initilization settings to calibrate, or manually calibrate using the led pin, or do a combination of both
*/

//PRESET COLORS ----------- (20 colors)

struct Color {
  const char* name;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t fileNum;
};

const Color colorSpace[] = {{"russian violet", 37, 18, 74, 0x01}, {"moss green", 15, 90, 15, 0x02}, {"red", 255, 0, 0, 0x03}, {"lime", 0, 255, 0, 4}, {"blue", 0, 0, 255, 5}, {"yellow", 255, 255, 0, 6}, {"cyan", 0, 255, 255, 7}, {"magenta", 255, 0, 255, 8},
  {"chocolate", 210, 105, 30, 9}, {"oxford blue", 0, 0, 70, 10}, {"maroon", 128, 0, 0, 11}, {"olive", 128, 128, 0, 12}, {"green", 0, 128, 0, 13}, {"purple", 128, 0, 128, 14}, {"dark khaki", 189, 183, 107, 15}, {"teal", 0, 128, 128, 16},
  {"navy", 0, 0, 128, 17}, {"brown", 150, 75, 0, 18}, {"salmon", 250, 128, 114, 19}, {"orange", 255, 165, 0, 20}, {"gold", 255, 215, 0, 21}, {"khaki", 240, 230, 140, 22}, {"yellow green", 154, 205, 50, 23}, {"dark olive", 85, 107, 47, 24},
  {"lawn green", 124, 252, 0, 25}, {"light green", 144, 238, 144, 26}, {"coral", 255, 127, 80, 27}, {"sea green", 46, 139, 87, 28}, {"dark sea green", 143, 188, 143, 29}, {"spring green", 0, 255, 127, 30}, {"burgundy",120,20,70,31},
  {"aqua marine", 127, 255, 212, 32}, {"turquoise", 64, 224, 208, 33}, {"light cyan", 176, 224, 230, 34}, {"steel blue", 70, 130, 180, 35}, {"corn flower blue", 100, 149, 237, 36}, {"blue jays blue", 4, 118, 191, 37}, {"light blue", 173, 216, 230, 38},
  {"sky blue", 135, 206, 235, 39}, {"midnight blue", 25, 25, 112, 40}, {"indigo", 75, 0, 130, 41}, {"violet", 238, 130, 238, 42}, {"orchid", 218, 112, 214, 43}, {"dark magenta", 139, 0, 139, 44}, {"dark violet", 148, 0, 211, 45}, {"pink", 255, 192, 203, 46},
  {"deep pink", 255, 20, 147, 47}, {"bubble-gum pink", 255, 105, 180, 48}, {"beige", 245, 245, 220, 49}, {"sienna", 160, 69, 19, 50}, {"tan", 210, 180, 140, 51}, {"lavender", 230, 230, 250, 52}, {"azure", 240, 255, 255, 53}, {"dark crimson", 140, 20, 20, 54}, 
  {"light yellow", 255, 255, 170, 55}, {"lemon chiffon", 255, 255, 200, 56}, {"fire engine red", 200, 25, 35, 57}
};
//Audio files should be named by number, and they can be formatted to anything under 180kb each to hit target color count
//Dont rename files with format rename

//HELPER FUNCTIONS —————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

Color identifyColor(float r, float g, float b, bool redmeanApprox, bool useCalibration, bool printProcess, bool printName) {
  long dr; long dg; long db; //max value is 255^2 = 65025 whereas int only stores up to 32767
  float diff = 1000; float newDiff; //the sqrt() introduces decimals, int truncates that accuracy

  Color current;

  for (int i = 0; i < sizeof(colorSpace) / sizeof(Color); i++) {
    //Calculate "euclidian distance" ie. the distance between points in a 3D grid with R,G,B making each axis
    dr = r - colorSpace[i].red;
    dg = g - colorSpace[i].green;
    db = b - colorSpace[i].blue;
    
    dr = pow(dr, 2);
    dg = pow(dg, 2);
    db = pow(db, 2);

    if (redmeanApprox) {
      //"redmean" approximation to better match human color perception
      float rmod = (r + colorSpace[i].red) / 2;
      dr = (2 + (rmod / 256)) * dr;
      dg = 4 * dg;
      db = (2 + ((255 - rmod) / 256)) * db;
    };

    newDiff = sqrt(dr + dg + db);

    if (printProcess) {
      Serial.print("Color:\t"); Serial.print(colorSpace[i].name);
      Serial.print("\tDist:\t"); Serial.println(newDiff);
    };

    if (newDiff < diff) {
      diff = newDiff;
      current = colorSpace[i];
    };
  };
  if (printName) {
    Serial.println(""); Serial.println("RESULTS"); Serial.println("");
    Serial.print("Name:\t"); Serial.println(current.name);
  }
  return current;
};

//VOICE MODULE (MP3 Storage) ———————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

SoftwareSerial Serial1(10, 11); //setting rx pin to be 10 and tx pin to be 11

uint8_t readSerial(int numOfBytes, unsigned long maxTime, bool printValues, int indexToReturn) {
  unsigned long start = millis();
  unsigned long time = start;
  static bool exceededMax;
  uint8_t r[numOfBytes];

  if (exceededMax) {
    Serial.println("Resetting Serial Buffer -> Try Again");
    while (Serial1.available() != 0) {
      Serial1.read();
    }
    return;
  }

  while (Serial1.available() != numOfBytes) {
    if (Serial1.available() > numOfBytes) {
      Serial1.read();
    }
    else if ((time - start) >= maxTime) {
      exceededMax = true;
      Serial.println("Exceeded Read Time -> Resetting Next Read");
      return;
    };
  };

  Serial.println("");
  for (int i = 0; i < numOfBytes; i++) {
    r[i] = Serial1.read();
    if (printValues) {
      Serial.print(r[i]); Serial.print("\t");
    }
  }
  Serial.println("");

  return r[indexToReturn];
}

void volume(uint8_t selectedVolume) {
  //command buffer for setting volume
  //First 3 bytes indicate which command, 4th and 5th byte is volume
  //volume ranges fromm 0x00-0x1E (0 to 30)
  uint8_t vol[5] = {0xAA, 0x13, 0x01, selectedVolume, selectedVolume + 0xBE} ;
  Serial1.write(vol, 5); //write to serial 1 with sequence of bytes and indicate number of bytes
};

void play(uint8_t Track) {
  uint8_t play[6] = {0xAA, 0x07, 0x02, 0x00, Track, Track + 0xB3};
  Serial1.write(play, 6); //write to serial 1 with sequence of bytes and indicate number of bytes
};

void stop() {
  uint8_t stop[4] = {0xAA, 0x04, 0x00, 0xAE};
  Serial1.write(stop, 4);
};

uint8_t checkAudio() {
  //SM equals sum of previous bytes
  uint8_t check[4] = {0xAA, 0x0D, 0x00, 0xB7}; //Module will return "AA 0D 02 upper-byte lower-byte SM" in 6 bytes ; SM is the sum of previous bytes
  Serial1.write(check, 4);
  return readSerial(6, 1000, true, 4); //return currently playing file number
};

uint8_t status() {
  uint8_t status[4] = {0xAA, 0x01, 0x00, 0xAB}; //Module will return "AA 01 01 Playback-status SM" in 5 bytes ; SM is the sum of previous bytes
  Serial1.write(status, 4);
  return readSerial(5, 1000, true, 3); //return playback status ;  0 is no audio and 1 is playing
};

//COLOR SENSOR (TCS34725) ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void attachSensor() {
  Wire.begin();
  while (!tcs.attach(Wire)) {
    Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    delay(1000);
  }
}

void configureSensor(int intTime = 100, TCS34725::Gain gain = TCS34725::Gain::X16, float rgbScale = 1, TCS34725::Mode setMode = TCS34725::Mode::RGBC, bool printSettings = true) {
  tcs.gain(gain);
  tcs.integrationTime(intTime);
  tcs.scale(rgbScale); //default: rgb color scaling , keeps values lower at higher clears
  tcs.mode(setMode); //default: set to rgbc mode, sensor continously updates data -> whereas in idle mode, sensor is on standby, use tcs.singleRead() to store one set of values when needed

  //Good settings that work:
  //intTime = 20, gain = 60, rgb scale = 3 --> this creates insensitivity at low values and high sensitivity at high values (LED must be on)
  //Cons -> light colors converge to white often

  //Print out TCS34725 Settings———————
  if (printSettings) {
    Serial.println("");
    Serial.print("Integration Time:"); Serial.println(tcs.integrationTime());
    Serial.print("Gain:"); Serial.println(tcs.gain());
    Serial.print("Current Mode Is:"); Serial.println(MODENAMES[(uint8_t) tcs.mode()]);
    Serial.println("");
    delay(1000);
  }
}

void updateSensorValues(bool singleReadOnIdleMode = false, bool printRaw = true, bool printRGB = true, bool printClear = true, bool getNonLEDValues = false) {
  if (getNonLEDValues){
    digitalWrite(ledPin, LOW);  
    delay(200);
  }
  if (singleReadOnIdleMode) {
    tcs.singleRead();
  }
  if (tcs.available()) {
    color = tcs.color(); //Rgb color calculation takes into account clear values, to provide accurate color values regardless of lighting
    raw = tcs.raw();
    if (printRaw) {
      Serial.print("Rr:"); Serial.print(raw.r);
      Serial.print("\t"); Serial.print("Gr:\t"); Serial.print(raw.g);
      Serial.print("\t"); Serial.print("Br:\t"); Serial.print(raw.b);
    }
    if (printRGB) {
      Serial.print("\t"); Serial.print("R:\t"); Serial.print(color.r);
      Serial.print("\t"); Serial.print("G:\t"); Serial.print(color.g);
      Serial.print("\t"); Serial.print("B:\t"); Serial.print(color.b);
    }
    if (printClear) {
      Serial.print("\t"); Serial.print("Clear:\t"); Serial.print(raw.c);
    }
    if (printRaw or printRGB or printClear){
      Serial.println("");
    }
  }
}

//MAIN LOOP (ARDUINO NANO) ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

void setup()
{
  Serial.begin(57600);
  Serial1.begin(9600); //voice module UART bit rate is 9600
  attachSensor();
  configureSensor();

  pinMode(buttonPin, INPUT_PULLUP);

  volume(0x1E);
}

void loop() {
  Serial1.listen();

  updateSensorValues();
  buttonState = digitalRead(buttonPin);

  if (buttonState == 0) {
    Serial.println(""); Serial.println("BUTTON PRESSED"); Serial.println("");
    play(identifyColor(color.r, color.g, color.b, true, true, true, true).fileNum);
    delay(1500);
  }
}
