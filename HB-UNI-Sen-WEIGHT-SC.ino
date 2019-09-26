//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-09-22 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

// https://github.com/bogde/HX711
// https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all


#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4
#define HX711_SCK_PIN      6
// Ausgang Trigger SC-Kontakt ->
#define SC_TRIGGER_PIN     7
// -> Eingang Trigger SC-Kontakt
#define SC_PIN             9
#define ISR_PIN            5

const uint8_t HX711_DOUT_PINS[]  = {    14  ,    15  ,    16  ,   17     };
const float HX711_CALIBRATIONS[] = { 400.87f, 412.73f, 413.41f,  399.76f };
const int32_t    HX711_OFFSETS[] = {  37366L,  31404L,  58191L, -151437L };
#define DEVICE_CHANNEL_COUNT sizeof(HX711_DOUT_PINS) + 1

#define AVERAGE_READ_COUNT 10


#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>
#include <ThreeState.h>

#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define LCD_COLUMNS     16
#define LCD_ROWS        2
#define EMPTYLINE       "                "

// number of available peers per channel
#define PEERS_PER_CHANNEL      4
#define PEERS_PER_SCCHANNEL    8
#define SC_INVERT              false

enum sc_states {
  TRG_OFF,
  TRG_ON
};

volatile bool isrDetected = false;
#define sendISR(pin) class sendISRHandler { public: static void isr () { isrDetected = true; } }; pinMode(pin, INPUT_PULLUP); if( digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT ) enableInterrupt(pin,sendISRHandler::isr,RISING); else attachInterrupt(digitalPinToInterrupt(pin),sendISRHandler::isr,RISING);

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x4C, 0x00},          // Device ID
  "JPWEIGHT40",                // Device Serial
  {0xF3, 0x4C},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AskSin<StatusLed<LED_PIN>, BatterySensorUni<A1,5>, Radio<AvrSPI<10, 11, 12, 13>, 2>> Hal;
Hal hal;

DEFREGISTER(UReg0, MASTERID_REGS, 0x01, 0x02, 0x03, 0x04, 0x05, 0x20, 0x21, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    bool TaraOnRestart (uint8_t value) const {
      return this->writeRegister(0x05, 0x01, 0, value & 0xff);
    }
    bool TaraOnRestart () const {
      return this->readRegister(0x05, 0x01, 0, false);
    }

    bool Tara (int32_t value) const {
      return
          this->writeRegister(0x01, (value >> 24) & 0xff) &&
          this->writeRegister(0x02, (value >> 16) & 0xff) &&
          this->writeRegister(0x03, (value >> 8) & 0xff) &&
          this->writeRegister(0x04, (value) & 0xff)
          ;
    }

    int32_t Tara () const {
      return
          ((int32_t)(this->readRegister(0x01, 0)) << 24) +
          ((int32_t)(this->readRegister(0x02, 0)) << 16) +
          ((int32_t)(this->readRegister(0x03, 0)) << 8) +
          ((int32_t)(this->readRegister(0x04, 0)))
          ;
    }

    bool WeightLimit (int32_t value) const {
      return
          this->writeRegister(0x23, (value >> 24) & 0xff) &&
          this->writeRegister(0x24, (value >> 16) & 0xff) &&
          this->writeRegister(0x25, (value >> 8) & 0xff) &&
          this->writeRegister(0x26, (value) & 0xff)
          ;
    }

    int32_t WeightLimit () const {
      return
          ((int32_t)(this->readRegister(0x23, 0)) << 24) +
          ((int32_t)(this->readRegister(0x24, 0)) << 16) +
          ((int32_t)(this->readRegister(0x25, 0)) << 8) +
          ((int32_t)(this->readRegister(0x26, 0)))
          ;
    }

    bool WeightHysteresis (int32_t value) const {
      return
          this->writeRegister(0x27, (value >> 24) & 0xff) &&
          this->writeRegister(0x28, (value >> 16) & 0xff) &&
          this->writeRegister(0x29, (value >> 8) & 0xff) &&
          this->writeRegister(0x2a, (value) & 0xff)
          ;
    }

    int32_t WeightHysteresis () const {
      return
          ((int32_t)(this->readRegister(0x27, 0)) << 24) +
          ((int32_t)(this->readRegister(0x28, 0)) << 16) +
          ((int32_t)(this->readRegister(0x29, 0)) << 8) +
          ((int32_t)(this->readRegister(0x2a, 0)))
          ;
    }

    void defaults () {
      clear();
      Sendeintervall(180);
      TaraOnRestart(0);
      Tara(0);
      //WeightLimit(0);
      //WeightHysteresis(0);
    }
};

DEFREGISTER(Reg1, CREG_AES_ACTIVE, CREG_MSGFORPOS, CREG_EVENTDELAYTIME, CREG_LEDONTIME, CREG_TRANSMITTRYMAX)
class SCList1 : public RegList1<Reg1> {
  public:
    SCList1 (uint16_t addr) : RegList1<Reg1>(addr) {}
    void defaults () {
      clear();
      //msgForPosA(1);
      //msgForPosB(2);
      aesActive(false);
      eventDelaytime(0);
      ledOntime(100);
      transmitTryMax(3);
    }
};

typedef ThreeStateChannel<Hal, UList0, SCList1, DefList4, PEERS_PER_SCCHANNEL> SCChannel;

class LcdType {
private:
  LiquidCrystal_I2C LCD;
  const char waitchar[4];
  uint8_t wc;
public:
  LcdType () : LCD(0x27, LCD_COLUMNS, LCD_ROWS), waitchar{ '|', '/', '-', byte(7)}, wc(0)  {}
  virtual ~LcdType () {}

  void init() {
     LCD.init();
     LCD.clear();
     LCD.backlight();
     uint8_t customBlockFrame[8]  = { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F };
     uint8_t customBlockFilled[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
     uint8_t customBackslash[8]   = { 0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00 };
     LCD.createChar(5, customBlockFrame);
     LCD.createChar(6, customBlockFilled);
     LCD.createChar(7, customBackslash);
   }

  void waitProgress(bool show) {
    LCD.setCursor(0,1);
    if (show) {
      LCD.print(waitchar[wc++]);
      if (wc > 3) wc = 0;
    } else {
      LCD.print(" ");
    }
  }

  void writeLine(uint8_t line, const char * l, bool center, bool clear) {
    LCD.setCursor(0, line - 1);
    if (clear) LCD.print(EMPTYLINE);
    if (center) {
      LCD.setCursor((16 - strlen(l)) / 2 , line - 1);
    }
    LCD.print(l);
  }

  void writeChar(const char c, uint8_t col, uint8_t row) {
    LCD.setCursor(col - 1, row - 1);
    LCD.write(c);
  }

} lcd;

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, int32_t *weight, int32_t sum) {
      Message::init(0x0d + (sizeof(HX711_DOUT_PINS) * 3), msgcnt, 0x53, BIDI | WKMEUP, 0x00, 0x42 );
      pload[0] = (weight[0]>> 8) & 0x7f;
      pload[1] = weight[0] & 0xff;

      for (uint8_t s = 1; s < sizeof(HX711_DOUT_PINS); s++) {
        pload[(s * 3) - 1] = 0x42 + s;
        pload[(s * 3)    ] = (weight[s] >> 8) & 0x7f;
        pload[(s * 3) + 1] = (weight[s])      & 0xff;
      }

      pload[(sizeof(HX711_DOUT_PINS) * 3) - 1] = 0x42 + sizeof(HX711_DOUT_PINS);
      pload[(sizeof(HX711_DOUT_PINS) * 3)    ] = (sum >> 8) & 0x7f;
      pload[(sizeof(HX711_DOUT_PINS) * 3) + 1] = (sum) & 0xff;
    }
};

class MeasureChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    MeasureEventMsg msg;
    bool            first;
    int32_t         weight[sizeof(HX711_DOUT_PINS)];
    int32_t         weight_sum;
    HX711           hx711[sizeof(HX711_DOUT_PINS)];
    uint16_t        tickcnt ;
    uint8_t         sc_state;

  public:
    MeasureChannel () : Channel(), Alarm(0), first(true), weight_sum(0), tickcnt(0), sc_state(0) {}
    virtual ~MeasureChannel () {}

    void scChanged(uint8_t state = TRG_OFF) {
      sc_state = state;
      digitalWrite(SC_TRIGGER_PIN, (SC_INVERT) ? !sc_state : sc_state);
      lcd.writeChar(byte(sc_state == TRG_OFF ? 5 : 6), 16, 2);
      processMessage();
    }

    void i2str(int i, char *buf){
      byte l=0;
      if(i<0) buf[l++]='-';
      boolean leadingZ=true;
      for(int div=10000, mod=0; div>0; div/=10){
        mod=i%div;
        i/=div;
        if(!leadingZ || i!=0){
           leadingZ=false;
           buf[l++]=i+'0';
        }
        i=mod;
      }
      buf[l]=0;
    }

    void measure() {
      DPRINTLN("measuring... ");
      bool ToR=device().getList0().TaraOnRestart();
      weight_sum = device().getList0().Tara();
      //DPRINT("Tara on Restart: ");DDECLN(ToR);
      for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
        lcd.waitProgress(true);
        if (first) {
          (ToR == true) ? hx711[i].tare(AVERAGE_READ_COUNT) : hx711[i].set_offset(HX711_OFFSETS[i]);
        }
        weight[i] = (int32_t)(hx711[i].get_units(AVERAGE_READ_COUNT));
        DPRINT(F("+Weight (#")); DDEC(i+1); DPRINT(F(") g: ")); DDECLN(weight[i]);
        weight_sum += weight[i];
      }

      DPRINT(F("+Tara        g: "));DDECLN(device().getList0().Tara());
      DPRINT(F("+Weight (#")); DDEC(sizeof(HX711_DOUT_PINS) + 1); DPRINT(F(") g: ")); DDECLN(weight_sum);

      if (weight_sum > (device().getList0().WeightLimit() + device().getList0().WeightHysteresis())) {
        if (sc_state == TRG_OFF) {
          DPRINTLN(F("ABOVE LIMIT SC TRIGGER"));
          scChanged(TRG_ON);
        }
      }

      if (weight_sum < (device().getList0().WeightLimit() - device().getList0().WeightHysteresis())) {
        if (sc_state == TRG_ON) {
          DPRINTLN(F("BELOW LIMIT SC TRIGGER"));
          scChanged(TRG_OFF);
        }
      }

      if (first) scChanged(sc_state);

      lcd.waitProgress(false);

      char intStr[17];
      //i2str(weight_sum,intStr);
      sprintf(intStr, "%d",weight_sum);
      strcat(intStr, " Gramm");
      lcd.writeLine(1, intStr, true, true);
      DPRINTLN(intStr);

      first = false;
    }

    void processMessage() {
      msg.init(device().nextcount(), weight, weight_sum);
      device().broadcastEvent(msg, *this);
      tickcnt = 0;
    }

    void irq() {
      sysclock.cancel(*this);
      measure();
      processMessage();
      sysclock.add(*this);
    }

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      measure();

      tick = seconds2ticks(10);

      tickcnt++;
      if (tickcnt == (device().getList0().Sendeintervall() / 10)) {
        processMessage();
      }
      sysclock.add(*this);
    }

    void init() {
      for (uint8_t i = 0; i < sizeof(HX711_DOUT_PINS); i++) {
        DPRINT(F("Init HX711"));
        hx711[i].begin(HX711_DOUT_PINS[i], HX711_SCK_PIN);
        for (uint8_t chk = 0; chk < 10; chk++) {
          hx711[i].power_up();
          DPRINT(F("."));
          if (hx711[i].is_ready()) {
            DPRINTLN(F(" OK"));
            hx711[i].set_scale(HX711_CALIBRATIONS[i] * 1.0); //(to ensure that it is a float)
            break;
          }
          if (chk == 9) {
            DPRINTLN(F(" ERR"));
            while(1) {}
          }
          _delay_ms(100);
        }
      }
    }

    virtual void configChanged() {
      DPRINT(F("*man. Tara       : ")); DDECLN(device().getList0().Tara());
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      pinMode(SC_TRIGGER_PIN, OUTPUT);
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      uint8_t flags = this->device().battery().low() ? 0x80 : 0x00;
      return flags;
    }
};

class UType : public ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 2, UList0> {
  public:
    VirtChannel<Hal, SCChannel, UList0>           channel1;
    VirtChannel<Hal, MeasureChannel, UList0>      channel2;
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, UList0>, 2, UList0> DeviceType;

    UType (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      DeviceType::registerChannel(channel1, 1);
      DeviceType::registerChannel(channel2, 2);
    }
    virtual ~UType () {}

    SCChannel& scChannel ()  {
      return channel1;
    }

    MeasureChannel& measureChannel ()  {
      return channel2;
    }

    void initLCD() {
      lcd.init();
      uint8_t serial[11];
      this->getDeviceSerial(serial);
      serial[10] = 0;
      lcd.writeLine(2, (char*)serial, true, true);
    }

    virtual void configChanged () {
      DeviceType::configChanged();
      DPRINT(F("*Sendeintervall  : ")); DDECLN(this->getList0().Sendeintervall());
      DPRINT(F("*TaraOnRestart   : ")); DDECLN(this->getList0().TaraOnRestart());
      DPRINT(F("*Tara            : ")); DDECLN(this->getList0().Tara());
      DPRINT(F("*WeightHysteresis: ")); DDECLN(this->getList0().WeightHysteresis());
      DPRINT(F("*WeightLimit     : ")); DDECLN(this->getList0().WeightLimit());
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  sdev.initLCD();
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  const uint8_t posmap[4] = {Position::State::PosA, Position::State::PosB, Position::State::PosA, Position::State::PosB};
  sdev.scChannel().init(SC_PIN, SC_PIN, posmap);
  sdev.scChannel().changed(true);
  sdev.measureChannel().init();
  sendISR(ISR_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if (isrDetected) {
      DPRINTLN(F("manual button pressed"));
      sdev.measureChannel().irq();
      isrDetected = false;
    }
    hal.activity.savePower<Idle<>>(hal);
  }
}
