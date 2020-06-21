//RFM69Pi V2 with RFM69CW Firmware
//Based on JCW RF12 Demo: https://github.com/jcw/jeelib/tree/master/examples/RF12/RF12demo
//Edited for RFM69Pi and emonPi operation December 2019 by JATG
//--------------------------------------------------------------------------------------------------------------------------------------------

#include <RF69.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define MAJOR_VERSION RF69_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[RF69demo.20]"           // keep in sync with the above

#define SERIAL_BAUD 38400   // max baud for 8Mhz RFM12Pi http://openenergymonitor.org/emon/node/6244/
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
#define LED_PIN     9       // activity LED, comment out to disable


/// Save a few bytes of flash by declaring const if used more than once.
const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

byte trace_mode = 0;

#define _receivePin 8
static int _bitDelay;
static char _receive_buffer;
static byte _receive_buffer_index;

// TODO: replace with code from the std avr libc library:
//  http://www.nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
void whackDelay (word delay) {
  byte tmp=0;

  asm volatile("sbiw      %0, 0x01 \n\t"
  "ldi %1, 0xFF \n\t"
  "cpi %A0, 0xFF \n\t"
  "cpc %B0, %1 \n\t"
  "brne .-10 \n\t"
  :
  "+r" (delay), "+a" (tmp)
  :
  "0" (delay)
);
}

ISR (PCINT0_vect) {
  char i, d = 0;
  if (digitalRead(_receivePin))       // PA2 = Jeenode DIO2
    return;             // not ready!
  whackDelay(_bitDelay - 8);
  for (i=0; i<8; i++) {
    whackDelay(_bitDelay*2 - 6);    // digitalread takes some time
    if (digitalRead(_receivePin)) // PA2 = Jeenode DIO2
      d |= (1 << i);
  }
  whackDelay(_bitDelay*2);
  if (_receive_buffer_index)
    return;
  _receive_buffer = d;                // save data
  _receive_buffer_index = 1;  // got a byte
}


static byte inChar () {
  byte d;
  if (! _receive_buffer_index)
    return -1;
  d = _receive_buffer; // grab first and only byte
  _receive_buffer_index = 0;
  return d;
}


static unsigned long now () {
  return millis() / 1000;
}

static void activityLed (byte on) {
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, on);
#endif
}

static void printOneChar (char c) {
  Serial.print(c);
}

/// @details
/// For the EEPROM layout, see http://jeelabs.net/projects/jeelib/wiki/RF69demo
/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/

// RF69 configuration area
typedef struct {
  byte nodeId;            // used by rf69_config, offset 0
  byte group;             // used by rf69_config, offset 1
  byte format;            // used by rf69_config, offset 2
  byte hex_output   : 2;   // 0 = dec, 1 = hex, 2 = hex+ascii
  byte collect_mode : 1;   // 0 = ack, 1 = don't send acks
  byte quiet_mode   : 1;   // 0 = show all, 1 = show only valid packets
  byte led_enable   : 1;    // enable led activity
  byte spare_flags  : 3;
  word frequency_offset;  // used by rf69_config, offset 4
  byte pad[RF69_EEPROM_SIZE-8];
  word crc;
 
}
RF69Config;

static RF69Config config;
static char cmd;
static word value;
static byte stack[RF69_MAXDATA+4], top, sendLen, dest;
static byte testCounter;

static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

static void showByte (byte value) {
  if (config.hex_output) {
    showNibble(value >> 4);
    showNibble(value);
  }
  else
    Serial.print((word) value);
}

static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
      break;
    if (c == '\n')
      printOneChar('\r');
    printOneChar(c);
  }
}

static word calcCrc (const void* ptr, byte len) {
  word crc = ~0;
  for (byte i = 0; i < len; ++i)
    crc = _crc16_update(crc, ((const byte*) ptr)[i]);
  return crc;
}

static void loadConfig () {
  // eeprom_read_block(&config, RF69_EEPROM_ADDR, sizeof config);
  // this uses 166 bytes less flash than eeprom_read_block(), no idea why
  for (byte i = 0; i < sizeof config; ++ i)
    ((byte*) &config)[i] = eeprom_read_byte(RF69_EEPROM_ADDR + i);
}

static void saveConfig () {
  config.format = MAJOR_VERSION;
  config.crc = calcCrc(&config, sizeof config - 2);
  // eeprom_write_block(&config, RF69_EEPROM_ADDR, sizeof config);
  // this uses 170 bytes less flash than eeprom_write_block(), no idea why
  eeprom_write_byte(RF69_EEPROM_ADDR, ((byte*) &config)[0]);
  for (byte i = 0; i < sizeof config; ++ i)
    eeprom_write_byte(RF69_EEPROM_ADDR + i, ((byte*) &config)[i]);

  if (rf69_configSilent())
    rf69_configDump();
  else
    showString(INITFAIL);
}

static byte bandToFreq (byte band) {
  return band == 4 ? RF69_433MHZ : band == 8 ? RF69_868MHZ : band == 9 ? RF69_915MHZ : 0;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#if DATAFLASH
#include "dataflash.h"
#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)
#define df_wipe()
#define df_append(x,y)

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

const char helpText1[] PROGMEM =
"\n"
"Available commands:\n"
"  <nn> i     - set node ID (standard node ids are 1..30)\n"
"  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
"  <nnnn> o   - change frequency offset within the band (default 1600)\n"
"               96..3903 is the range supported by the RFM12B\n"
"  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
"  <n> c      - set collect mode (advanced, normally 0)\n"
"  t          - broadcast max-size test packet, request ack\n"
"  ...,<nn> a - send data packet to node <nn>, request ack\n"
"  ...,<nn> s - send data packet to node <nn>, no ack\n"
"  <n> q      - set quiet mode (1 = don't report bad packets)\n"
"  <n> x      - set reporting format (0: decimal, 1: hex, 2: hex+ascii)\n"
"  <nnn> y    - enable signal strength trace mode, default:0 (disabled)\n"
"               sample interval <nnn> secs/100 (0.01s-2.5s) eg 10y=0.1s\n"
"  123 z      - total power down, needs a reset to start up again\n"
"  <n> l      - enable activity led (0 = disable led)\n"
;

const char helpText2[] PROGMEM =
"Flash storage (JeeLink only):\n"
"    d                                  - dump all log markers\n"
"    <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r    - replay from specified marker\n"
"    123,<bhi>,<blo> e                  - erase 4K block\n"
"    12,34 w                            - wipe entire flash memory\n"
;



static void showHelp () {
  showString(helpText1);
  if (df_present())
    showString(helpText2);
  showString(PSTR("Current configuration:\n"));
  rf69_configDump();
}

static void handleInput (char c) {
  if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
    return;
  }

  if (c == ',') {
    if (top < sizeof stack)
      stack[top++] = value; // truncated to 8 bits
    value = 0;
    return;
  }

  if ('a' <= c && c <= 'z') {
    showString(PSTR("> "));
    for (byte i = 0; i < top; ++i) {
      Serial.print((word) stack[i]);
      printOneChar(',');
    }
    Serial.print(value);
    Serial.println(c);
  }

  // keeping this out of the switch reduces code size (smaller branch table)
  if (c == '>') {
    // special case, send to specific band and group, and don't echo cmd
    // input: band,group,node,header,data...
    stack[top++] = value;
    // TODO: frequency offset is taken from global config, is that ok?
    rf69_initialize(stack[2], bandToFreq(stack[0]), stack[1],
    config.frequency_offset);
    rf69_sendNow(stack[3], stack + 4, top - 4);
    rf69_sendWait(2);
    rf69_configSilent();
  }
  else if (c > ' ') {
    switch (c) {

    case 'i': // set node id
      if ((value > 0) && (value < 31)) {
        config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
        saveConfig();
        }
      break;

    case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
      value = bandToFreq(value);
      if (value) {
        config.nodeId = (value << 6) + (config.nodeId & 0x3F);
        config.frequency_offset = 1600;
        saveConfig();
      }
      break;

    case 'o':
      { // Increment frequency within band
        // Stay within your country's ISM spectrum management guidelines, i.e.
        // allowable frequencies and their use when selecting operating frequencies.
        if ((value > 95) && (value < 3904)) { // supported by RFM12B
          config.frequency_offset = value;
          saveConfig();
        }
        break;
      }

    case 'g': // set network group
      config.group = value;
      saveConfig();
      break;

    case 'c': // set collect mode (off = 0, on = 1)
      config.collect_mode = value;
      saveConfig();
      break;

    case 't': // broadcast a maximum size test packet, request an ack
      cmd = 'a';
      sendLen = RF69_MAXDATA;
      dest = 0;
      for (byte i = 0; i < RF69_MAXDATA; ++i)
        stack[i] = i + testCounter;
      showString(PSTR("test "));
      showByte(testCounter); // first byte in test buffer
      ++testCounter;
      break;

    case 'a': // send packet to node ID N, request an ack
    case 's': // send packet to node ID N, no ack
      cmd = c;
      sendLen = top;
      dest = value;
      break;

    case 'z': // put the ATmega in ultra-low power mode (reset needed)
      if (value == 123) {
        showString(PSTR(" Zzz...\n"));
        Serial.flush();
        rf69_sleep(RF69_SLEEP);
        cli();
        //Sleepy::powerDown();
      }
      break;

    case 'q': // turn quiet mode on or off (don't report bad packets)
      config.quiet_mode = value;
      saveConfig();
      break;

    case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
      config.hex_output = value;
      saveConfig();
      break;

    case 'v': //display the interpreter version and configuration
      displayVersion();
      rf69_configDump();
      break;

    case 'l': // turn activity LED on or off
      config.led_enable = value;
      saveConfig();
      break;

    case 'd': // dump all log markers
      if (df_present())
        df_dump();
      break;

    case 'r': // replay from specified seqnum/time marker
      if (df_present()) {
        word seqnum = (stack[0] << 8) | stack[1];
        long asof = (stack[2] << 8) | stack[3];
        asof = (asof << 16) | ((stack[4] << 8) | value);
        df_replay(seqnum, asof);
      }
      break;

    case 'e': // erase specified 4Kb block
      if (df_present() && stack[0] == 123) {
        word block = (stack[1] << 8) | value;
        df_erase(block);
      }
      break;

    case 'w': // wipe entire flash memory
      if (df_present() && stack[0] == 12 && value == 34) {
        df_wipe();
        showString(PSTR("erased\n"));
      }
      break;

    case 'y': // turn signal strength trace mode on or off (rfm69 only)
      trace_mode = value;
      break;

    default:
      showHelp();
    }
  }

  value = top = 0;
}

static void displayASCII (const byte* data, byte count) {
  for (byte i = 0; i < count; ++i) {
    printOneChar(' ');
    char c = (char) data[i];
    printOneChar(c < ' ' || c > '~' ? '.' : c);
  }
  Serial.println();
}

static void displayVersion () {
  showString(PSTR(VERSION));
}

void setup () {
  
  activityLed(1);
  delay(1000); // shortened for now. Handy with JeeNode Micro V1 where ISP
  // interaction can be upset by RF69B startup process.

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  displayVersion();

  if (rf69_configSilent()) {
    loadConfig();
  }
  else {
    memset(&config, 0, sizeof config);
    config.nodeId = 0x4F;                           // RFM12Pi - 433 MHz, node 15
    config.group = 0xD2;                           // RFM12Pi - default group 210
    config.frequency_offset = 1600;
    config.quiet_mode = true;   // Default flags, quiet on
    saveConfig();
    rf69_configSilent();
  }

  rf69_configDump();
  df_initialize();

  delay(2000);        //rfm12pi keep LED for for 1s to show it's working at startup
  activityLed(0);
}

void loop () {
  if (Serial.available())
    handleInput(Serial.read());
  if (trace_mode == 0) {
  if (rf69_recvDone()) {
    byte n = rf69_len;
    
    if (rf69_crc == 0)
    {
      if (config.led_enable) activityLed(1);
      showString(PSTR("OK"));
    }
    else {
      if (config.quiet_mode)
        return;
      showString(PSTR(" ?"));
      if (n > 20) // print at most 20 bytes if crc is wrong
        n = 20;
    }
    if (config.hex_output)
      printOneChar('X');
    if (config.group == 0) {
      showString(PSTR(" G"));
      showByte(rf69_grp);
    }
    printOneChar(' ');
    showByte(rf69_hdr & RF69_HDR_MASK);
    for (byte i = 0; i < n; ++i) {
      if (!config.hex_output)
        printOneChar(' ');
      showByte(rf69_data[i]);
    }
    // display RSSI value after packet data
    showString(PSTR(" ("));
    if (config.hex_output)
      showByte(rssi);
    else
      Serial.print(-(rssi>>1));
    showString(PSTR(") "));

    Serial.println();

    if (config.hex_output > 1) { // also print a line as ascii
      showString(PSTR("ASC "));
      if (config.group == 0) {
        showString(PSTR(" II "));
      }
      printOneChar(rf69_hdr & RF69_HDR_DST ? '>' : '<');
      printOneChar('@' + (rf69_hdr & RF69_HDR_MASK));
      displayASCII((const byte*) rf69_data, n);
    }

    if (rf69_crc == 0) {
      if (config.led_enable) activityLed(1);

      if (df_present())
        df_append((const char*) rf69_data - 2, rf69_data + 2);

      if (RF69_WANTS_ACK && (config.collect_mode) == 0) {
        showString(PSTR(" -> ack\n"));
        rf69_sendStart(RF69_ACK_REPLY, 0, 0);
      }
      activityLed(0);
    }
  }

  if (cmd && rf69_canSend()) {
    if (config.led_enable) activityLed(1);
    showString(PSTR(" -> "));
    Serial.print((word) sendLen);
    showString(PSTR(" b\n"));
    byte header = cmd == 'a' ? RF69_HDR_ACK : 0;
    if (dest)
      header |= RF69_HDR_DST | dest;
    rf69_sendStart(header, stack, sendLen);
    cmd = 0;

    activityLed(0);
  }
  activityLed(0);

  } else {
      rf69_recvDone();
      byte y = (rssi>>1);
      for (byte i = 0; i < (100-y); ++i) {
          printOneChar('-');
      }
      Serial.print("*");
      for (byte i = 0; i < (y); ++i) {
          printOneChar(' ');
      }
      Serial.print(-y);
      Serial.println("dB");
      delay(trace_mode*10);

  }
}
