#include <avr/interrupt.h>
#include <util/crc16.h>
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRFMSB          0x07
#define REG_AFCFEI          0x1E
#define REG_RSSIVALUE       0x24
#define REG_DIOMAPPING1     0x25
#define REG_IRQFLAGS1       0x27
#define REG_IRQFLAGS2       0x28
#define REG_SYNCCONFIG      0x2E
#define REG_SYNCVALUE1      0x2F
#define REG_SYNCVALUE2      0x30
#define REG_SYNCVALUE3      0x31
#define REG_NODEADRS        0x39
#define REG_PACKETCONFIG2   0x3D
#define REG_AESKEY1         0x3E
#define MODE_SLEEP          0x00
#define MODE_STANDBY        0x04
#define MODE_RECEIVER       0x10
#define MODE_TRANSMITTER    0x0C
#define IRQ1_MODEREADY      0x80
#define IRQ1_RXREADY        0x40
#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_FIFOOVERRUN    0x10
#define IRQ2_PACKETSENT     0x08
#define IRQ2_PAYLOADREADY   0x04
#define DMAP1_PACKETSENT    0x00
#define DMAP1_PAYLOADREADY  0x40
#define DMAP1_SYNCADDRESS   0x80
#define AFC_CLEAR           0x02
#define RF_MAX                72

#define ROM_UINT8       const prog_uint8_t
#define ROM_READ_UINT8  pgm_read_byte
#define ROM_DATA        PROGMEM

#define IRQ_ENABLE      sei()
#define SS_DDR           DDRB
#define SS_PORT         PORTB
#define SS_BIT              2     // for PORTB: 2 = d.10, 1 = d.9, 0 = d.8
#define SPI_SS              2     // PB2
#define SPI_MOSI            3     // PB3
#define SPI_MISO            4     // PB4
#define SPI_SCK             5     // PB5

#define rf69_grp        rf69_buf[0]
#define rf69_hdr        rf69_buf[1]
#define rf69_len        rf69_buf[2]
#define rf69_data       (rf69_buf + 3)

#define RF69_HDR_CTL    0x80
#define RF69_HDR_DST    0x40
#define RF69_HDR_ACK    0x20
#define RF69_HDR_MASK   0x1F

#define RF69_MAXDATA    66

#define RF69_433MHZ     1   ///< RFM12B 433 MHz frequency band.
#define RF69_868MHZ     2   ///< RFM12B 868 MHz frequency band.
#define RF69_915MHZ     3   ///< RFM12B 915 MHz frequency band.

#define RF69_SLEEP           0     ///< Enter sleep mode.
#define RF69_WAKEUP         -1     ///< Wake up from sleep mode.
#define CSMA_LIMIT          -90    ///< Upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_CSMA_LIMIT_MS  1000   ///< Max time limit waiting for non carrier sensed

// EEPROM address range used by the rf69_config() code
#define RF69_EEPROM_ADDR    ((uint8_t*) 0x20)  ///< Starting offset.
#define RF69_EEPROM_SIZE    16                 ///< Number of bytes.
#define RF69_EEPROM_EKEY    ((uint8_t*) 0x40)  ///< EE start, same as before.
#define RF69_EEPROM_ELEN    16                 ///< EE number of bytes.
#define RF69_EEPROM_VERSION 1                  ///< Only this version is valid.

#define RF_SLEEP_MODE   0x8205

/// Shorthand to simplify detecting a request for an ACK.
#define RF69_WANTS_ACK ((rf69_hdr & RF69_HDR_ACK) && !(rf69_hdr & RF69_HDR_CTL))
/// Shorthand to simplify sending out the proper ACK reply.
#define RF69_ACK_REPLY (rf69_hdr & RF69_HDR_DST ? RF69_HDR_CTL : \
            RF69_HDR_CTL | RF69_HDR_DST | (rf69_hdr & RF69_HDR_MASK))

extern volatile uint16_t rf69_crc;
extern volatile uint8_t rf69_buf[];
extern uint8_t  rssi;

uint8_t control(uint8_t cmd, uint8_t val);
void setFrequency (uint32_t freq);
void configure ();
void interrupt();
int16_t readRSSI();
void sleep (bool off);
uint8_t rf69_initialize(uint8_t id, uint8_t band, uint8_t group=0xD4, uint16_t frequency=1600);
uint8_t rf69_configSilent();
void rf69_configDump();
uint8_t rf69_config(uint8_t show =1);
uint8_t rf69_recvDone(void);
uint8_t rf69_canSend(void);
void rf69_sendStart(uint8_t hdr, const void* ptr, uint8_t len);
void rf69_sendNow(uint8_t hdr, const void* ptr, uint8_t len);
void rf69_sendWait(uint8_t mode);
bool rf69_sendWithRetry (uint8_t nodeId,const void* ptr, uint8_t len,uint8_t retries=2, uint8_t retryWaitTime=40);
uint8_t rf69_receiveStart(uint8_t nodeIdSrc, const void* ptr, uint8_t len);
void rf69_sleep(char n);
void rf69_encrypt(const uint8_t*);
