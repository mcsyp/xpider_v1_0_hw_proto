#include <SoftwareSerial.h>
#include "Arduhdlc.h"
#include "Task.h"

#define RX_PIN 12
#define TX_PIN 13
SoftwareSerial mySerial(RX_PIN,TX_PIN);// RX, TX

#define MAX_HDLC_FRAME_LENGTH 32
void send_character(uint8_t data);
void hdlc_frame_handler(const uint8_t *data, uint16_t length);

/* Initialize Arduhdlc library with three parameters.
1. Character send function, to send out HDLC frame one byte at a time.
2. HDLC frame handler function for received frame.
3. Length of the longest frame used, to allocate buffer in memory */
Arduhdlc hdlc(&send_character, &hdlc_frame_handler, MAX_HDLC_FRAME_LENGTH);
/* Function to send out one 8bit character */
void send_character(uint8_t data) {
    mySerial.print((char)data);
}

typedef struct pack_dmp{
  int16_t index;
  int16_t heading;
}pack_dmp_t;

/* Frame handler function. What to do with received data? */
void hdlc_frame_handler(const uint8_t *data, uint16_t length) {
    // Do something with data that is in framebuffer
    pack_dmp_t * p = (pack_dmp_t*)data;
    Serial.print("index:");Serial.print(p->index);
    Serial.print(",heading:");Serial.println(p->heading);
}

#define TASK_WDT_INTERVAL 2000000 //sync every 2s
#define TASK_WDT_INDEX 10
typedef struct pack_wdt_sync
{
  int16_t index;
  pack_wdt_sync():index(TASK_WDT_INDEX){}
}pack_wdt_sync_t;
pack_wdt_sync_t g_packWdt;
Task g_taskWdt;
void task_wdt_sync(){
  Serial.println("Alive!=====================");
  hdlc.frameDecode((char*)&g_packWdt,sizeof(pack_wdt_sync_t));
}
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println("Hello world!");
    mySerial.begin(9600);

    g_taskWdt.init(TASK_WDT_INTERVAL,task_wdt_sync);
    
}

void loop() {
  while(mySerial.available()){
    hdlc.charReceiver(mySerial.read());  
  }
  long t = micros();
  g_taskWdt.trigger(t);
}
