#include <SoftwareSerial.h>
#include "Arduhdlc.h"
#include "task_dmp.h"
#include "task.h"

#define RST_PIN 13
#define RX_PIN 11
#define TX_PIN 10
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

#define TASK_DMP_UPLOAD_INTERVAL 100000 //100ms
#define TASK_DMP_UPLOAD_INDEX 24
typedef struct pack_dmp{
  int16_t index;
  int16_t heading;
  pack_dmp():index(TASK_DMP_UPLOAD_INDEX){}
}pack_dmp_t;
float g_yawDmp;
pack_dmp_t  g_packDmp;
Task g_taskDmp;
void task_dmp_upload(){
    g_packDmp.index = TASK_DMP_UPLOAD_INDEX;
    g_packDmp.heading = (int16_t)(g_yawDmp*18000.0f/3.14f);
    //Serial.print("heading:");Serial.println(g_packDmp.heading);
    hdlc.frameDecode((char*)&g_packDmp,sizeof(g_packDmp));
}

#define TASK_WDT_INTERVAL 2000000
#define TASK_WDT_MAX_COUNTER 3
#define TASK_WDT_INDEX 10
int g_task_wdt_counter=0;
Task g_taskWdt;
void task_wdt_sync(){
  if(g_task_wdt_counter>=TASK_WDT_MAX_COUNTER){
    digitalWrite(RST_PIN,HIGH);
    Serial.print("Watch dog reset triggered. Counter:");Serial.println(g_task_wdt_counter);
    delay(500);
    digitalWrite(RST_PIN,LOW);
    g_task_wdt_counter=0;
  }else{
    g_task_wdt_counter++;
  }
}
void task_reset_wdt()
{
  g_task_wdt_counter=0;
  
}


/* Frame handler function. What to do with received data? */
typedef struct pack_head{
  int16_t  index;
}pack_head_t;
void hdlc_frame_handler(const uint8_t *data, uint16_t length) {
    // Do something with data that is in framebuffer
    pack_head_t *ptr = (pack_head_t*)data;
    Serial.print("Frame:len=");Serial.print(length);
    Serial.print(",index=");Serial.println(ptr->index);
    
    switch(ptr->index){
      case TASK_WDT_INDEX:
        task_reset_wdt();
        break;
    }
}

void setup() {

    Serial.begin(115200); 
    mySerial.begin(9600);

    //task set up
    //dmp task update
    task_dmp_setup();//init the dmp task
    
    //dmp heading update
    g_taskDmp.init(TASK_DMP_UPLOAD_INTERVAL,task_dmp_upload);
    
    //task wdt sync 
    g_taskWdt.init(TASK_WDT_INTERVAL,task_wdt_sync);
    
    pinMode(RST_PIN,OUTPUT);
    digitalWrite(RST_PIN,LOW);
}


#define PERFORMACNE_WARNING_THRESHOLD 20
long start_ms=0;
long period_ms=0;
float yaw,pitch,roll;
void loop() {
    start_ms=millis();

    do{
      long t = micros();
      //do dmp job
      task_dmp_loop(&g_yawDmp, &pitch,&roll);
      
      //do dmp update
      g_taskDmp.trigger(t);
      
      //do wdt sync
      g_taskWdt.trigger(t);
    }while(0);

    //receive the soft serial hdlc data,
    while(mySerial.available()){
      hdlc.charReceiver(mySerial.read());
    }

    period_ms = millis()-start_ms;
    if(period_ms>PERFORMACNE_WARNING_THRESHOLD){
      Serial.print("Perforamnce WARINING!! Cycle time:  ");
      Serial.print(period_ms);
      Serial.println("ms.");
    }
    delay(5);
}
