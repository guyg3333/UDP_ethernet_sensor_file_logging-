#include <EtherCard.h>
#include <Servo.h>

#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2


static byte mymac[] = { 0x1A,0x2B,0x3C,0x4D,0x5E,0x6F };
static byte dstmac[] = {0xB8,0x27,0xEB,0x77,0xf9,0xE1 };

typedef union _group_16 {
  int u16;
  uint8_t u8[2];
} group_16;



Servo myservo;
byte Ethernet::buffer[700];
static uint32_t timer;

group_16 pos; 

//const char website[] PROGMEM = "server.example.net";
const int dstPort PROGMEM = 25001;
const int srcPort PROGMEM = 25001;

//------------ PWM

volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unThrottleInShared;
volatile uint32_t ulThrottleStart;
volatile uint32_t ulSteeringStart;
volatile unsigned long pulse_time  ;


//------------

void setup () {
  Serial.begin(9600);


  attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */,calcThrottle,CHANGE);
  attachInterrupt(1 /* INT1 = STEERING_IN_PIN */,calcSteering,CHANGE);

 
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
    Serial.println( "Failed to access Ethernet controller");
  
  

//---------------------------
//  if (!ether.dnsLookup(website))
//    Serial.println("DNS failed");
//--------------------------------

//dest_ip

  ether.hisip[0] = 10;
  ether.hisip[1] = 100;
  ether.hisip[2] = 102;
  ether.hisip[3] = 3;

//dest_ip

  ether.myip[0] = 10;
  ether.myip[1] = 100;
  ether.myip[2] = 102;
  ether.myip[3] = 2;


ether.copyMac(ether.buffer + ETH_SRC_MAC, mymac);
  

}

char textToSend[2] = {0};

void loop () {  

Serial.println("test");


if(bUpdateFlagsShared)
{
  
  pos.u16 =  unSteeringInShared;
  textToSend[0] = pos.u8[0];
  textToSend[1] = pos.u8[1];
  
 
  
    if (millis() > timer) {
      timer = millis(); 
      ether.copyMac( ether.buffer + ETH_DST_MAC, dstmac); 
     //static void sendUdp (char *data,uint8_t len,uint16_t sport, uint8_t *dip, uint16_t dport);      
     ether.sendUdpBGR(textToSend, sizeof(textToSend), srcPort, ether.hisip, dstPort , dstmac );   
      Serial.println(unSteeringInShared);
      bUpdateFlagsShared = 0;
  }
   delay(20);
}
  
}



// -------------------------------- PWM ---------------------------------


// simple interrupt service routine
void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    unSteeringInShared =  map(unSteeringInShared,1170,1670,0,180);
if (unSteeringInShared<=15 || unSteeringInShared >200 )
{
unSteeringInShared = 0;
}
   //Serial.println(unSteeringInShared);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

//------------------------------ PWM ---------------------------------------------
