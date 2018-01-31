// Include the libraries we need
//#include <OneWire.h>
#include <DallasTemperature.h>
//#include <EtherCard.h>


// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 7
#define THERMISTOT_PIN 3
#define EROR_PIN 5
#define TEMPERATURE_PRECISION 9


//temp variable 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress  device[4]; //one, two,tree,forw;

/*----------Ethernet----------------*/

//static byte mymac[] = { 0x1A,0x2B,0x3C,0x4D,0x5E,0x6F };
//static byte dstmac[] = {0xE0,0x69,0x95,0x3E,0xCD,0xAB };

typedef union _group_32 {
  float u32;
  uint8_t u8[4];
} group_32;



int ThermistorPin = 0;
int Vo;
float R1 = 2040;
float R2, T , logR2;

//const int dstPort PROGMEM = 25001;
//const int srcPort PROGMEM = 25001;

//byte Ethernet::buffer[700];


unsigned char sens_working[4]; 
int sens_count = 0;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  pinMode(EROR_PIN , OUTPUT);


  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  // 
  // method 1: by index
  if (!sensors.getAddress(device[0], 0)) 
  {
    Serial.println("Unable to find address for Device 1"); 
    sens_working[0] = 0x00;
  }
  else
  {
      Serial.print("Device 1 Address: ");
      printAddress(device[0]);
      sens_working[0] = 0x01;
      sens_count++;
      Serial.println();
  }
  
  if (!sensors.getAddress(device[1], 1)) 
  {
    Serial.println("Unable to find address for Device 2");
    sens_working[1] = 0x00;
  }
  else
  {
      Serial.print("Device 2 Address: ");
      printAddress(device[1]);
      sens_working[1] = 0x01;
      sens_count++;
      Serial.println();
  }
  
  if (!sensors.getAddress(device[2], 2)) 
  {
    Serial.println("Unable to find address for Device 3");
    sens_working[2] = 0x00;
  }
  else
  {
      Serial.print("Device 3 Address: ");
      printAddress(device[2]);
      sens_working[2] = 0x01;
      sens_count++;
      Serial.println();
  }
  
  if (!sensors.getAddress(device[3], 3)) 
  {
    Serial.println("Unable to find address for Device 4"); 
    sens_working[3] = 0x00;
  }
  else
  {
    
      Serial.print("Device 4 Address: ");
      printAddress(device[3]);
      sens_working[3] = 0x01;
      sens_count++;
      Serial.println(); 
  }

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
  // assigns the seconds address found to outsideThermometer
  //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

  // show the addresses we found on the bus
  
  #if 0
  Serial.print("Device 1 Address: ");
  printAddress(device[0]);
  Serial.println();

  Serial.print("Device 2 Address: ");
  printAddress(device[1]);
  Serial.println();

    Serial.print("Device 3 Address: ");
  printAddress(device[2]);
  Serial.println();

  Serial.print("Device 4 Address: ");
  printAddress(device[3]);
  Serial.println();

    sensors.setResolution(device[0], TEMPERATURE_PRECISION);
  sensors.setResolution(device[1], TEMPERATURE_PRECISION);
  sensors.setResolution(device[2], TEMPERATURE_PRECISION);
  sensors.setResolution(device[3], TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(device[0]), DEC); 
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(device[1]), DEC); 
  Serial.println();

   Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(device[2]), DEC); 
  Serial.println();

   Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(device[3]), DEC); 
  Serial.println();
  
#endif

  // set the resolution to 9 bit per device
  if( sens_working[0] == 0x01 )
  {
    sensors.setResolution(device[0], TEMPERATURE_PRECISION);
      Serial.print("Device 0 Resolution: ");
     Serial.print(sensors.getResolution(device[0]), DEC); 
    Serial.println();
  }

  if( sens_working[1] == 0x01 )
  {
    sensors.setResolution(device[1], TEMPERATURE_PRECISION);
      Serial.print("Device 1 Resolution: ");
     Serial.print(sensors.getResolution(device[1]), DEC); 
    Serial.println();
  }

    if( sens_working[2] == 0x01 )
  {
    sensors.setResolution(device[2], TEMPERATURE_PRECISION);
      Serial.print("Device 2 Resolution: ");
     Serial.print(sensors.getResolution(device[2]), DEC); 
    Serial.println();
  }
      if( sens_working[3] == 0x01 )
  {
    sensors.setResolution(device[3], TEMPERATURE_PRECISION);
      Serial.print("Device 3 Resolution: ");
     Serial.print(sensors.getResolution(device[3]), DEC); 
    Serial.println();
  }


/*------- Ethernet ---------*/
/*
 if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
    Serial.println( "Failed to access Ethernet controller");

//dest_ip

  ether.hisip[0] = 192;
  ether.hisip[1] = 168;
  ether.hisip[2] = 137;
  ether.hisip[3] = 1;

//dest_ip

  ether.myip[0] = 192;
  ether.myip[1] = 168;
  ether.myip[2] = 137;
  ether.myip[3] = 23;


  ether.copyMac(ether.buffer + ETH_SRC_MAC, mymac);
*/
}//setup

unsigned char textToSend[16] = {0};

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();    
}

/*
 * Main function, calls the temperatures in a loop.
 */
void loop(void)
{ 
float temp;
unsigned char i,j,num;
group_32  tempC[4]; // tempC_1,tempC_2,tempC_3,tempC_4;
// call sensors.requestTemperatures() to issue a global temperature 
// request to all devices on the bus

while(1){
//Serial.print("Requesting temperatures...");
sensors.requestTemperatures();
//Serial.println("DONE");

  Vo = analogRead(THERMISTOT_PIN);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = 7e-13*pow(R2,4) - 4e-09*pow(R2,3) - 6e-06*pow(R2,2) + 0.1113*R2 - 154.89; 

  Serial.print("Temperature in engine: "); 
  Serial.print(T);
  Serial.print("C  "); 



if( sens_working[0] == 0x01 )
{
  temp = sensors.getTempC(device[0]);
    //if(temp!= -127)
      tempC[0].u32 = temp;

    Serial.print("IN_INVERTER:");  
    Serial.print(tempC[0].u32);
    Serial.print("C  ");
  
}
if( sens_working[1] == 0x01 )
{
    temp = sensors.getTempC(device[1]);
    //if(temp!= -127)
      tempC[1].u32 = temp;

    Serial.print("OUT_INVERTER:");  
    Serial.print(tempC[1].u32);
    Serial.print("C  "); 
}
if( sens_working[2] == 0x01 )
{
    temp = sensors.getTempC(device[2]);
    //if(temp!= -127)
      tempC[2].u32 = temp;

    Serial.print("OUT_ENGINE:");  
    Serial.print(tempC[2].u32);
    Serial.print("C  "); 
  
}
if( sens_working[3] == 0x01 )
{
    temp = sensors.getTempC(device[3]);
    //if(temp!= -127)
      tempC[3].u32 = temp;

    Serial.print("4:");  
    Serial.print(tempC[3].u32);
    Serial.print("C  "); 
  
}

Serial.println("");


  if( ( ( tempC[0].u32 > 60.0 ) && sens_working[0]  ) || ( ( tempC[1].u32 > 60.0 ) && sens_working[1]  ) || ( ( tempC[2].u32 > 60.0 ) && sens_working[2]  ) || ( ( tempC[3].u32 > 60.0 ) && sens_working[3]  ) || T > 80.0 )
  {

    digitalWrite( EROR_PIN , HIGH );
  }
  else
  {
    digitalWrite( EROR_PIN , LOW );
  }

#if 0
for(i = 0 ; i < sens_count ; i++){
  temp = sensors.getTempC(device[i]);
  if(temp!= -127)
   tempC[i].u32 = temp;
    Serial.print(tempC[i].u32);
    Serial.print(" ");

  }
  
  Serial.println("");



#endif  

 /*
  tempC_1.u32 = sensors.getTempC(one);
  tempC_2.u32 = sensors.getTempC(two);
  tempC_3.u32 = sensors.getTempC(tree);
  tempC_4.u32 = sensors.getTempC(forw);
*/
/*

for(i=0;i<4;i++)
    textToSend[i] = tempC[0].u8[i];


for(i=0;i<4;i++)
    textToSend[4+i] = tempC[1].u8[i];

    
for(i=0;i<4;i++)
    textToSend[8+i] = tempC[2].u8[i];

    
for(i=0;i<4;i++)
    textToSend[12+ i] = tempC[3].u8[i];

  
   // ether.copyMac( ether.buffer + ETH_DST_MAC, dstmac); 
    //static void sendUdp (char *data,uint8_t len,uint16_t sport, uint8_t *dip, uint16_t dport);      
   //  ether.sendUdpBGR(textToSend, sizeof(textToSend), srcPort, ether.hisip, dstPort , dstmac );   
  
*/

 delay(20);
  }//while
}
