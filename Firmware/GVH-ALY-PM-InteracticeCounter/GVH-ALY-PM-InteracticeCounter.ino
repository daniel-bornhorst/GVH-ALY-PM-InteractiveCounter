#define DEBUG 1

#ifdef DEBUG
# define DEBUG_PRINTLN(x) Serial.println(x);
# define DEBUG_PRINT(x) Serial.print(x);
#else
# define DEBUG_PRINTLN(x)
# define DEBUG_PRINT(x)
#endif


#define MY_IP 10,32,16,232
#define MY_SUBNET 255,255,255,0
#define UDP_INCOMING_PORT 8888
#define UDP_REMOTE_PORT 9000


//Ethernet
#include <QNEthernet.h>
using namespace qindesign::network;
EthernetUDP Udp;

#include <OSCMessage.h>
OSCMessage msg;

/* Values for v1 protoboard
const uint8_t clockPin = 13;
const uint8_t shiftLoad = 11;
const uint8_t dataIn = 12;
*/
const uint8_t clockPin = 9;
const uint8_t shiftLoad = 7;
const uint8_t dataIn = 6;


elapsedMillis shiftTimer;
const unsigned int shiftInterval = 5;


byte incomingByte1;
byte incomingByte2;
byte incomingByte3;
uint32_t combinedBytes = 0;
uint32_t old_combinedBytes = 0;
int count = 0;

byte oldIncomingByte1;

void setup() {
  Serial.begin(9600); delay(10);
  
  pinMode(clockPin, OUTPUT);
  pinMode(shiftLoad, OUTPUT);
  pinMode(dataIn, INPUT_PULLUP);
  delay(1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(shiftLoad, HIGH);


  //Ethernet
	uint8_t mac[6];
	Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  Serial.printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	
	Ethernet.onLinkState([](bool state) {
		if (state) {
			Serial.printf("[Ethernet] Link ON: %d Mbps, %s duplex, %s crossover\r\n",
				Ethernet.linkSpeed(),
				Ethernet.linkIsFullDuplex() ? "full" : "half",
				Ethernet.linkIsCrossover() ? "is" : "not");
		} 
    else {
			Serial.printf("[Ethernet] Link OFF\r\n");
		}
	});


	Ethernet.begin({MY_IP}, {MY_SUBNET}, {MY_IP});
	
	
	if ( Udp.begin(UDP_INCOMING_PORT) == 1 ) {
		Serial.print("My IP address: ");
		Serial.println(Ethernet.localIP());	
	} 
  else {
		Serial.println("No ethernet");
	}
	
	Ethernet.setRetransmissionTimeout(20);
	Ethernet.setRetransmissionCount(1);

  shiftTimer = 0;

}

void loop() {

  if (shiftTimer >= shiftInterval) {
    digitalWrite(shiftLoad, LOW);
    delayMicroseconds(100);
    digitalWrite(shiftLoad, HIGH);
    delayMicroseconds(100);

    // DEBUG_PRINT("Touch Sample ");
    // DEBUG_PRINT(count++);
    // DEBUG_PRINT(": ");

    // Byte 3
    incomingByte1 = shiftIn(dataIn, clockPin, MSBFIRST);

    // if (oldIncomingByte1 != incomingByte3) {
    //   printBinary(incomingByte3);
    //   DEBUG_PRINTLN();
    //   oldIncomingByte1 = incomingByte3;
    // }
    //DEBUG_PRINT(bitRead(incomingByte3, 1)); // We only care about the two least significat bit of the first byte
    //DEBUG_PRINT(bitRead(incomingByte3, 0));
    //printBinary(incomingByte3);

    //DEBUG_PRINT(" ");

    // Byte 2
    incomingByte2 = shiftIn(dataIn, clockPin, MSBFIRST);
    //printBinary(incomingByte2);

    //DEBUG_PRINT(" ");

    // Byte 1
    incomingByte3 = shiftIn(dataIn, clockPin, MSBFIRST);
    //printBinary(incomingByte1);

    combinedBytes = combineBytes(incomingByte1, incomingByte2, incomingByte3);
    
    //printBinaryInt(combinedBytes);

    if (combinedBytes != old_combinedBytes) {
      DEBUG_PRINT("Touch Sample ");
      DEBUG_PRINT(count++);
      DEBUG_PRINT(": ");
      printBinaryInt(combinedBytes);
      DEBUG_PRINTLN();
      old_combinedBytes = combinedBytes;
    }

    //DEBUG_PRINTLN();

    //printTouchGrid(combinedBytes);
    

    //DEBUG_PRINTLN();
    
    shiftTimer = 0;
  }

  //OSCReceive();

}

uint32_t combineBytes(byte byte1, byte byte2, byte byte3) {
  uint32_t combinedBytes = 0;
  combinedBytes = (uint32_t)((byte1 << 16) | (byte2 << 8) | (byte3 << 0));
  return combinedBytes;
}

void printBinary(byte inByte) {
  for (int b = 7; b >= 0; b--) {
    DEBUG_PRINT(bitRead(inByte, b));
  }
}

void printBinaryInt(uint32_t inInt) {
  for (int b = 23; b >= 0; b--) {
    DEBUG_PRINT(bitRead(inInt, b));

    // add a space after each byte
    if (b % 8 == 0) {
      DEBUG_PRINT(" ");
    }
  }
}

void printTouchGrid(uint32_t touchData) {
  bool offsetToggle = false;
  Serial.print("  ");
  for (int i = 17; i >=0; i--) {
    Serial.print(bitRead(touchData, i));
    Serial.print("   ");

    if (i % 3 == 0) {
      Serial.println();
      if (offsetToggle){
        Serial.print("  ");
      }
      offsetToggle = !offsetToggle;
    }
  }
}

void OSCReceive() {
	int size;
	
	static unsigned long microsTime = 0;
	
	if( (size = Udp.parsePacket())>0) {
    microsTime = micros();
    while(size--) {
      //Serial.println(size);
      msg.fill(Udp.read());
    }
    if ((micros() - microsTime) > 10000) return; //Timeout for no eoP()

    if(!msg.hasError()) {
      //need to add all the show manager commands
      msg.dispatch("/test", testOSC);
    }	
    msg.empty();
  }
}


void testOSC(OSCMessage &msg ) {
  static int msgCount = 0;
	Serial.println("__TEST OSC__");	
  sendOSCMessage(msgCount++);
}


void sendOSCMessage(int argument){

  //The raw dog method for sending OSC
  OSCMessage msg("/BACK/AT/YA");
  msg.add(argument);
  Udp.beginPacket(Udp.remoteIP(), UDP_REMOTE_PORT);
  msg.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  msg.empty(); // free space occupied by message
}
