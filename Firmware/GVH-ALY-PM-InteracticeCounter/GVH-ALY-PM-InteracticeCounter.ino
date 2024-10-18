#define DEBUG 1

#ifdef DEBUG
# define DEBUG_PRINTLN(x) Serial.println(x);
# define DEBUG_PRINT(x) Serial.print(x);
#else
# define DEBUG_PRINTLN(x)
# define DEBUG_PRINT(x)
#endif

const uint8_t clockPin = 13;
const uint8_t shiftLoad = 11;
const uint8_t dataIn = 12;

byte incoming;
int count = 0;

void setup() {
  Serial.begin(9600); delay(10);
  
  pinMode(clockPin, OUTPUT);
  pinMode(shiftLoad, OUTPUT);
  pinMode(dataIn, INPUT_PULLUP);
  delay(1);
  digitalWrite(clockPin, HIGH);
  digitalWrite(shiftLoad, HIGH);


}

void loop() {

  digitalWrite(shiftLoad, LOW);
  delay(1);
  digitalWrite(shiftLoad, HIGH);
  delay(1);

  incoming = shiftIn(dataIn, clockPin, LSBFIRST);
  DEBUG_PRINT("byte ");
  DEBUG_PRINT(count++);
  DEBUG_PRINT(": ");
  printBinary(incoming);

  incoming = shiftIn(dataIn, clockPin, LSBFIRST);
  DEBUG_PRINT(", ");
  printBinary(incoming);

  incoming = shiftIn(dataIn, clockPin, LSBFIRST);
  DEBUG_PRINT(", ");
  printBinary(incoming);

  DEBUG_PRINTLN();
  delay(2000);

}

void printBinary(byte inByte) {
  for (int b = 7; b >= 0; b--) {
    DEBUG_PRINT(bitRead(inByte, b));
  }
}
