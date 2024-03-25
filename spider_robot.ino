/*
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols (except Bang&Olufsen) are active.
 * This must be done before the #include <IRremote.hpp>
 */
//#define DECODE_DENON        // Includes Sharp
//#define DECODE_JVC
//#define DECODE_KASEIKYO
//#define DECODE_PANASONIC    // alias for DECODE_KASEIKYO
//#define DECODE_LG
//#define DECODE_NEC          // Includes Apple and Onkyo
//#define DECODE_SAMSUNG
//#define DECODE_SONY
//#define DECODE_RC5
#define DECODE_RC6

//#define DECODE_BOSEWAVE
//#define DECODE_LEGO_PF
//#define DECODE_MAGIQUEST
//#define DECODE_WHYNTER
//#define DECODE_FAST

//#define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width protocols
//#define DECODE_HASH         // special decoder for all protocols

//#define DECODE_BEO          // This protocol must always be enabled manually, i.e. it is NOT enabled if no protocol is defined. It prevents decoding of SONY!

//#define DEBUG               // Activate this for lots of lovely debug output from the decoders.

//#define RAW_BUFFER_LENGTH  180  // Default is 112 if DECODE_MAGIQUEST is enabled, otherwise 100.

#include <Arduino.h>
#include <Servo.h>

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */


#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library

#define IR_RECEIVE_PIN      A3

Servo servo[8]; 


 //       
 // 7()   front   4(8)
 //   3(8)_____0(10)
 //      |     |
 //      |_____| 
 //   2(2)     1(6)
 //  6(4)          5(7)


//const uint8_t servoPin[8] = {2,   4,   6,   8,       3,   5,   7,  9};
const uint8_t servoPin[8] = {10,   6,   2,   3,       8,   7,   4,  5};


const uint8_t servoMapToLow[8]  = {0, 190,   10,   200,      0,   180,   0, 180};
const uint8_t servoMapToHigh[8] = {200, 0, 200,     0,     190,     0, 180,   0};

const uint8_t servoLimitMin[8]  = {0,   0,   0,   0,     0,   0,   0,   0};
const uint8_t servoLimitMax[8]  = {180, 180, 180, 180,   180, 180, 180, 180};

uint8_t startServoAngle[8]   = {90, 90, 90, 90, 90, 90, 90, 90};
uint8_t currentServoAngle[8] = {90, 90, 90, 90, 90, 90, 90, 90};
uint8_t tergetServoAngle[8]  = {90, 90, 90, 90, 90, 90, 90, 90};


uint16_t targetTimeCount  = 0;
uint16_t recievTimeCount  = 0;
uint16_t currentTimeCount = 0;

uint16_t targetAddress  = 0;
uint16_t recievAddress  = 0;
uint16_t currentAddress = 0;
uint8_t isCommandReciev = 0;
uint8_t commandReciev = 0xff;

uint8_t lenghtLineCurrentCommand       = 0;
uint8_t recievLenghtLineCurrentCommand = 0;

const uint8_t commandNumber[] = {
  0x00, 0, // 0
  0x01, 1, // 1
  0x02, 2, // 2
  0x03, 3, // 3
  0x04, 4, // 4
  0x05, 5, // 5
  0x06, 6, // 6
  0x07, 7, // 7
  0x08, 8, // 8
  0x09, 9, // 9
  0x0D, 0, // mute
  0x0F, 0, // display
  0x20, 0, // next
  0x21, 0, // prev
  0x31, 0, //stop
  0x4B, 0, //subtitle
  0x4E, 0, //audio
  0x58, 10,  // up
  0x59, 11,  // down
  0x5A, 12,  // left
  0x5B, 13,  // right
  0x5C, 14,  // ok
  0x82, 0,  // setup
  0x83, 0,  // title
  0xC7, 15,  // power
  0xD1, 0,  // menu
  0xE7, 0, //vocal
  0xE8, 0, //karaoke
  0xF7, 0, // zoom
};

const uint8_t lenghtLineCommand[] = {
  1, // 0
  1, // 1
  1, // 2
  1, // 3
  1, // 4
  8, // 5
  4, // 6
  4, // 7
  16, // 8
  8, // 9
  4, //10 вперед
  4, //11 назад  
  4, //12 лево
  4, //13 право 
  10, //14
  1, //15
  0,  // up
  0,  // down
  0,    // <
  0,  // >
  0,  // ok

  
};

const uint8_t data[] PROGMEM = {

//       
//    135_[7]     front     [4]_135
//       /|  90 135   135 90  |\ 
//    90 45    \ |     | /    45 90
//         45 _[3]_____[0]_ 45
//               |     |       
//         45 _  |_____|  _ 45
//             [2]     [1]
//            /  |     |  \  
//          90  135   135  90 
//   135_[6]                 [5]_135
//      /|                     |\ 
//    90  45                  45 90

//count_times 1 = 0.01 sec
//  ct,   0,   1,   2,   3,       4,   5,   6,   7,

//0
    20,   90,  90,  90,  90,      90,  90,  90,  90,
//1    
    20,  135,  135,  45,  45,      45,  45,  45,  45,
//2
    20,  45, 45, 135, 135,     135, 135, 135, 135,
//3 
    20,  0,  0,  135,  135,   90,  90,  90,  90,
//2
    20,  135, 135, 0, 0,     90,  90,  90,  90,   

//5 
    15,   90, 110,  90,  70,      90,  90,  90,  90, // подал тело вперед
    15,   90, 110,  90,  70,      90, 120,  90, 120, // поднял две другие ноги по диагонали 2
    15,   90,  70,  90, 110,      90, 120,  90, 120, // подал вперед две поднятые ноги
    15,   90,  70,  90, 110,      90,  90,  90,  90, // опустил две поднятые ноги
    15,   70,  90,  110, 90,      90,  90,  90,  90, // подал тело вперед
    15,   70,  90,  110, 90,     120,  90, 120,  90, // поднял две ноги по диагонали 1
    15,  110,  90,  70,  90,     120,  90, 120,  90, // подал вперед две поднятые ноги
    15,  110,  90,  70,  90,      90,  90,  90,  90, // опустил две поднятые ноги
  
  //6 

    15,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    15,  110, 110,  70,  70,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    15,   90,  90,  90,  90,      110, 90, 110,  90, // поднимаем ноги
    15,   70,  70, 110, 110,      90,  90,  90,  90, // подал тело вперед
  //7 

    10,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    10,  110, 110,  45,  45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    10,   90,  90,  90,  90,      110, 90, 110,  90, // поднимаем ноги
    10,   45,  45, 110, 110,      90,  90,  90,  90, // подал тело вперед

  //8
    20,   45, 90,  90,  90,      90,  90,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 45,  90,  90,      90,  90,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  45,  90,      90,  90,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  90,  45,      90,  90,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  90,  90,      45,  90,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  90,  90,      90,  45,  90,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  90,  90,      90,  90,  45,  90,
    20,   90, 90,  90,  90,      90,  90,  90,  90,

    20,   90, 90,  90,  90,      90,  90,  90,  45,
    20,   90, 90,  90,  90,      90,  90,  90,  90,
  
  // 9

  //       
//    135_[7]     front     [4]_135
//       /|  90 135   135 90  |\ 
//    90 45    \ |     | /    45 90
//         45 _[3]_____[0]_ 45
//               |     |       
//         45 _  |_____|  _ 45
//             [2]     [1]
//            /  |     |  \  
//          90  135   135  90 
//   135_[6]                 [5]_135
//      /|                     |\ 
//    90  45                  45 90

    // 20,   120, 100,  80,  60,      120,  80,  80,  80,
    // 20,   120, 100,  80,  60,      80,  80,  80,  120,
    // 20,   100, 80,  60,  120,      80,  80,  80,  120,
    // 20,   100, 80,  60,  120,      80,  80,  120,  80,
    // 20,   80, 60,  120,  100,      80,  80,  120,  80,
    // 20,   80, 60,  120,  100,      80,  120,  80,  80,
    // 20,   60, 120, 100,  80,       80,  120,  80,  80,
    // 20,   60, 120, 100,  80,       120,  80,  80,  80,


    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,  110, 110,  45,  45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      110, 90, 110,  90, // поднимаем ноги
    20,   45,  45, 110, 110,      90,  90,  90,  90, // подал тело вперед
    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,  110, 110,  45,  45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      110, 90, 110,  90, // поднимаем ноги
    20,   45,  45, 110, 110,      90,  90,  90,  90, // подал тело вперед
  
  
  //10 вперед
    20,   90,  90,  90,  90,      110, 90, 110,  90,  // поднимаем ноги
    20,  110, 110,  45,  45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,   45,  45, 110, 110,      90,  90,  90,  90, // подал тело вперед
  
    //11 назад
    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,  110, 110,  45,  45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      110, 90, 110,  90, // поднимаем ноги
    20,   45,  45, 110, 110,      90,  90,  90,  90, // подал тело вперед

    //12 лево
    20,   90,  90,  90,  90,      110, 90, 110,  90,  // поднимаем ноги
    20,  110, 110,  110,  110,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,   45,  45, 45, 45,      90,  90,  90,  90, // подал тело вперед


    //13 право
    20,   90,  90,  90,  90,      110, 90, 110,  90,  // поднимаем ноги
    20,    45,  45, 45, 45,      90,  90,  90,  90, // поднял две другие ноги по диагонали 2
    20,   90,  90,  90,  90,      90, 110,  90, 110, // поднимаем ноги
    20,    110, 110,  110,  110,   90,  90,  90,  90, // подал тело вперед

    //    135_[7]     front     [4]_135
//       /|  90 135   135 90  |\ 
//    90 45    \ |     | /    45 90
//         45 _[3]_____[0]_ 45
//               |     |       
//         45 _  |_____|  _ 45
//             [2]     [1]
//            /  |     |  \  
//          90  135   135  90 
//   135_[6]                 [5]_135
//      /|                     |\ 
//    90  45                  45 90

//14  махать
    20,   100,  80,  80,  100,      90, 90, 90,  90,  
    20,   100,  80,  80,  100,      180, 90, 90,  90,
    20,   130,  80,  80,  100,      180, 90, 90,  90, 
    20,   50,  80,  80,  100,      180, 90, 90,  90,  
    20,   130,  80,  80,  100,      180, 90, 90,  90, 
    20,   50,  80,  80,  100,      180, 90, 90,  90, 
    20,   130,  80,  80,  100,      180, 90, 90,  90, 
    20,   50,  80,  80,  100,      180, 90, 90,  90, 
    20,   100,  80,  80,  100,      90, 90, 90,  90,
    20,   90, 90, 90,  90,      90, 90, 90,  90,

//15  лежать
    20,   130,  130,  130,  130,      130, 130, 130,  130,  



  };

void setup() {
    Serial.begin(115200);
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

    for (byte i = 0; i <= 7; i++) {
      servo[i].attach(servoPin[i]);
    }
}

// uint16_t getAddressByCommand2(uint8_t command) {
//   uint16_t localAddress = 0;
//   for (int i = 0; i < sizeof(commandNumber) / 2; i++) {
//     recievLenghtLineCurrentCommand = lenghtLineCommand[commandNumber[i * 2 + 1]];
      
//       Serial.print("command: ");
//       Serial.print(commandNumber[i * 2], HEX);
//       Serial.print(" commandNumber: ");
//       Serial.print(commandNumber[i * 2 + 1]);
//       Serial.print(" lenghtLineCommand: ");
//       Serial.print(recievLenghtLineCurrentCommand);
//       Serial.println();

//     if (commandNumber[i * 2] == command) {
//       return localAddress;
//     } else {
//       localAddress += recievLenghtLineCurrentCommand * 9;
//     }
//   }
// }

uint16_t getAddressByCommand(uint8_t command) {
  uint16_t localAddress = 0;
  for (uint16_t i = 0; i < sizeof(commandNumber) / 2; i++) {
    if (commandNumber[i * 2] == command) {
      // Serial.print("command: ");
      // Serial.print(commandNumber[i * 2], HEX);
      // Serial.print(" commandNumber: ");
      // Serial.print(commandNumber[i * 2 + 1]);
      // Serial.println();


      for (uint16_t ii = 0; ii < commandNumber[i * 2 + 1]; ii++) {
        localAddress += lenghtLineCommand[ii];
        // Serial.print(" ii: ");
        // Serial.print(ii);
        // Serial.print(" lenghtLineCommand: ");
        // Serial.print(lenghtLineCommand[ii]);
        // Serial.println();

      }
      recievLenghtLineCurrentCommand = lenghtLineCommand[commandNumber[i * 2 + 1]];
      // Serial.print(" lenghtLineCommand: ");
      // Serial.print(recievLenghtLineCurrentCommand);
      // Serial.print(" localAddress line: ");
      // Serial.print(localAddress);
      // Serial.println();

      return localAddress * 9;
    } 
  }
}

void loop() {
  if (IrReceiver.decode()) {

      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
        //  Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
          // We have an unknown protocol here, print more info
        //  IrReceiver.printIRResultRawFormatted(&Serial, true);
      } else {
        Serial.println();
        Serial.print("IR command: 0x");
        Serial.println(IrReceiver.decodedIRData.command, HEX);


      if (commandReciev != IrReceiver.decodedIRData.command) {
        commandReciev = IrReceiver.decodedIRData.command;
        recievAddress = getAddressByCommand(IrReceiver.decodedIRData.command);
      }
       
      isCommandReciev = 1;
      for (byte i = 0; i <= 7; i++) {
        if (!servo[i].attached()) {
          servo[i].attach(servoPin[i]);
        }
      }
      //   if (lenghtLineCurrentCommand > 0) {
      //     if (recievAddress == targetAddress) {
            
      //     } else {
      // //      targetAddress = recievAddress;
      // //      currentAddress = recievAddress;
      //       recievTimeCount = pgm_read_byte(&data[recievAddress]);
      //       currentTimeCount = recievTimeCount;
      //       targetTimeCount = recievTimeCount;
      //       lenghtLineCurrentCommand = recievLenghtLineCurrentCommand + 1;
      //     }
      //   } else {
      // //    targetAddress = recievAddress;
      // //    currentAddress = recievAddress;
      //     recievTimeCount = pgm_read_byte(&data[recievAddress]);
      //     currentTimeCount = recievTimeCount;
      //     targetTimeCount = recievTimeCount;
      //     lenghtLineCurrentCommand = recievLenghtLineCurrentCommand + 1;
      //   }
      }
      IrReceiver.resume(); // Enable receiving of the next value
  }

  static uint32_t tmr;
  if (millis() - tmr >= 10) {
    tmr = millis();
    if (lenghtLineCurrentCommand > 0 ) {
      if (currentTimeCount < targetTimeCount ) {
        currentTimeCount++;
        uint8_t percent = 100 * currentTimeCount / targetTimeCount;
        Serial.print(".");
        for (byte i = 0; i <= 7; i++) {
          if (tergetServoAngle[i] > startServoAngle[i]) {
            currentServoAngle[i] = startServoAngle[i] + (tergetServoAngle[i] * percent - startServoAngle[i] * percent) / 100;
          } else {
            currentServoAngle[i] = startServoAngle[i] - (startServoAngle[i] * percent - tergetServoAngle[i] * percent) / 100;
          }
          uint8_t value = map(currentServoAngle[i],0, 180, servoMapToLow[i], servoMapToHigh[i]);
          if (value > servoLimitMax[i]) {
            value = servoLimitMax[i];
          } else if (value < servoLimitMin[i]) {
            value = servoLimitMin[i];
          }
          servo[i].write(value); 
        }      
      } else {
        lenghtLineCurrentCommand--;
        Serial.println();
        Serial.print("lenghtLineCurrentCommand: ");
        Serial.println(lenghtLineCurrentCommand);

        if (lenghtLineCurrentCommand > 0) {
          targetTimeCount = pgm_read_byte(&data[currentAddress]);
          currentTimeCount = 0;

          Serial.print("current ");
          for (byte i = 0; i <= 7; i++) {
            Serial.print(currentServoAngle[i]);
            Serial.print(" ");
          } 
          Serial.println();

          Serial.print("target ");
          for (byte i = 0; i <= 7; i++) {
            tergetServoAngle[i] = pgm_read_byte(&data[i + currentAddress + 1]);
            startServoAngle[i] = currentServoAngle[i];
            Serial.print(tergetServoAngle[i]);
            Serial.print(" ");
          } 
          Serial.println();

          currentAddress+=9;
        }
      }
    } else {
      if (isCommandReciev) {
        isCommandReciev = 0;
        targetAddress = recievAddress;
        currentAddress = recievAddress;
        recievTimeCount = pgm_read_byte(&data[recievAddress]);
        currentTimeCount = recievTimeCount;
        targetTimeCount = recievTimeCount;
        lenghtLineCurrentCommand = recievLenghtLineCurrentCommand + 1;
      } else {
        for (byte i = 0; i <= 7; i++) {
          servo[i].detach();
        }
      }
    }
  }
}


