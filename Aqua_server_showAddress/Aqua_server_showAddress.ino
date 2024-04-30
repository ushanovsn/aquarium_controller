#include <OneWire.h>
#include "DallasTemperature.h"
#include <EEPROM.h>


// линия данных на пине 4
#define ONE_WIRE_PIN 4
// реле обогревателя управляется с пина 2
#define HEATING_PIN 2
// яркость освещения пин 3
#define LIGHT_PIN 3

// создаем экземпляр (oneWire) объекта
OneWire oneWire(ONE_WIRE_PIN);

// передаем ссылку на объект oneWire библиотеке DallasTemperature
DallasTemperature sensors(&oneWire);

// переменная для считывания адресов датчиков
DeviceAddress SensorAdress;

// количество датчиков с коэффицентами калибровки
const int cntCalSens=2;

// массив структуры линейных калибровочных коэффициентов для датчиков
// при добавлении нового элемента нужно соответственно увеличить размерность
struct calibreSensors {
  DeviceAddress sensAddr;       // адрес датчика
  float calibreVal;             // калибровочный коэфф. датчика
} sensAddrCalibre[cntCalSens];  // создаем экземпляр в виде массива
// экземпляр структуры для работы с массивом
calibreSensors curSensAddrCalibre;



// структура для передачи на панель управления
struct forSend {
  byte intPartTemp1;          // целая часть температуры датчика 1
  byte fractPartTemp1;        // дробная часть температуры датчика 1 (2 знака)
  byte intPartTemp2;          // целая часть температуры датчика 2
  byte fractPartTemp2;        // дробная часть температуры датчика 2 (2 знака)
  byte intPartTempSet;        // целая часть температуры для регулировки
  byte fractPartTempSet;      // дробная часть температуры для регулировки (2 знака)
  byte packFlags;             // упакованные флаги состояний
  byte CRC8_sum;              // контрольная сумма
} packForSend;                // создаем экземпляр

// структура для принятых от панели данных
struct receivedFrom {
  byte intPartTempSet;        // целая часть температуры для регулировки
  byte fractPartTempSet;      // дробная часть температуры для регулировки (2 знака)
  byte packFlags;             // упакованные флаги состояний (0-включить свет, 1-установить новое значение рег)
  byte CRC8_sum;              // контрольная сумма   
} packReceivedFrom;           // создаем экземпляр



float EEMEM needTemp_addr;    // адрес сохраненной температуры в EEPROM

int deviceTmpCount = 0;       // количество подключенных температурных датчиков
float tempC[2];               // массив для значений температуры (максимум 2 датчика)
int periodMeasure = 1000;     // задержка между запросами температуры (миллисекунды)
int periodSend = 1000;        // задержка между отправкой данных (миллисекунды)
uint32_t timerForTemp;        // таймер для задержки запросов (uint32_t = unsigned long)
uint32_t timerForSend;        // таймер для отправки данных в COM порт


int i;
int j;
float tmpTemp;


// переменные для регулировки температуры
int alrmMinTmp = 10;          // минимально допустимая температура "достоверности"
int alrmMaxTmp = 50;          // максимально допустимая температура "достоверности"
int minTmp = 17;              // минимально допустимая температура "регулирования"
int maxTmp = 29;              // максимально допустимая температура "регулирования"
float avrCurTemp;             // усредненное значение температуры с нескольких датчиков
float needTemp = 25;          // установленная для удержания температура (считываем из EEPROM)
float needNewTemp;            // новое значение для регулирования
float hTemp = 0.25;           // гистерезис для регулировки температуры
bool needHeat;                // флаг включения\отключения обогревателя
bool lastStateHeat;           // состояние нагревателя с предыдущего скана
bool needSetTemp = false;     // флаг установки нового значения регулирования
bool needSwichLight;          // флаг нажатия кнопки включения освещения
bool lightON;                 // флаг состояния освещения
bool needRst;                 // флаг перезагрузки
float curLighting;            // значение яркости лампы
forSend lastPackForSend;      // состояние структуры для передачи с предыдущего скана


void setup() {
  Serial.begin(9600);
  sensors.begin();    // запускаем библиотеку DallasTemperature

  needHeat = false;   // при старте обогреватель отключен
  lastStateHeat = false;

  lightON = false;    // при старте свет выключен
  curLighting = 0;

  pinMode(HEATING_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  timerForTemp = millis();  // типа инициализируем таймер
  timerForSend = millis();

  needRst = false;





  // записываем калибровочные коэффициенты для известных датчиков
  // при добавлении нового датчика - нужно увеличить размерность экземпляра структуры выше
  sensAddrCalibre[0] = (calibreSensors) {{0x28, 0xAA, 0xCD, 0x04, 0x4F, 0x14, 0x01, 0xAA}, 0.41};
  sensAddrCalibre[1] = (calibreSensors) {{0x28, 0xE3, 0x5B, 0x79, 0x97, 0x10, 0x03, 0xEC}, 1.10};



  deviceTmpCount = sensors.getDeviceCount();    // запрос количества датчиков


  // проверяем, есть ли в EEPROM сохраненное, достоверное значение регулирования
  EEPROM.get((int)&needTemp_addr, tmpTemp);
  if ((tmpTemp >= minTmp) && (tmpTemp <= maxTmp)){
    needTemp = tmpTemp;
  }
  // записываем новое значение рег-ия в структуру для отправки (оно под индексом 10)
  packForSend = insertToSend(10, needTemp, packForSend);
}



// функция перезагрузки
void(* resetFunc) (void) = 0;//объявляем функцию reset с адресом 0



void loop() {

sensors.requestTemperatures();  // запрос температуры




uint8_t sa[8];
    
tempC[0] = sensors.getTempCByIndex(0);

        // проверяем адрес датчика на наличие калибровочного коэффициента для него
        sensors.getAddress (SensorAdress, 0);
        for (int k=0; k < cntCalSens; k++){
          curSensAddrCalibre = sensAddrCalibre[k];
          if (SensorAdress == curSensAddrCalibre.sensAddr) {
            tempC[0] = tempC[0] + curSensAddrCalibre.calibreVal;
            Serial.println("find 0");
          }
        }

sensors.getAddress (sa, 0);
Serial.print ("index 0:  address: ");
for (i = 0; i < 8; i++) {
Serial.print (sa[i], HEX); 
Serial.print (" ");
}
Serial.print (",    temp = ");
Serial.println (tempC[0]);






tempC[1] = sensors.getTempCByIndex(1);

        // проверяем адрес датчика на наличие калибровочного коэффициента для него
        sensors.getAddress (SensorAdress, 1);
        for (int k=0; k < cntCalSens; k++){
          curSensAddrCalibre = sensAddrCalibre[k];
          if (SensorAdress == curSensAddrCalibre.sensAddr) {
            tempC[1] = tempC[1] + curSensAddrCalibre.calibreVal;
            Serial.println("find 1");
          }
        }
        
sensors.getAddress (sa, 1);
Serial.print ("index 1:  address: ");
for (i = 0; i < 8; i++) {
Serial.print (sa[i], HEX);
Serial.print (" ");
}
Serial.print (",    temp = ");
Serial.println (tempC[1]);


Serial.println ("---------------------------------------------------------------");
delay (3000);
}








//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// сохранение нового значения регулирования в EEPROM
bool setNewTemp (int addrEEM, float paramTmp){
  float tempTemp;
  EEPROM.put(addrEEM, paramTmp);
  EEPROM.get(addrEEM, tempTemp);
  return paramTmp == tempTemp;  
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// сохраняем значение температуры в структуру для передачи
forSend insertToSend(int numParam, float paramTemp, forSend pointStruct){
  if (paramTemp > 0) {
    switch (numParam){
      case 1:
        pointStruct.intPartTemp1 = paramTemp;    //дробная часть отбросится сама
        pointStruct.fractPartTemp1 = (paramTemp - pointStruct.intPartTemp1) * 100.0;
        break;
      case 2:
        pointStruct.intPartTemp2 = paramTemp;    //дробная часть отбросится сама
        pointStruct.fractPartTemp2 = (paramTemp - pointStruct.intPartTemp2) * 100.0;
        break;
      case 10:
        pointStruct.intPartTempSet = paramTemp;    //дробная часть отбросится сама
        pointStruct.fractPartTempSet = (paramTemp - pointStruct.intPartTempSet) * 100.0;
        break;
    }
  }
  else {
    // если датчик неисправен - значение температуры минус 127 градусов
    switch (numParam){
      case 1:
        pointStruct.intPartTemp1 = 0;
        pointStruct.fractPartTemp1 = 0;
        break;
      case 2:
        pointStruct.intPartTemp2 = 0;
        pointStruct.fractPartTemp2 = 0;
        break;
      case 10:
        pointStruct.intPartTempSet = 0;
        pointStruct.fractPartTempSet = 0;
        break;
    }
  }
  return pointStruct;
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// вычисление CRC и отправка данных в COM порт
void sendMyPack (forSend newPackForSend){
  newPackForSend.CRC8_sum = 0;
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.intPartTemp1 + newPackForSend.fractPartTemp1);
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.intPartTemp2 + newPackForSend.fractPartTemp2);
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.intPartTempSet + newPackForSend.fractPartTempSet);
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.packFlags);

  Serial.write((uint8_t *) & newPackForSend, sizeof(newPackForSend));
  }
