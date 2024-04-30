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

// общее количество подключенных датчиков
const int cntSens = 2;

// переменная для считывания адресов датчиков
DeviceAddress SensorAdress;


// массив адресов текущих датчиков
DeviceAddress SensorsAdreses[cntSens];
// массив калибровочных коэфф. текущих датчиков
float SensorsCals[cntSens];

// количество датчиков с коэффицентами калибровки
const int cntCalSens = 3;

// массив структуры линейных калибровочных коэффициентов для датчиков
// при добавлении нового элемента нужно соответственно увеличить размерность
struct calibreSensors {
  DeviceAddress sensAddr;       // адрес датчика
  float calibreVal;             // калибровочный коэфф. датчика
} sensAddrCalibre[cntCalSens];  // создаем экземпляр в виде массива



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
float tempC[cntSens];         // массив для значений температуры
float tempCCal[cntSens];      // массив для калибровок значений температуры
int periodMeasure = 1000;     // задержка между запросами температуры (миллисекунды)
int periodSend = 1000;        // задержка между отправкой данных (миллисекунды)
uint32_t timerForTemp;        // таймер для задержки запросов (uint32_t = unsigned long)
uint32_t timerForSend;        // таймер для отправки данных в COM порт


int i;
int j;
int k;
float tmpTemp;


// переменные для регулировки температуры
int alrmMinTmp = 10;          // минимально допустимая температура "достоверности"
int alrmMaxTmp = 50;          // максимально допустимая температура "достоверности"
int minTmp = 17;              // минимально допустимая температура "регулирования"
int maxTmp = 29;              // максимально допустимая температура "регулирования"
float avrCurTemp;             // усредненное значение температуры с нескольких датчиков
float needTemp = 25;          // установленная для удержания температура (считываем из EEPROM)
float needNewTemp;            // новое значение для регулирования
float hTemp = 0.3;            // гистерезис для регулировки температуры
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
  sensAddrCalibre[2] = (calibreSensors) {{0x28, 0x55, 0xAB, 0x16, 0xA8, 0x01, 0x3C, 0xA3}, 0.00};


  deviceTmpCount = sensors.getDeviceCount();    // запрос количества датчиков
  if (deviceTmpCount > cntSens) {
    Serial.println("Devices is more then defined! Find " + String(deviceTmpCount) + " devices");
    deviceTmpCount = cntSens;
  }
  for (i=0; i < deviceTmpCount; i++) {
    // считываем адреса подключенных датчиков
    sensors.getAddress (SensorsAdreses[i], i);
    // обнуляем коэфф. калибровки
    SensorsCals[i] = 0.0;
  }



  //сравниваем адреса найденных датчиков с ранее определенными и посчитанными коэффициентами
  for (i=0; i < deviceTmpCount; i++){   // перебираем доступные датчики
    for (j=0; j < cntCalSens; j++){     // перебираем описаные датчики с коэффициентами
      bool eqAddr = true;         // взводим флаг для поиска соответствия
      for (k=0; k <8; k++){
        // адреса состоят из массива (8 шт) значений типа uint8_t, перебираем каждый элемент адреса
        if (SensorsAdreses[i][k] != sensAddrCalibre[j].sensAddr[k]){
          eqAddr = false;         // если найдено несоответствие в адресах, сбрасываем флаг
        }
      }
      // если флаг не сброшен - значит результат поиска успешный
      if (eqAddr) {
        SensorsCals[i] = sensAddrCalibre[j].calibreVal;         // запоминаем коэффициент калибровки
      }
    }
  }



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

//****************************************************************************
  // Получение данных от панели и разбор полученного
  if (Serial.available() >= (int) sizeof(packReceivedFrom)){
    Serial.readBytes((uint8_t *) & packReceivedFrom, sizeof(packReceivedFrom));
    // проверка CRC
    if (byte(packReceivedFrom.intPartTempSet + packReceivedFrom.fractPartTempSet + packReceivedFrom.packFlags) == packReceivedFrom.CRC8_sum){
      needNewTemp = packReceivedFrom.intPartTempSet;
      needNewTemp += ((float)packReceivedFrom.fractPartTempSet / 100.0);
      // 0-й бит включает\отключает свет
      needSwichLight = ((packReceivedFrom.packFlags & B00000001) > 0);
      // 1-й бит задает температуру для регулирования
      needSetTemp = ((packReceivedFrom.packFlags & B00000010) > 0);
      // 2-й бит вызывает перезагрузку
      needRst = ((packReceivedFrom.packFlags & B00000100) > 0);
      // если требуется установить температуру - проверяем вхождение в допустимые пределы
      if (needSetTemp){
        needSetTemp = (needNewTemp >= minTmp) && (needNewTemp <= maxTmp);
      }
    }
    // если получено больше данных чем предполагалось - вычитываем оставшиеся быйты для обнуления буфера
    if (Serial.available() > 0){
      Serial.readBytes((uint8_t *) & packReceivedFrom, Serial.available());
    }
  }




//****************************************************************************
  // обработка температурных датчиков и включение обогревателя 
  
  // millis сбрасывается через 50 дней
  if (millis() < timerForTemp) {
    timerForTemp = millis();  // перезапускаем таймер
  }
  
  
  // считываем температуру и обрабатываем её по таймеру
  if (millis() - timerForTemp >= periodMeasure) {
    lastStateHeat = needHeat;   // сохраняем текущее состояние нагревателя
    needHeat = false;           // по умолчанию обогреватель отключен

    // если выполняется включение/выключение света - ничего не делаем
    // иначе плавного включения не будет (кипятильник на это время будет отключен)
    if (!((curLighting < 255) && (curLighting > 0))) {

      sensors.requestTemperatures();  // запрос температуры
    
      // заполняем массив температур
      avrCurTemp = 0;
      j = 0;
      for (i=0; i < deviceTmpCount; i++) {
        // если датчик отвалился - значение будет минус 127
        tempC[i] = sensors.getTempC(SensorsAdreses[i]);  // считываем температуру по адресу датчика
        tempC[i] = tempC[i] + SensorsCals[i];            // прибавляем калибровочный коэффициент для датчика

        
        // проверяем достоверность считанных значений
        // если датчик ушел в зашкал - не считаем его
        if ((tempC[i] > alrmMinTmp) && (tempC[i] < alrmMaxTmp)) {
          avrCurTemp += tempC[i];
          j++;
        }
        // записываем в структуру для передачи на панель значения температур
        packForSend = insertToSend(i + 1, tempC[i], packForSend);
        //Serial.println(i);
      }         

Serial.println ("");
Serial.println ("==============================================================================");
Serial.println ("Темп1. = " + String(tempC[0]) + " C");
Serial.println ("Темп2. = " + String(tempC[1]) + " C");



      
      // если среднее значение больше нуля - значит живой минимум 1 датчик
      // и значения температуры в рабочем диапазоне
      if (avrCurTemp > 0) {
        avrCurTemp /= j;  // усредняем сумму температур
    
        // обогреватель был включен и темп поднялась выше заданной
        if ((lastStateHeat) && (avrCurTemp >= (needTemp))) {
          needHeat = false;
        }
        // обогреватель был отключен и темп упала ниже заданной минус гистерезис
        if ((!lastStateHeat) && (avrCurTemp <= (needTemp - hTemp))) {
          needHeat = true;
        }
      }
      
Serial.println ("Усредненная = " + String(avrCurTemp) + " C");
Serial.println ("needHeat = " + String(needHeat));
Serial.println ("hTemp = " + String(hTemp));  


   
    }
    timerForTemp = millis();  // перезапускаем таймер
    // если все датчики в зашкале, то установленный в начале обработчика таймера
    // останется needHeat = false
  }




//****************************************************************************
  // нужно установить новое значение для регулирования
  if (needSetTemp){
    //если новое значение записалось в EEPROM удачно - принимаем его для регулирования
    if (setNewTemp((int)&needTemp_addr, needNewTemp))   {
      needTemp = needNewTemp;
      // записываем новое значение рег-ия в структуру для отправки (оно под индексом 10)
      packForSend = insertToSend(10, needTemp, packForSend);
    }
    needSetTemp = false;
  }


//****************************************************************************
  // нужно включить/выключить свет
  if (needSwichLight) {
    lightON = !lightON;
    needSwichLight = false;
  }
  
  if (lightON && (curLighting < 255)) {
    curLighting = curLighting + ((curLighting+1) / 1000.0);
    if (curLighting >= 254) {
      curLighting=255;
    }
  }
  
  if ((!lightON) && (curLighting > 0)) {
    curLighting = curLighting - (3.0 / curLighting);
    if (curLighting <= 2) {
      curLighting=0;
    }
  }
  
  analogWrite(LIGHT_PIN, curLighting);



//****************************************************************************
  // передаем данные в COM порт по таймеру
  // millis сбрасывается через 50 дней
  if (millis() < timerForSend) {
    timerForSend = millis();  // перезапускаем таймер
  }
  if (millis() - timerForSend >= periodSend) {
    //sendMyPack (packForSend);
    timerForSend = millis();
  }
   



//****************************************************************************
  // включаем\отключаем обогреватель
  digitalWrite(HEATING_PIN, needHeat);

//****************************************************************************
  // сохраняем текущее состояние структуры для отправки (думаю пригодится потом)
  lastPackForSend = packForSend;


//****************************************************************************
  // в конце loop проверяем - нужно ли перезагрузиться
  if (needRst) {
    needRst = false;
    resetFunc(); //вызываем reset
  }
  
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
