// подключение библиотеки энкодера (кавычки значат что либа лежит рядом со скетчем) (https://alexgyver.ru/encoder/)
#include "GyverEncoder.h"
// подклчение самописной библиотеки с символами для индикаторов
#include "LibCharIndicator.h"
// подключение библиотеки прерываний по таймеру
#include "GyverTimers.h"


// кнопка включения света на пине 7
#define LIGHT_BTN_PIN 7
// кнопка  пине 8
#define RST_BTN_PIN 8
// кнопка энкодера на пине 11
#define ENC_BTN_PIN 11
// первый пин энкодера (данные)
#define ENC_PIN_DT 10
// второй пин энкодера (тактовый)
#define ENC_PIN_CLK 12
// пин подключен к ST_CP входу 74HC595 (защелка)
#define LATCH_PIN  3
// пин подключен к SH_CP входу 74HC595 (тактовый)
#define CLOCK_PIN  4
// пин подключен к DS входу 74HC595 (данные)
#define DATA_PIN  2
// пин первого разряда индикаторов
#define INDIC_1_PIN 5
// пин второго разряда индикаторов
#define INDIC_2_PIN 6
// пин третьего разряда индикаторов
#define INDIC_3_PIN 9
// пин подключения датчика света
#define LIGHT_PIN A0



// структура для принятых от сервера данных
struct receivedFrom {
  byte intPartTemp1;          // целая часть температуры датчика 1
  byte fractPartTemp1;        // дробная часть температуры датчика 1 (2 знака)
  byte intPartTemp2;          // целая часть температуры датчика 2
  byte fractPartTemp2;        // дробная часть температуры датчика 2 (2 знака)
  byte intPartTempSet;        // целая часть температуры для регулировки
  byte fractPartTempSet;      // дробная часть температуры для регулировки (2 знака)
  byte packFlags;             // упакованные флаги состояний 
  byte CRC8_sum;              // контрольная сумма
} packReceivedFrom;           // создаем экземпляр

// структура для передачи серверу
struct forSend {
  byte intPartTempSet;        // целая часть температуры для регулировки
  byte fractPartTempSet;      // дробная часть температуры для регулировки (2 знака)
  byte packFlags;             // упакованные флаги состояний  (0-включить свет, 1-установить новое значение рег)
  byte CRC8_sum;              // контрольная сумма   
} packForSend;                // создаем экземпляр

// структура для хранения состояния кнопки
struct btnStatusStruct {
  byte pinBtn;                // номер пина, куда подключена кнопка
  uint32_t timeBtnNoise;      // время для расчета подавления дребезга
  bool btnCurState;           // текущий статус нажатия
  bool btnBackState;          // состояние нажатия с прошлого скана
  bool typeBtn;               // тип кнопки, High при нажатии это True
};

// пины транзисторов индикаторов
byte indicPin[] = {INDIC_1_PIN, INDIC_2_PIN, INDIC_3_PIN};

uint32_t fixTimeRcv;          // хранение времени последней посылки (uint32_t = unsigned long)
uint32_t setTimer;            // таймер ожидания применения уставки
uint32_t flashTimer;          // таймер моргания индикаторами
uint32_t dispTimer;           // таймер отображения переходных процессов
uint32_t tuningTimer;         // таймер ожидания ввида новой уставки


int minTmp = 17;              // минимально допустимая температура "регулирования"
int maxTmp = 29;              // максимально допустимая температура "регулирования"
int timeWaitSet = 10000;      // время ожидания применения уставки
int timeFlash =  200;         // время мигания индикаторами
int dispTime = 2000;          // время отображения переходных процессов
int tuningTime = 10000;       // время ожидания ввода новой уставки
int timeRcv = 10000;          // контрольное время получения посылки


bool CRC_OK;                  // статус проверки CRC полученных данных
bool needSend;                // флаг требования передачи данных
bool needRefreshInd;          // флаг необходимости обновления цифр на индикаторах
bool showError;               // флаг отображения ошибки
bool flash;                   // флаг мигания индикаторов
bool needRst;                 // флаг перезагрузки

// статус режима ввода уставки регулирования  
byte statusTuningParam;       //0-отображ/ 1-ввод/ 2-применить/ 3-применилась/ 10-ошибка

byte i, j;                    // глобальные счетчики
byte countDigit;              // счетчик отображаемого разряда индикатора
byte bright;                  // яркость свечения индикаторов
byte brHyst;                  // гистерезис для яркости
byte curSetPoint[2];          // здесь храним значение уставки, полученное с сервера


byte indChar[2][3];            // массив для вывода символов на индикаторы


btnStatusStruct lightBtn;     // экземпляр для кнопки включения света
btnStatusStruct rstBtn;       // экземпляр для кнопки Reset
btnStatusStruct encBtn;       // экземпляр для кнопки энкодера

// пин clk, пин dt, пин sw
Encoder enc(ENC_PIN_CLK, ENC_PIN_DT, ENC_BTN_PIN); // экземпляр объекта энкодера




void setup() {
  Serial.begin(9600);

  // Пины D9 и D10 - ШИМ 976 Гц
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00001011;  // x64 fast pwm

  pinMode(LIGHT_BTN_PIN, INPUT);
  pinMode(RST_BTN_PIN, INPUT);
  pinMode(ENC_BTN_PIN, INPUT);
  pinMode(ENC_PIN_CLK, INPUT);
  pinMode(ENC_PIN_DT, INPUT);  
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(INDIC_1_PIN, OUTPUT);
  pinMode(INDIC_2_PIN, OUTPUT);
  pinMode(INDIC_3_PIN, OUTPUT);
  pinMode(LIGHT_PIN, INPUT);

  CRC_OK = false;   // пока не будет получена посылка с правильным CRC флаг сброшен

  i = 0;
  bright = 200;           // яркость индикаторов по умолчанию
  brHyst = 30;            // гистерезис для переключения яркости свечения индикаторов
  countDigit = 0;         // счетчик для переключения транзисторов по разрядам индикаторов
  statusTuningParam = 0;  // при старте просто отображаем температуру

  // инициализируем структуры для кнопок
  lightBtn.pinBtn = LIGHT_BTN_PIN;
  lightBtn.timeBtnNoise = millis();
  lightBtn.typeBtn = true;
  rstBtn.pinBtn = RST_BTN_PIN;
  rstBtn.timeBtnNoise = millis();
  rstBtn.typeBtn = true;
  encBtn.pinBtn = ENC_BTN_PIN;
  encBtn.timeBtnNoise = millis();
  encBtn.typeBtn = false;

  needRst = false;

  enc.setPinMode(HIGH_PULL);    // тип подключения энкодера, подтяжка HIGH_PULL (внутренняя) или LOW_PULL (внешняя на GND)
  enc.setType(TYPE1);           // TYPE1 / TYPE2 - тип энкодера TYPE1 одношаговый, TYPE2 двухшаговый.
  enc.setTickMode(AUTO);      // MANUAL / AUTO - ручной или автоматический опрос энкодера функцией tick()

  fixTimeRcv = millis(); //чтобы при старте не выводились ошибки

  // отключаем лампочку 13-го пина
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);




  // настройка прерываний
  Timer2.setPeriod(5000);     // в микросекундах
  Timer2.enableISR();
  
}


// функция перезагрузки
void(* resetFunc) (void) = 0;//объявляем функцию reset с адресом 0


void loop() {
//****************************************************************************
  // Получение данных от сервера и проверка чек-суммы
  if ( Serial.available() >= (int) sizeof(packReceivedFrom)){
    Serial.readBytes((uint8_t *) & packReceivedFrom, sizeof(packReceivedFrom));
    // проверка CRC
    CRC_OK = (byte(packReceivedFrom.intPartTemp1 + packReceivedFrom.fractPartTemp1 + packReceivedFrom.intPartTemp2 + packReceivedFrom.fractPartTemp2 + packReceivedFrom.intPartTempSet + packReceivedFrom.fractPartTempSet + packReceivedFrom.packFlags) == packReceivedFrom.CRC8_sum);
    fixTimeRcv = millis();
    needRefreshInd = true;
  }

  // если долго нет связи или ошибка чек-суммы - переходим в состояние ОШИБКА
  if ((((millis () - fixTimeRcv) > timeRcv)) || (!CRC_OK) )  {
    statusTuningParam = 10;
    needRefreshInd = true;
  }






//****************************************************************************
  // проверяем все кнопки, предварительно сохраняем состояние текущего нажатия
  lightBtn.btnBackState = lightBtn.btnCurState;
  lightBtn = makeBtnState(lightBtn);
  // кнопка Reset
  rstBtn.btnBackState = rstBtn.btnCurState;
  rstBtn = makeBtnState(rstBtn);
  // кнопка энкодера обрабатывается библиотекой энкодера
  //encBtn.btnBackState = encBtn.btnCurState;
  //encBtn = makeBtnState(encBtn);

  // нажатие кнопки включения света - нужно передать серверу
  if (lightBtn.btnBackState < lightBtn.btnCurState) {
    // упаковываем флаг включения в структуру (0-й бит)
    packForSend.packFlags =  packForSend.packFlags | B00000001;
    needSend = true;
  }

  // нажатие кнопки Reset - нужно передать серверу
  if (rstBtn.btnBackState < rstBtn.btnCurState) {
    // упаковываем флаг включения в структуру (2-й бит с нуля)
    packForSend.packFlags =  packForSend.packFlags | B00000100;
    needSend = true;
    needRst = true;
  }



  

  // нажатие кнопки энкодера переключает режим ввода уставки регулирования
  if (enc.isClick()) {
    switch (statusTuningParam) {
      case 0:   // штатный режим отображения
        // фиксируем текущее значение уставки регулирования
        if (CRC_OK) {
          curSetPoint[0] = packReceivedFrom.intPartTempSet;
          curSetPoint[1] = packReceivedFrom.fractPartTempSet;
          // переводим в режим установки
          statusTuningParam = 1;
          tuningTimer = millis();
          needRefreshInd = true;
        }
        else showError = true;
        break;
      case 1:   // режим ввода уставки
        // при нажатии переводим в режим применения
        statusTuningParam = 2;
        setTimer = millis();
        needRefreshInd = true;
        // перекладываем новую уставку для передачи серверу
        packForSend.intPartTempSet = curSetPoint[0];
        packForSend.fractPartTempSet = curSetPoint[1]; 
        // упаковываем флаг установки в структуру (1-й бит с нуля)
        packForSend.packFlags =  packForSend.packFlags | B00000010;
        needSend = true;
        break;
      //default:
        // если придумаю что делать по умолчанию - буду писать сюда
    }
  }







//****************************************************************************
  // вывод температуры на индикаторы
  if (needRefreshInd && (statusTuningParam == 0) && CRC_OK) {
    setTempIndicator(0, packReceivedFrom.intPartTemp1, packReceivedFrom.fractPartTemp1 );
    setTempIndicator(1, packReceivedFrom.intPartTemp2, packReceivedFrom.fractPartTemp2 );
    needRefreshInd = false;
  }

  // вывод текущей уставки на индикаторы
  if (needRefreshInd && (statusTuningParam == 1) ) {
    // в режиме настройки выводим на верхний индикатор SET в нижнем изменяем значение энкодером
    setViewIndicator(0, 1); 
    setTempIndicator(1, curSetPoint[0], curSetPoint[1] );
    needRefreshInd = false;
  }

  // если долго бездействовать - сбрасываем в текущий режим
  if ((statusTuningParam == 1) && ((millis() - tuningTimer) > tuningTime)) {
    statusTuningParam = 0;
    needRefreshInd = true;
  }

  // применение уставки
  if (needRefreshInd && (statusTuningParam == 2) ) {
    // в режиме применения выводим SET на оба индикатора
    setViewIndicator(0, 1); 
    setViewIndicator(1, 1);
    needRefreshInd = false;
  }
  // новая уставка должна примениться в течение установленного времени - иначе ошибка
  if ( (statusTuningParam == 2) ) {
    if (CRC_OK && (curSetPoint[0] == packReceivedFrom.intPartTempSet) ) {
      // сравниваем округленную дробную часть
      if (roumdFract(curSetPoint[1]) == roumdFract(packReceivedFrom.fractPartTempSet)){
        // уставка применилась успешно!
        statusTuningParam = 3;
        flashTimer = millis();
        dispTimer = millis();
        needRefreshInd = true;        
      }
    }
    if (millis() - setTimer > timeWaitSet) {
      // уставка не применилась
      statusTuningParam = 10;
      dispTimer = millis();
      needRefreshInd = true;
    }
  }

  // уставка применилась
  if (statusTuningParam == 3) {
    if (millis() - flashTimer >= timeFlash){
      flashTimer = millis();
      flash = !flash;
      needRefreshInd = true;
    }
    if ( flash && needRefreshInd ) {
      // здесь выводим примененную мигающую уставку на оба индикатора
      setTempIndicator(0, packReceivedFrom.intPartTempSet,  packReceivedFrom.fractPartTempSet );
      setTempIndicator(1, packReceivedFrom.intPartTempSet,  packReceivedFrom.fractPartTempSet );
      needRefreshInd = false;  
    }
    if ( !flash && needRefreshInd ) {
      // здесь "выключаем"
      setViewIndicator(0, 0); 
      setViewIndicator(1, 0);
      needRefreshInd = false;  
    }
    if ( millis() - dispTimer > dispTime ) {
      // всё ок, переходим в режим отображения
      statusTuningParam = 0;
      needRefreshInd = true;
    }
  }

  // отображение ERROR
  if (statusTuningParam == 10) {
    if (millis() - flashTimer >= timeFlash){
      flashTimer = millis();
      flash = !flash;
      needRefreshInd = true;
    }
    if ( flash && needRefreshInd ) {
      // здесь выводим теткс ERROR
      setViewIndicator(0, 10); 
      setViewIndicator(1, 10);
      needRefreshInd = false;  
    }
    if ( !flash && needRefreshInd ) {
      // здесь "выключаем"
      setViewIndicator(0, 0); 
      setViewIndicator(1, 0);
      needRefreshInd = false;  
    }
    if ( CRC_OK && millis() - dispTimer > dispTime ) {
      // всё ок, переходим в режим отображения
      statusTuningParam = 0;
      needRefreshInd = true;
    }
  }









//****************************************************************************
  // изменяем значение текущей уставки - энкодером (если мы в режиме настройки)
  if (enc.isLeft() && (statusTuningParam == 1) && (curSetPoint[0] >= minTmp)) {
    if ((curSetPoint[1] < 5) && (curSetPoint[0] > minTmp)) {
      curSetPoint[1] = 95;
      curSetPoint[0]--;
    }
    else if ((curSetPoint[1] < 5) && (curSetPoint[0] = minTmp)) {
      curSetPoint[1] = 0;
    }
    else curSetPoint[1] -= 5;
    needRefreshInd = true;
  }

  if (enc.isRight() && (statusTuningParam == 1) && (curSetPoint[0] < maxTmp)) {
    curSetPoint[1] += 10;
    if (curSetPoint[1] > 100) {
      curSetPoint[1] = 0;
      curSetPoint[0]++;
    }
    needRefreshInd = true;
  }

  // сбрасываем таймер бездействия
  if (enc.isTurn()) tuningTimer = millis();






//****************************************************************************
  // изменение яркости свечения от датчика
  if ((analogRead(LIGHT_PIN) < 800 ) && (analogRead(LIGHT_PIN) > (400 + brHyst) ) ) {
    bright = 100;
  }
  else if ((analogRead(LIGHT_PIN) < 400) && (analogRead(LIGHT_PIN) > (200 + brHyst) ) ){
    bright = 10;
  }
  else if (analogRead(LIGHT_PIN) < 200 ){
    bright = 1;
  }
  else if (analogRead(LIGHT_PIN) > (800 + brHyst)){
    bright = 200;
  }






//****************************************************************************
  // передача данных серверу
  if (needSend) {
    sendMyPack (packForSend);
    needSend = false;
    // некоторые флаги нужно сбросить после отправки
    packForSend.packFlags =  packForSend.packFlags & B11111110; // вкл. свет
    packForSend.packFlags =  packForSend.packFlags & B11111101; // новая уставка
  }

  // вывод инфы на индикаторы
  // showIndicators();





//****************************************************************************
  // в конце loop проверяем - нужно ли перезагрузиться
  if (needRst) {
    needRst = false;
    resetFunc(); //вызываем reset
  }
}












//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// обработка нажатий кнопок (при нажатии будет HIGH)
btnStatusStruct makeBtnState (btnStatusStruct thisBtn) {
  uint32_t curTime = millis();
  
  //каждые 50 дней millis сбрасывается на 0
  if (thisBtn.timeBtnNoise > curTime) {
    thisBtn.timeBtnNoise = curTime;
  }

  // убираем дребезг менее 50 мсек, фиксируем нажатие
  if (digitalRead(thisBtn.pinBtn) == thisBtn.typeBtn && !thisBtn.btnCurState && (curTime - thisBtn.timeBtnNoise) > 50) {
    thisBtn.btnCurState = true;
    thisBtn.timeBtnNoise = curTime;
  }
  
  // убираем дребезг менее 50 мсек, фиксируем отпускание
  if (digitalRead(thisBtn.pinBtn) == (!thisBtn.typeBtn) && thisBtn.btnCurState && (curTime - thisBtn.timeBtnNoise) > 50) {
    thisBtn.btnCurState = false;
    thisBtn.timeBtnNoise = curTime;
  }
  return thisBtn;
}




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// вычисление CRC и отправка данных в COM порт
void sendMyPack (forSend newPackForSend){
  newPackForSend.CRC8_sum = 0;
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.intPartTempSet + newPackForSend.fractPartTempSet);
  newPackForSend.CRC8_sum = byte(newPackForSend.CRC8_sum + newPackForSend.packFlags);

  Serial.write((uint8_t *) & newPackForSend, sizeof(newPackForSend));
  }
 




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// установка температуры для индикаторов
void setTempIndicator (byte indNum, byte intParamInd, byte fractParamInd) {
  byte tmpNum[3];

  if (intParamInd < 100) {
    tmpNum[0] = intParamInd / 10;
    tmpNum[1] = intParamInd % 10;
  }

  if (fractParamInd < 100) {
    tmpNum[2] = fractParamInd / 10;
    if ((fractParamInd % 10) > 5) tmpNum[2]++;
  }
  else tmpNum[2] = 9;

  for (int k = 0; k<3; k++) {
    indChar[indNum][k] = indCharLib[tmpNum[k]];
    if (k == 1) indChar[indNum][k] = indChar[indNum][k] | indCharLib[13]; // точка после второго знака
  }
}
 





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// округление до десятков
byte roumdFract (byte someValue) {
    byte tmp;
    tmp = someValue / 10;
    if ((someValue % 10) > 5) tmp++;
    tmp *= 10;
    return tmp;
}





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// установка прочей информации на индикаторах
void setViewIndicator (byte indNum, byte paramInd) {
  switch (paramInd) {
    case 0:     // 0 - гасим индикаторы
      indChar[indNum][0] = 0;
      indChar[indNum][1] = 0;
      indChar[indNum][2] = 0;
      break;    
    case 1:     // 1 - отображение надписи SET
      indChar[indNum][0] = indCharLib[10];
      indChar[indNum][1] = indCharLib[11];
      indChar[indNum][2] = indCharLib[12];
      break;

    case 2:     //

      break;

    case 10:    // 10 - отображение надписи ERROR
      indChar[indNum][0] = indCharLib[11];
      indChar[indNum][1] = indCharLib[14];
      indChar[indNum][2] = indCharLib[14];
      break;  
  }
}

 




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// вывод информации на индикаторы
void showIndicators () {
// при вызове функции прерыванием - показания нестабильны


  // сначала сбрасываем сдвиговые регистры, чтобы небыло шлейфов от предыдущих значений
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, B00000000); 
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, B00000000); 
  digitalWrite(LATCH_PIN, HIGH);
/*
  // подсвечиваем соответствующий символ индикаторов
  for (int k = 0; k<3; k++) {
    analogWrite(indicPin[k],  ((countDigit == k) * bright));
  }
*/
  // так должно быть быстрее
  analogWrite(INDIC_1_PIN,  ((countDigit == 0) * bright));
  analogWrite(INDIC_2_PIN,  ((countDigit == 1) * bright));
  analogWrite(INDIC_3_PIN,  ((countDigit == 2) * bright));


  // устанавливаем синхронизацию "защелки" на LOW
  digitalWrite(LATCH_PIN, LOW);
  // передаем код символа для вывода на второй индикатор
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, indChar[1][countDigit]); 
  // передаем код символа для вывода на первый индикатор
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, indChar[0][countDigit]);
  //"защелкиваем" регистр, тем самым устанавливая значения на выходах
  digitalWrite(LATCH_PIN, HIGH);

  // в следующем вызове работаем со следующей цифрой
  countDigit++;
  if (countDigit > 2) countDigit = 0;
}





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// сработка прерывания по таймеру
ISR(TIMER2_A) {
  // выводим информацию на индикаторы
  showIndicators(); 
}
