TaskHandle_t thp[5];//マルチスレッドのタスクハンドル格納用

SemaphoreHandle_t xMutex = NULL;
int sharedResource = 0;

#include <Servo.h>//使用するライブラリをインクルード
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <CheapStepper.h>

Servo Servo1;
Servo Servo2;

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

#define IN1_1 32
#define IN1_2 33
#define IN1_3 25
#define IN1_4 26

#define IN2_1 27
#define IN2_2 23
#define IN2_3 19
#define IN2_4 18

#define Servo1_PIN 13
#define Servo2_PIN 14

#define Sensor1_PIN 4 
#define Sensor2_PIN 16
#define Sensor3_PIN 17

#define TX 35
#define RX 34

#define A 2

CheapStepper stepper1(IN1_1,IN1_2,IN1_3,IN1_4); // IN1,IN2,IN3,IN4
CheapStepper stepper2(IN2_1,IN2_2,IN2_3,IN2_4); // IN1,IN2,IN3,IN4

boolean clockwise = true;
boolean error = 1;

static const uint8_t packet_begin[3] = { 0xFF, 0xD8, 0xEA };

int i=90;
int i2=90;
int x=225;
int y=225;
int CALC_1;//計算結果
int CALC_2;
int CALC_3_X;
int CALC_3_Y;
int dist_1;//距離
int dist_2;
int dist_3;

char map1[200][200];

float root;//√2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BMP_WIDTH    128
#define BMP_HEIGHT   32

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX, TX);  //RX,TX
  
  delay(50);  // Serial Init Wait

  stepper1.setRpm(10);
  stepper2.setRpm(10);

  root  = sqrt(A);//平方根を計算
  
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

    // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds
  
  Wire.begin();//I2C通信ライブラリ(Wire)の初期化

  pinMode(Sensor1_PIN, OUTPUT); //XSHUIを5に接続
  pinMode(Sensor2_PIN, OUTPUT);//XSHUIを4に接続
  pinMode(Sensor3_PIN, OUTPUT);//XSHUIを4に接続
  digitalWrite(Sensor1_PIN, LOW);//ここからセンサのi2cアドレスを書き換える処理
  digitalWrite(Sensor2_PIN, LOW);
  digitalWrite(Sensor3_PIN, LOW);

  delay(500);

  Servo1.attach(Servo1_PIN);//Servo
  Servo1.write(90);
  Servo2.attach(Servo2_PIN);
  Servo2.write(90);

  Serial.println(F("Program start"));
  
  pinMode(Sensor1_PIN, INPUT);
  delay(150);
  Serial.println("00");
  sensor1.init(true);//距離センサ(VL53L0X)の初期化

  Serial.println(F("01"));
  delay(100);
  sensor1.setAddress((uint8_t)22);
  Serial.println(F("02"));

  pinMode(Sensor2_PIN, INPUT);
  delay(150);
  sensor2.init(true);//距離センサ(VL53L0X)の初期化
  Serial.println(F("03"));
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println(F("04"));

  pinMode(Sensor3_PIN, INPUT);
  delay(150);
  sensor3.init(true);//距離センサ(VL53L0X)の初期化
  Serial.println(F("05"));
  delay(100);
  sensor3.setAddress((uint8_t)28);
  Serial.println(F("06"));

  Serial.println(F("addresses set"));
  sensor1.startContinuous(100);//連続測定を開始する、100⇒100ms間隔でサンプリングする、ここに0を指定した場合は可能な限り頻繁にサンプリングする
  sensor2.startContinuous(100);
  sensor3.startContinuous(100);

  xMutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(Core0a, "Core0a", 8102, NULL, 3, &thp[0], 0);//マルチタスク
  xTaskCreatePinnedToCore(Core0b, "Core0b", 8192, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(Core1c, "Core1c", 4096, NULL, 3, &thp[4], 1);

  if( xMutex != NULL ){
    xTaskCreatePinnedToCore(Core1a, "Core1a", 8192, NULL, 4, &thp[2], 1);
    xTaskCreatePinnedToCore(Core1b, "Core1b", 8192, NULL, 3, &thp[3], 1);
  }
  else{
    while(1){
      Serial.println("rtos mutex create error, stopped");
      delay(1000);
    }
  }
}

void loop() {
  if (Serial2.available()) {//Maix Bitから座標を受け取る
    uint8_t rx_buffer[10];
    int rx_size = Serial2.readBytes(rx_buffer, 10);    
    if (rx_size == 10) {
      if ((rx_buffer[0] == packet_begin[0]) && (rx_buffer[1] == packet_begin[1]) && (rx_buffer[2] == packet_begin[2])) {
        x=rx_buffer[5];
        y=rx_buffer[6];
        //Serial.print(x);
        //Serial.print(",");
        //Serial.println(y);
      }
    }
  }  
}

void Core0a(void *args){
  while(1){
    delay(1);

    while(x<=106&&i<=120&&x!=225){//停止座標、サーボリミット
      delay(1);
      i++;
      Servo1.write(i);
      //Serial.println(val);
      delay(50);
    }

    while(x>=118&&i>=60&&x!=225){//停止座標、サーボリミット
      delay(1);
      i--;
      Servo1.write(i);
      //Serial.println(val);
      delay(50);
    }
  }
}

void Core0b(void *args){
  while(1){
    delay(1);
    
    while(y<=106&&i2<=100&&y!=225){//停止座標、サーボリミット
      delay(1);
      i2++;
      Servo2.write(i2);
      delay(50);
    }

    while(y>=118&&i2>=80&&y!=225){//停止座標、サーボリミット
      delay(1);
      i2--;
      Servo2.write(i2);
      delay(50);
    }
  }
}

void Core1b(void *args){
  BaseType_t xStatus;
  //const TickType_t xTicksToWait = 3000UL; //
  xSemaphoreGive(xMutex);

  delay(3000);

  while(1){
    xStatus = xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xStatus == pdTRUE){
      display.clearDisplay();
      display.fillRect(0,0,128,32,WHITE);
      display.display();
      delay(1);
    }
    xSemaphoreGive(xMutex);
  }
}

void Core1a(void *args){
  
  BaseType_t xStatus;
  //const TickType_t xTicksToWait = delay1;
  xSemaphoreGive(xMutex);

  while(1){

    xStatus = xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xStatus == pdTRUE){
      Serial.println (F("I2C scanner. Scanning ..."));
      byte count = 0;

      for (byte i = 1; i < 120; i++)
      {
        Wire.beginTransmission (i);
        if (Wire.endTransmission () == 0)
        {
          Serial.print (F("Found address: "));
          Serial.print (i, DEC);
          Serial.print (F(" (0x"));
          Serial.print (i, HEX);
          Serial.println (F(")"));
          count++;
          delay (1); // maybe unneeded?
        } // end of good response
      } // end of for loop
    
      Serial.println(F("Done."));
      Serial.print(F("Found "));
      Serial.print (count, DEC);
      Serial.println(F(" device(s)."));

      int delay1 = 3500;
      error = 1;

      dist_1 = sensor1.readRangeContinuousMillimeters(); //センサーから距離[mm]
      Serial.print(dist_1);
      Serial.print(F("[mm]\n"));

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor1.timeoutOccurred()){
        Serial.print(F("time-out\n"));
      }

      dist_2 = sensor2.readRangeContinuousMillimeters(); //センサーから距離[mm]
      Serial.print(dist_2);
      Serial.print(F("[mm]\n"));

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor2.timeoutOccurred()){
        Serial.print(F("time-out\n"));
      }

      dist_3 = sensor3.readRangeContinuousMillimeters(); //センサーから距離[mm]
      Serial.print(dist_3);
      Serial.print(F("[mm]\n"));

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor3.timeoutOccurred()){
        Serial.print("タイムアウトが起きました\n");
      }
      
      if(dist_1 <= 8095 && dist_2 <= 8095 && dist_3 <= 8095){
      
        delay1 = 1000;
        error = 0;
      
        //float Decimal_1 = root / 2 * (dist_1 + 17.84) + 37.5;三平方の定理を使用してxy軸の長さを推定
        //float Decimal_2 = root / 2 * (dist_2 + 17.84) + 37.5;
    
        float rad = i * (3.14 / 180);//度をラジアンに変換

        float Decimal_3_X =  (dist_3  + 67.5) * cos(rad);//いろいろ計算
        float Decimal_3_Y =  (dist_3  + 67.5) * sin(rad);

        Serial.println(Decimal_3_X);
        Serial.println(i);

        CALC_1 = round(root / 2 * (dist_1 + 17.84) + 37.5);//いろいろ計算&小数点以下切り捨て
        CALC_2 = round(root / 2 * (dist_2 + 17.84) + 37.5);
        CALC_3_X = round(Decimal_3_X);
        CALC_3_Y = round(Decimal_3_Y);

        CALC_1 = CALC_1 / 10;//ミリをセンチに変換
        CALC_2 = CALC_2 / 10;
        CALC_3_X = CALC_3_X / 10;
        CALC_3_Y = CALC_3_Y / 10;

        int CALC_1_X = (CALC_1 * -1) + 100;//座標に変換
        int CALC_1_Y = CALC_1 + 100;

        Serial.print(F("CALC_1_X:"));
        Serial.println(CALC_1_X);
        Serial.print(F("CALC_1_Y:"));
        Serial.println(CALC_1_Y);
        Serial.print(F("CALC_2:"));
        Serial.println(CALC_2);
        Serial.print(F("CALC_3_X:"));
        Serial.println(CALC_3_X);
        Serial.print(F("CALC_3_Y:"));
        Serial.println(CALC_3_Y);

        if( CALC_1>=0 && CALC_1<=200 ){//配列に記録
          map1[CALC_1_X][CALC_1_Y] = 1;
          Serial.println(F("record complete"));
        }
    
        if( CALC_2>=0 && CALC_2<=200 ){
          map1[CALC_2][CALC_2] = 1;
          Serial.println(F("record complete"));
        }

        if( CALC_3_X>=0 && CALC_3_Y>=0 && CALC_3_X<=200 && CALC_3_Y<= 200 ){
          map1[CALC_3_X][CALC_3_Y] = 1;
          Serial.println(F("record complete"));
        }
      }
      xSemaphoreGive(xMutex);
      delay(delay1);
    }
  }
}

void Core1c(void *args){
  while(1){
    delay(2);
    if(error == 0){
      stepper1.run();
      stepper2.run();
      stepper1.newMoveDegrees(clockwise,360);
      stepper2.newMoveDegrees(clockwise,360);
    }
  }
}