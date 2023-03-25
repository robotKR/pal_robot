TaskHandle_t thp[6];//マルチスレッドのタスクハンドル格納用
QueueHandle_t xQueue_1;
QueueHandle_t xQueue_2;

SemaphoreHandle_t xMutex = NULL;
int sharedResource = 0;

#include <Servo.h>//使用するライブラリをインクルード
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>

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

#define n 2

static const uint8_t packet_begin[3] = { 0xFF, 0xD8, 0xEA };

const float pi = 3.14;

unsigned long time_1;

int i=90;
int i2=90;
int x_1=225;
int y_1=225;
int CALC_1;//計算結果
int CALC_2;
int CALC_3_X;
int CALC_3_Y;
int dist_1;//距離
int dist_2;
int dist_3;

int Stepper_1= 1;
int Stepper_2= 1;

int speed_1 = 1;
int speed_2 = 1;

int count_1;
int count_2;

byte map1[200][200];

float root;//√2

float X;
float Y;

int Order = 1;

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

  WiFi.begin(ssid, password);  // WiFiへの接続を開始

  while (WiFi.status() != WL_CONNECTED) {  // WiFiに接続するまで待機
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  
  delay(50);  // Serial Init Wait

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

  /*for (int i = 0; i < 200; i++) {
    delay(1);
    for (int j = 0; j < 200; j++) {
      delay(1);
      map1[i][j] = 0;
    }
  }*/

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
  xQueue_1 = xQueueCreate( 10, 16 );
  xQueue_2 = xQueueCreate( 10, 16 );

  pinMode(IN1_1,OUTPUT);
  pinMode(IN1_2,OUTPUT);
  pinMode(IN1_3,OUTPUT);
  pinMode(IN1_4,OUTPUT);

  pinMode(IN2_1,OUTPUT);
  pinMode(IN2_2,OUTPUT);
  pinMode(IN2_3,OUTPUT);
  pinMode(IN2_4,OUTPUT);
  
  xTaskCreatePinnedToCore(following_X, "following_X", 4096, NULL, 3, &thp[0], 0);//マルチタスク
  xTaskCreatePinnedToCore(following_Y, "following_Y", 4096, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(StepMotor, "StepMotor", 4096, NULL, 3, &thp[2], 1);
  xTaskCreatePinnedToCore(Odometry, "Odometry", 4096, NULL, 3, &thp[5], 0);

  if( xMutex != NULL ){
    xTaskCreatePinnedToCore(VL53L0X, "VL53L0X", 8192, NULL, 4, &thp[3], 1);
    xTaskCreatePinnedToCore(eyes, "eyes", 8192, NULL, 3, &thp[4], 1);
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
        x_1=rx_buffer[5];
        y_1=rx_buffer[6];
        //Serial.print(x);
        //Serial.print(",");
        //Serial.println(y);
      }
    }
  }  
}

void following_X(void *args){
  while(1){
    delay(1);

    while(x_1<=106&&i<=120&&x_1!=225){//停止座標、サーボリミット
      delay(1);
      i++;
      Servo1.write(i);
      //Serial.println(val);
      delay(50);
    }

    while(x_1>=118&&i>=60&&x_1!=225){//停止座標、サーボリミット
      delay(1);
      i--;
      Servo1.write(i);
      //Serial.println(val);
      delay(50);
    }
  }
}

void following_Y(void *args){
  while(1){
    delay(1);
    
    while(y_1<=106&&i2<=100&&y_1!=225){//停止座標、サーボリミット
      delay(1);
      i2++;
      Servo2.write(i2);
      delay(50);
    }

    while(y_1>=118&&i2>=80&&y_1!=225){//停止座標、サーボリミット
      delay(1);
      i2--;
      Servo2.write(i2);
      delay(50);
    }
  }
}

void Odometry(void *args){
  int P_1 , P_2;
  float lR_1 = 0;
  float lL_1 = 0;
  float AG_1 = 0;
  float X_1 = 0;
  float Y_1 = 0;
  float AG = 0;
  while(1){
    delay(1);
    
    xQueueReceive( xQueue_1, &P_1, portMAX_DELAY );
    xQueueReceive( xQueue_2, &P_2, portMAX_DELAY );
    
    int dist_1_1 = dist_1;
    int dist_2_1 = dist_2;
    int dist_3_1 = dist_3;
    
    /*Serial.print("P:");
    Serial.print(P_1);
    Serial.print(",");
    Serial.println(P_2);*/
    
    float lR = 0.044 * P_1;//走行距離
    float lL = 0.044 * P_2;

    /*Serial.print("l:");
    Serial.print(lR);
    Serial.print(",");
    Serial.println(lL);*/

    float Vr_1 = (lR - lR_1) / 2;//各ホイールの速度
    float Vl_1 = (lL - lL_1) / 2;

    /*Serial.println(Vr_1);
    Serial.println(Vl_1);*/

    float V_1 = (Vr_1 + Vl_1) / 2;//全体の速度

    /*Serial.print("V_1:");
    Serial.println(V_1);*/

    float AG = (lR - lL) / 132;//角度

    /*Serial.print("AG:");
    Serial.println(AG);*/

    float AVE_1 = (Vr_1 - Vl_1) / 132;//角速度

    /*Serial.print("AVE_1:");
    Serial.println(AVE_1);*/

    float delta_AG_1 = AG - AG_1;//Δ角度

    /*Serial.print("delta_AG:");
    Serial.println(delta_AG_1);*/

    if(AVE_1==0){
      X = X_1 + V_1 * 2 * cos(AG_1);
      Y = Y_1 + V_1 * 2 * sin(AG_1);
    } else{
      X = X_1 + (2 * V_1) / AVE_1 * cos(AG_1 + (delta_AG_1 / 2)) * sin((delta_AG_1) / 2);
      Y = Y_1 + (2 * V_1) / AVE_1 * sin(AG_1 + (delta_AG_1 / 2)) * sin((delta_AG_1) / 2);
    }
    
    /*Serial.print("X:");
    Serial.println(X);
    Serial.print("Y:");
    Serial.println(Y);*/

    AG_1 = AG;//前回の値として保持
    X_1 = X;
    Y_1 = Y;
    lR_1 = lR;
    lL_1 = lL;

    //float Decimal_1 = root / 2 * (dist_1 + 17.84) + 37.5;三平方の定理を使用してxy軸の長さを推定
    //float Decimal_2 = root / 2 * (dist_2 + 17.84) + 37.5;
    
    float rad = i * (3.14 / 180);//度をラジアンに変換

    float Decimal_3_X =  (dist_3_1  + 67.5) * cos(rad);//いろいろ計算
    float Decimal_3_Y =  (dist_3_1  + 67.5) * sin(rad);

    /*Serial.println(Decimal_3_X);
    Serial.println(i);*/

    CALC_1 = round(root / 2 * (dist_1_1 + 17.84) + 37.5);//いろいろ計算&小数点以下切り捨て
    CALC_2 = round(root / 2 * (dist_2_1 + 17.84) + 37.5);
    CALC_3_X = round(Decimal_3_X);
    CALC_3_Y = round(Decimal_3_Y);

    CALC_1 = CALC_1 / 10;//ミリをセンチに変換
    CALC_2 = CALC_2 / 10;
    CALC_3_X = CALC_3_X / 10;
    CALC_3_Y = CALC_3_Y / 10;

    int CALC_1_X = (CALC_1 * -1) + 100;//座標に変換
    int CALC_1_Y = CALC_1 + 100;

    /*Serial.print(F("CALC_1_X:"));
    Serial.println(CALC_1_X);
    Serial.print(F("CALC_1_Y:"));
    Serial.println(CALC_1_Y);
    Serial.print(F("CALC_2:"));
    Serial.println(CALC_2);
    Serial.print(F("CALC_3_X:"));
    Serial.println(CALC_3_X);
    Serial.print(F("CALC_3_Y:"));
    Serial.println(CALC_3_Y);*/

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

    sendMapData(Serial);

    Serial.println(map1[1][1]);
  }
}

void eyes(void *args){
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

      xSemaphoreGive(xMutex);

      delay(3000);

      xStatus = xSemaphoreTake(xMutex, portMAX_DELAY);

      if(xStatus == pdTRUE){
        display.clearDisplay();
        display.fillRect(22,0,84,32,WHITE);
        display.display();

        delay(100);
      }
    }
    xSemaphoreGive(xMutex);
  }
}

void VL53L0X(void *args){
  
  BaseType_t xStatus;
  //const TickType_t xTicksToWait = delay1;
  xSemaphoreGive(xMutex);

  while(1){

    xStatus = xSemaphoreTake(xMutex, portMAX_DELAY);

    if(xStatus == pdTRUE){
      /*Serial.println (F("I2C scanner. Scanning ..."));
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
      Serial.println(F(" device(s)."));*/

      dist_1 = sensor1.readRangeContinuousMillimeters(); //センサーから距離[mm]
      /*Serial.print(dist_1);
      Serial.print(F("[mm]\n"));*/

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor1.timeoutOccurred()){
        Serial.print(F("time-out\n"));
      }

      dist_2 = sensor2.readRangeContinuousMillimeters(); //センサーから距離[mm]
      /*Serial.print(dist_2);
      Serial.print(F("[mm]\n"));*/

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor2.timeoutOccurred()){
        Serial.print(F("time-out\n"));
      }

      dist_3 = sensor3.readRangeContinuousMillimeters(); //センサーから距離[mm]
      /*Serial.print(dist_3);
      Serial.print(F("[mm]\n"));*/

      /*タイムアウトが起きた場合はタイムアウトが起きたことを出力する。*/
      if (sensor3.timeoutOccurred()){
        Serial.print("タイムアウトが起きました\n");
      }
      xSemaphoreGive(xMutex);
      delay(1000);
    }
  }
}

void StepMotor(void *args){
  while(1){
    delay(1);
    while(Order==1){
      delay(speed_1);

      //step1
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step2
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step3
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step4
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step5
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step6
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,1);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step7
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,1);


      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step8
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,1);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );
      
    }

    while(Order==2){
      delay(speed_1);

      //step1
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,1);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step2
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,1);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step3
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,1);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step4
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,1);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);

      //step5
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,0);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,1);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );

      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step6
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,1);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,0);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step7
      digitalWrite(IN1_1,0);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,1);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);


      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );

      delay(speed_1);
      
      //step8
      digitalWrite(IN1_1,1);
      digitalWrite(IN1_2,0);
      digitalWrite(IN1_3,0);
      digitalWrite(IN1_4,1);

      digitalWrite(IN2_1,1);
      digitalWrite(IN2_2,0);
      digitalWrite(IN2_3,0);
      digitalWrite(IN2_4,0);

      delay(1);
      count_1++;
      xQueueSend( xQueue_1, &count_1, 0 );
      
      count_2++;
      xQueueSend( xQueue_2, &count_2, 0 );
    }
  }
}