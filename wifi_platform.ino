#include<Arduino.h>
#include <SoftwareSerial.h>
// Библиотека для работы с протоколом I2C (порты A5/SCL и A4/SDA)
#include <Wire.h>

SoftwareSerial wifiSerial(11,12);
int V = 200;//скорость вращения двигателей
//пины двигателей
int dir3 = 4;//левые5
int pwm3 = 6;
int dir2 = 2;//правые
int pwm2 = 5;//4
int sen_pin = 7;//датчик точки

String input = "";
bool ready = false;
int azimut = 0;
int sensor = 1;
byte Mode = 0;
bool isTransmitted = false;
int sig_delay = 40;

//переменные 2 режима (координаты)
int X = 0, Y = 0, Xp = 0, Yp = 0;
int OyZero = 0;// если ось повернута вправо, то дописываем со знаком -
        // если ось повернута влево, то дописываем со знаком +



//переменные для гироскопа
int PROG;
const int MPU_addr = 0x68; // упрощеный I2C адрес нашего гироскопа/акселерометра MPU-6050.
// переменные для хранения данных возвращаемых прибором.
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double AcYsum, GyXsum;
// Угол поворота вокруг оси Z.
float Ang_;
const int key_flash = 14;
const int flash_data = 15;

float CompensatorZ, CompensatorAcX;
//смещение нуля баланса.
double balancing_zerro = 2.4;
//Время на разгон/торможение в микросекундах.
const long time_razgon = 100000;
//время движения вперед/назад за 1 шаг[mcs]
const long timeForwBackwrd = 500000;
//скорректированный относительно 90 градусов угол
const float angle_90 = 50;

//декларация некоторых функций
void Angle(float);
void backward_t(long);
void forward_t(long);

//функция поворота  
int LRAction(String s){    
    if(s.equals("1")){//влево                                      
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);        
        delay(5);
        digitalWrite(dir2,LOW);
        digitalWrite(dir3,HIGH);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);        
      }
      if(s.equals("3")){ //вправо
        analogWrite (pwm3, LOW);
        delay(5);
        digitalWrite(dir2,HIGH);
        digitalWrite(dir3,LOW); 
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);
      }  
    return 0;
  }    

  

  //функция смены направления движения (а также остановка)
int move(String napr){
      if(napr.equals("1")){   //вперед
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);       
        delay(5);        
        digitalWrite(dir2,HIGH);
        digitalWrite(dir3,HIGH);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);       
      }
      if(napr.equals("2")){//остановка        
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);        
      }
      if(napr.equals("3")){//назад
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);
        delay(5);
        digitalWrite(dir2,LOW);
        digitalWrite(dir3,LOW);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);     
      }
      return 0;
  }

changeAngle(boolean clockWise){
      if(!clockWise){
          azimut = azimut + 90;
          if(azimut >= 360)
            azimut = 0;
        }else{
          azimut = azimut - 90;
          if(azimut < 0){
             azimut = 270;
              }
          
          }
        
  }
void mode1Drive(String mes){
  
    //Serial.println("Mode1: " + mes);
    String ch1 = mes.substring(3,4);
    String ch2 = mes.substring(4,5);
    String ch3 = mes.substring(5,6);

    int v = ch3.toInt();
    V = map(v,0,9,0,255);
    Serial.println("mode1Drive");
    if(!ch2.equals("2")){
        //LRAction(ch2);
        if(ch2.equals("3")){
            //поворот вправо 90 градусов
            changeAngle(false);
            Angle(-angle_90);//здесь и далее: есть некоторые расхождения в гироскопе, поэтому угол задается меньше
          }
         if(ch2.equals("1")){
            //поворот влево 90 градусов
            changeAngle(true);
            Angle(angle_90);           
          }
    }else{
        //move(ch1);
        if(ch1.equals("1")){
              //назад заданное время
              backward_t(timeForwBackwrd);
          }
        if(ch1.equals("3")){
              //вперед заданное время 
              forward_t(timeForwBackwrd);
          }
    }
  }

void mode2Drive(String mes){

  }

  
String getMessage(byte mode, int sens, int azim){
  String ret = "";
    if(mode == 0){
      ret +="b1/";
      }else{
        ret += "b2/";
        }
     ret = ret + sens + "/";
     ret = ret + azim + "/e";
     return ret;
  }


void sendMessage(String mes){
  wifiSerial.println("AT+CIPSEND=10");   
    boolean isSendOk = false;
    input = "";
    while(!isSendOk){
      while (wifiSerial.available()){
        char c = wifiSerial.read();
          if(c == '\n'| c == '\r'){
            delay(40);
               break;
              }
        input += c;  
        delay(3);
          }          
         
       if(input.endsWith("OK> ")){
          input = ""; 
          wifiSerial.println(mes);        
        }

        if(input.endsWith("SEND OK")){
          input = ""; 
          isSendOk = true;
          //Serial.println("TCP send OK");
        }      
    }
  }

void parseAnsw(){
    boolean isParsed = false;
    input = "";
    while(!isParsed){
          while (wifiSerial.available()){
                     char c = wifiSerial.read();
                if(c == '\n'| c == '\r'){
                  delay(40);
                    break;
                }
                input += c;  
               // Serial.println(input);
                delay(3);//10
          }
          
          if(input.endsWith("e")){
             int beg_mes = input.lastIndexOf(":") + 1;
             String key = input.substring(beg_mes,beg_mes + 2);
             if(key.equals("b1")){
                Mode = 0;
                mode1Drive(input.substring(beg_mes));
             }else if(key.equals("b2")){
                Mode = 1;
                mode2Drive(input.substring(beg_mes));      
             }
            input = "";
            isParsed = true;
            isTransmitted = false;
          }
          
      }
  
  }

void wifiInit(){
      wifiSerial.println("AT");
    while(!ready){
    while (wifiSerial.available()){
    char c = wifiSerial.read();
    if(c == '\n'| c == '\r'){
      delay(40);
      break;
      }
      input += c;  
      delay(10);//задержка необходима, чтобы успели подъехать следующие символы
      //Serial.println(input);
    }
//Serial.println(input);
    if(input.endsWith("WIFI CONNECTED")){
          Serial.println("Wifi connected");
          input = "";
      }
    if(input.endsWith("WIFI GOT IPOK")){
        input = "";       
        Serial.println("Got IP");
        wifiSerial.println("AT+CIPSTART=\"TCP\",\"192.168.173.1\",777");
      }
     if(input.endsWith("CONNECT")){
        input = "";       
        Serial.println("TCP connection sucsesful!!!");
        ready = true;
      }
     if(input.endsWith("OK")){
          Serial.println("ready to connect");
          wifiSerial.println("AT+CWJAP_CUR=\"SVR-carTR\",\"cartransmitter\"");
          input = "";
      }

  }  
  
  
}

  
//=========================================================
void Data_mpu6050()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  //Готовим для чтения регистры  с адреса 0x3B.
  Wire.endTransmission(false);
  // Запрос 14 регистров.
  Wire.requestFrom(MPU_addr, 14, true);
  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcX = Wire.read() << 8 | Wire.read();
  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcY = Wire.read() << 8 | Wire.read();
  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();
  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Tmp = Wire.read() << 8 | Wire.read();
  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyX = Wire.read() << 8 | Wire.read();
  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyY = Wire.read() << 8 | Wire.read();
  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();
}



// задержка но с расчетом изменения Z угла.
void time_gyro(float mill_sec)
{
  long ms = mill_sec;
  unsigned long timer = micros();
  unsigned long endtime = micros() + long(ms * 1000.0);
  while (endtime > timer) {
    timer = micros();
    Data_mpu6050();
    Ang_ = Ang_ + (((float)(GyZ) - CompensatorZ) * (float)(micros() - timer)) / 131000000.0;
    delayMicroseconds(1);
  }
}



  //======================================
// Поворот на заданный угол. ////////
//======================================
void Angle(float ang)
{
 int bufferedV = V;
   //возможно стоит уменьшить скорость, если угол маленький.
  V = 255;
  
  bool flag = false;
  float Ang_2 = Ang_; long timer;

  if (ang == 0) return;
  if (ang < 0)
  {
    do {
      //right(); time_gyro(1); _stop(); time_gyro(1);
      LRAction("3");time_gyro(1);move("2");time_gyro(1);
    } while (Ang_ > (ang + Ang_2));
  }
  else
  {
    do {
      //left(); time_gyro(1); _stop(); time_gyro(1);
      LRAction("1");time_gyro(1);move("2");time_gyro(1);
    } while (Ang_ < (ang + Ang_2));
  }
  move("2");
  Ang_2 = Ang_ - (ang + Ang_2);
  if (abs(Ang_2) > 1.0)
    Angle(-Ang_2);
  V = bufferedV;
}




//движение вперед заданное время
void forward_t( long microsec)
{

  bool flag = false;
  microsec = microsec + micros();
  long microsec_begin_tormog = microsec - time_razgon;
  float Ang_2 = Ang_; long timer;
  do {
    timer = micros();
    if (Ang_ < (-1.0 + Ang_2)) {
      LRAction("1"); time_gyro(0.5);
    }
    else if (Ang_ > (1.0 + Ang_2)) {
      LRAction("3"); time_gyro(0.5);
    }
    else {
      move("3"); time_gyro(0.5); move("2"); time_gyro(0.8);
    }
    if ((timer > microsec_begin_tormog)) {
      move("2"); time_gyro(0.5);
    }
  } while (microsec > timer);
  move("2");
}


//движение назад заданное время
void backward_t( long microsec)
{
  bool flag = false;
  microsec = microsec + micros();
  long microsec_begin_tormog = microsec - time_razgon;
  float Ang_2 = Ang_; long timer;
  do {
    timer = micros();
    if (Ang_ < (-1.0 + Ang_2)) {
      LRAction("3"); time_gyro(0.5);
    }
    else if (Ang_ > (1.0 + Ang_2)) {
      LRAction("1"); time_gyro(0.5);
    }
    else {
      move("1"); time_gyro(0.5); move("2"); time_gyro(0.8);
    }
    if ((timer > microsec_begin_tormog)) {
      move("2"); time_gyro(0.5);
    }
  } while (microsec > timer);
  move("2");
}



void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
  pinMode(sen_pin,INPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);


//инициализация гироскопа
// Заносим в переменные номера контактов (пинов) Arduino.
  // Для левых и правых моторов машинки.
  // Двигатели остановлены.
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Производим запись в регистр энергосбережения MPU-6050
  Wire.write(0);     // устанавливем его в ноль
  Wire.endTransmission(true);
  AcYsum = 0;
  GyXsum = 0;

  CompensatorZ = 0; CompensatorAcX = 0;
  for (int i = 0; i < 500; i++)
  {
    delay(10);
    Data_mpu6050();
    CompensatorZ += GyZ;
    CompensatorAcX += AcX;
  }
  CompensatorZ = CompensatorZ / 500.0;
  CompensatorAcX = CompensatorAcX / 500.0;
  
  Serial.begin(9600);
  while(!Serial){}

  wifiSerial.begin(9600);
  delay(3);
  Serial.println("Setup begin");
  wifiInit();
  Serial.println("Ready to work");
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(1000);   
}

void loop() {

   Serial.println("p1");
   parseAnsw();
   Serial.println("p2");
   sensor = digitalRead(sen_pin);
   if(sensor)sensor=0;
    else sensor = 1;
   Serial.println("p3");
   sendMessage(getMessage(Mode,sensor,azimut));
   Serial.println("p4");
  delay(40);
}
