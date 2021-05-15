#include <SPI.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include "SdFat.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_PWMServoDriver.h>
#include <AS5600.h>

#define launchPin A0  // quando stacchi va HIGH e parte il conteggio
#define dataLen 20    // lunghezza array di salvataggio (con 20 funziona ancora tutto)

#define SERVOMIN  120 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  470 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // prequenza di lavoro dei servi

unsigned int alfa=0, pos=0, countSD=0;
unsigned int relasePara = 6000;
unsigned int relaseWing = 2000;
unsigned long tRec[dataLen+1], t0=0, tPara=0, tWing=0;

float val=0.0,presSlm1=0.0,presSlm2=0.0,tmp=0.0,tmp2=0.0;
float presRec1[dataLen+1], presRec2[dataLen+1], pitchAngRec[dataLen+1]; // sarebbe meglio fossero int...

String dataString = "";
bool deployedWing=0,deployedPar=0;

// oggetti classi varie:
SdFat sd;
SdFile dataFile;
Adafruit_BMP280 bmp1;
Adafruit_BMP280 bmp2;
AS5600 encoder;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //0x40

//-----------------------------------------------------------------------------------

MPU6050 mpu6050(Wire);

int tstep = 20; // si puo cambare per i tre movimenti
int t[3];
float errp[3];
float erri[3];
float ref[3];
float y[3];
float kp[3];
float kd[3];
float ki[3];

// per pid
float err;
float P;
float D;
float I;
float cont;

// 0 rollio 1 beccheggio 2 imbardata

//---------------------------------------------

bool apogeo;

void setup() {

//-------------------------------- nuovo

apogeo = false;
  
  kappa();
   mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // vedere come si vuol far partire e aspettare i 7 secondi

for (int i = 0 ; i < 3 ; i++) {
    t[i] = millis();
    erri[i] = 0;
    errp[i] = 0;
    y[i] = 0;
    ref[i] = 0;
  }

  
  //---------------

Serial.begin(9600);
Wire.begin(); //start i2C  
Wire.setClock(800000L); //fast clock
pinMode(launchPin,INPUT);

//setup board dei motori
pwm.begin();
pwm.setOscillatorFrequency(27000000);
pwm.setPWMFreq(SERVO_FREQ);
alfa = 0;
pos = map(alfa, 0, 180, SERVOMIN, SERVOMAX);
pwm.setPWM(0,0,pos);
pwm.setPWM(1,0,pos);

//setup sensori pressione
if (!bmp1.begin(0x76)) {
  Serial.println(F("Could not find BMP280_1 sensor"));
  while (1) delay(10);
}
bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
if (!bmp2.begin(0x77)) {
  Serial.println(F("Could not find BMP280_2 sensor"));
  while (1) delay(10);
}
bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

// misuro la pressione a terra             
Serial.println(F("Measuring sea-level pressure: "));                
for(int i=0;i<20;i++) {
  bmp1.readPressure();
  bmp2.readPressure();  
  if(i>=10){
    tmp+=bmp1.readPressure();
    Serial.print(bmp1.readPressure());
    Serial.print(" ");
    
    tmp2+=bmp2.readPressure();
    Serial.println(bmp2.readPressure());
  }
  delay(100);
}
presSlm1 = tmp*1.0/10.0;
presSlm2 = tmp2*1.0/10.0;
Serial.println(presSlm1);
Serial.println(presSlm2);

// aspetto finche il razzo non parte e salvo t0
while(analogRead(launchPin)<512){
  Serial.println(F("Waiting launch..."));
  delay(1);
}
t0 = millis();
Serial.println(t0);

// salvo su SD i dati iniziali
if(!sd.begin(10)) { Serial.println(F("error Sd card")); } // ha bisogno di almeno 307 byte di memoria
else{
  Serial.println(F("Sd card OK"));
  dataFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END);
  dataFile.println(F("NUOVO TEST DI VOLO")); dataFile.println();
  dataFile.print(F("pressioneSLM_1 = "));
  dataFile.print(presSlm1);
  dataFile.print(F(" pressioneSLM_2 = "));
  dataFile.println(presSlm2);
  dataFile.print(F(" tempo alla partenza = "));
  dataFile.println(t0);
  dataFile.close();  
}

}
//----------------------------------------------------------------------------------------------

void loop() {
  
  //Serial.println(millis()-t0);
  //delay(100);

  //sgancio l ala
  if(abs(millis()-t0) >= relaseWing && deployedWing==0){
    alfa = 180;
    pos = map(alfa, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0,0,pos);// servo 0 --> ala
    sd.begin(10);
    dataFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END);
    dataFile.println(); dataFile.println(F("Wing deployed")); dataFile.println();
    Serial.println(); Serial.println(F("Wing deployed")); Serial.println();
    dataFile.close();
    deployedWing=1;
    
  }

  //sgancio il paracadute
  if(abs(millis()-t0) >= relasePara && deployedPar==0){
    alfa = 180;
    pos = map(alfa, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(1,0,pos);// servo 1 --> paracadute
    sd.begin(10);
    dataFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END);
    dataFile.println(); dataFile.println(F("Parachute deployed")); dataFile.println();
    dataFile.close();
    Serial.println(); Serial.println(F("Parachute deployed")); Serial.println();
    deployedPar=1;
  }
  
  //Leggo dati pressione
  tRec[countSD] = 8 + millis() - t0;
  presRec1[countSD] = bmp1.readPressure();
  presRec2[countSD] = bmp2.readPressure();
  
  // leggo dati angolo attacco (10 = 1grado)
  pitchAngRec[countSD] = encoder.getAngle()*3600./4095.;
  Serial.print(F("pitchAngle: "));
  Serial.println(pitchAngRec[countSD]);

  //salvo su SD gli array di dati
  countSD++;  
  if(countSD == dataLen) {
    countSD = 0;
    if(!sd.begin(10)) { Serial.println(F("error Sd card")); } // ha bisogno di almeno 307 byte di memoria
    else{
      //Serial.println(F("Sd card OK"));
      dataFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END);
      for(int i=0;i<dataLen;i++){
        dataString = tRec[i];
        dataString += " ";
        dataString += presRec1[i];
        dataString += " ";
        dataString += presRec2[i];
        dataString += " ";
        dataString += pitchAngRec[i];
        dataFile.println(dataString);
        //Serial.println(dataString);
      }
      dataFile.close();  
    } 
  }
  if (apogeo == true){
  for (int i = 0 ; i < 3 ; i++) {
    if ( millis() - t[i] > tstep) { // si puo separare il pid dai sensori
      sensori (i);
      pid(i);
      t[i] = millis();
    }
  }
  }
}


void sensori (int i) { // da controllare

  switch (i) {
    case 0: y[i] = mpu6050.getAngleX(); break;
    case 1: y[i] = mpu6050.getAngleY(); break;
    case 2: y[i] = mpu6050.getAngleZ(); break;
  }

  // da mettere sensore angolo di attacco per ref in beccheggioo
}



void pid(int i) {  // servi da 2 a 7 per gli alettoni

  err = y[i] - ref[i];

  P = kp[i] * err;
  D = kd[i] * (err - errp[i]);
  if (abs(err) < 5) {
    erri[i] = erri[i] + err;
  }

  I = ki[i] * erri[i];
  errp[i] = err;

  cont = P + D + I;

    alfa = 90-cont;
    pos = map(alfa, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(i*2+2,0,pos);
    if(i=0){
      alfa = 90+cont;   // perche in rollio si devono muovere opposti
      pos = map(alfa, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(i*2+3,0,pos);}
    else{
    pwm.setPWM(i*2+3,0,pos);}


}

void kappa(){
  
kp[0] = 0;
kd[0] = 0;
ki[0] = 0;
kp[1] = 0;
kd[1] = 0;
ki[1] = 0;
kp[2] = 0;
kd[2] = 0;
ki[2] = 0;

}
