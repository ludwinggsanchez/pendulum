#include "Simple_MPU6050.h"         // incluye libreria Simple_MPU6050
#include <Adafruit_MCP4725.h>
#include <Servo.h>
#include <Wire.h>
#define MPU6050_ADDRESS_AD0_LOW     0x68      // direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69      // direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW // por defecto AD0 en 

int steps[] = {-9, -20, 6, 15, 17, 2, 19, -13, 9, -20, -6, -15, -17, -2, -18, 13};
Simple_MPU6050 mpu;       // crea objeto con nombre mpu
Adafruit_MCP4725 dac;

ENABLE_MPU_OVERFLOW_PROTECTION();   // activa proteccion
#define OFFSETS  -1418,   -1220,    1686,      42,      35,       6
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
// spamtimer funcion para generar demora al escribir en monitor serie sin usar delay()

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
// printfloatx funcion para mostrar en monitor serie datos para evitar el uso se multiples print()
int plot,plo,co;
int escalon = 0;
long startTime=0;
int T = 200;
long n = 1;
long prueba;
int inic_motor=false;
int intPin = 2;
float vel_in;
float ang_in = -45;
//uint8_t val;
int val;
bool incrementar = false;
bool ready = false;
Servo esc;
// mostrar_valores funcion que es llamada cada vez que hay datos disponibles desde el sensor
void mostrar_valores (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {  
  uint8_t SpamDelay = 100;      // demora para escribir en monitor serie de 100 mseg
  Quaternion q;         // variable necesaria para calculos posteriores
  VectorFloat gravity;        // variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };     // array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };     // array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {      // si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);    // funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);   // funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity); // funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);   // funcion convierte a grados sexagesimales
        //-65
//    if(xyz[1]>=25) {
//      xyz[1]=xyz[1]-1;
//    }
//    if(xyz[1]>=45) {
//      xyz[1]=xyz[1]-2;
//    }
//    if(xyz[1]>=60){
//      xyz[1]=xyz[1]-1;
//    }
    //Serial.printfloatx(millis(), 4,F(";   "), xyz[1], 9, 4, F(";   "));  // muestra en monitor serie rotacion de eje Y, pitch
    String impr = String(millis())+", "+String(xyz[1])+", "+String(escalon);
    Serial.println(impr);
    // salto de linea

      plot = map(xyz[1], -90, 90, 0, 4095);
      // analogWrite(9, round(plot));
       
       //check_zero(plot);
      
      dac.setVoltage(plot, false);
      //delay(500);
      //delay(1);
      co = co + 1;
  }
}


//void interruptCount()
//{
//    val=analogRead(A5);
//}
//
//void check_zero (int16_t angle) {
//      if (angle >= 2045 && angle <= 2051 && ready == false){
//        delay(5000);
//        esc.writeMicroseconds(1300);
//        ready = true;
//      } else if (ready == false){
//        esc.writeMicroseconds(val);
//        delay(5000);
//      }
//}
void setup() {
  pinMode(7,OUTPUT);
  dac.begin(0x60);
 // degreesVolt=7;
  esc.attach(10);

 Serial.begin(115200);
  delay(500);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

//  pinMode(intPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(intPin), interruptCount, LOW);//
  
       // inicializacion de monitor serie a 115200 bps
  while (!Serial);      // espera a enumeracion en caso de modelos con USB nativo
  Serial.println(F("Inicio:"));   // muestra texto estatico
#ifdef OFFSETS                // si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));     // texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);  // inicializacion de sensor

#else                   // sin no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n" // muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read());    // lectura de monitor serie
  while (!Serial.available());        // si no hay espera              
  while (Serial.available() && Serial.read());    // lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();  // inicializacion de sensor
#endif
  mpu.on_FIFO(mostrar_valores);   // llamado a funcion mostrar_valores si memoria FIFO tiene valores
}
void loop() {
  //    // funcion que evalua si existen datos nuevos en el sensor y llama
//      if (plot >= 2045 && plot <= 2051 && ready == false){
//        delay(5000);
//        esc.writeMicroseconds(1300);
//        ready = true;
//      } else if (ready == false){
//        esc.writeMicroseconds(val);
//        delay(5000);
//      }
  // for (int i = 0; i < sizeof(steps); i = i+1){
  //   val=map(steps[i], -20 , 20, 1200, 1500);
  //   esc.writeMicroseconds(val);
  //   delay(5000);
  // }
 
 
//  if (millis() > 25000 && millis()<27000){
//    vel_in = map(ang_in, -90, 90, 1000, 1800);
//    esc.writeMicroseconds(vel_in);
//  }else{ esc.writeMicroseconds(1000);}
//
 
 


 
 if (millis() > n*T){
  mpu.dmp_read_fifo();
  //Serial.println("Entra el periodo"); 
  n=n+1;
  prueba = n*T;
  }
String lol = String(millis())+" "+String(n);
//Serial.println(lol); 
//
   if (millis() > 10000){               
     
       
        if(n%20 == 0){
          
              
                Serial.println(lol);
                ang_in = (0.25)*n-90;

        }
             vel_in = map(ang_in, -90, 90, 1000, 1800);
             esc.writeMicroseconds(vel_in);
        //delay(2000);          
  }   
    else{ esc.writeMicroseconds(1000);
    Serial.println("aqui");}

 

  
 //Para entrada análogica
//  val=analogRead(A2);
//  plo = map(val, 0, 1023, 1200, 1500);
//  delay(1);



//  if(inic_motor == false){
//  esc.writeMicroseconds(1000);
//  
//  delay(25000);
//  inic_motor = true;
//  escalon = 1420;
//  }
//  


  //delay(10000);
  //esc.writeMicroseconds(1450);
}       // a funcion mostrar_valores si es el caso  