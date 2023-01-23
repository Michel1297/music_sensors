#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEMidi.h>

TaskHandle_t Task1;
TaskHandle_t Task2;



#define encoder0PinA  34
volatile int NumPulsos; //variable para la cantidad de pulsos recibidos
float factor_conversion=7.5; //para convertir de frecuencia a caudal

int d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11 = 0;


void doEncoder0A(){ 
  NumPulsos++;  //incrementamos la variable de pulsos
} 

int ObtenerFrecuencia() 
{
  int frecuencia;
  NumPulsos = 0;   //Ponemos a 0 el número de pulsos
  interrupts();    //Habilitamos las interrupciones
  delay(100);   //muestra de 1 segundo
  noInterrupts(); //Desabilitamos las interrupciones
  frecuencia=NumPulsos*10; //Hz(pulsos por segundo)
  return frecuencia;
}

void sensors_control (void *param){
  
  Serial.begin(115200);

  Wire.begin(32,33,400000UL);

  // Create a sensor object
  Adafruit_MPU6050 mpu;
  sensors_event_t a, g, temp;
  float gyroX, gyroY, gyroZ;
  float accX, accY, accZ;
  float temperature;
  //Gyroscope sensor deviation
  float gyroXerror = 0.07;
  float gyroYerror = 0.03;
  float gyroZerror = 0.01;

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


  pinMode(encoder0PinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, RISING);  // encoder PIN A
  
  while(true){
    
    //Datos de sensor de caudal 
    float frecuencia=ObtenerFrecuencia(); //obtenemos la Frecuencia de los pulsos en Hz
    float caudal_L_m=frecuencia/factor_conversion; //calculamos el caudal en L/m
    float caudal_L_h=caudal_L_m*60; //calculamos el caudal en L/h
    
    //Datos MPU6050
    
    mpu.getEvent(&a, &g, &temp);

    float gyroX_temp = g.gyro.x;
    if(abs(gyroX_temp) > gyroXerror)  {
      gyroX += gyroX_temp/50.00;
    }
    float gyroY_temp = g.gyro.y;
    if(abs(gyroY_temp) > gyroYerror) {
      gyroY += gyroY_temp/70.00;
    }
    float gyroZ_temp = g.gyro.z;
    if(abs(gyroZ_temp) > gyroZerror) {
      gyroZ += gyroZ_temp/90.00;
    }

    //Asingación y mapeo
    d1 = map(frecuencia,0,300,50,80);
    d2 = (int)map(abs(gyroX*180*PI),0,360,0,127); if(d2>127){d2 = 127;}
    d3 = (int)map(abs(gyroY*180*PI),0,360,0,127); if(d3>127){d3 = 127;}
    d4 = (int)map(abs(gyroZ*180*PI),0,360,0,127); if(d4>127){d4 = 127;}


  }

  
}

void MIDI_control (void *param){

  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("BLE MIDI ESP32 1");
  Serial.println("Waiting for connections...");

  while(true){
      
    if(BLEMidiServer.isConnected()) {             // If we've got a connection, we send an A4 during one second, at full velocity (127)
    int note = d1;
    int velocity = d2;
    float pitch = d3/127.0;

    if (note > 50){
      BLEMidiServer.pitchBend(1,1,pitch);
      BLEMidiServer.noteOn(1,note,velocity);
      delay(10);
      
      BLEMidiServer.noteOff(1,note,velocity);
      

      delay(1);
    }

    Serial.print("C: ");
    Serial.print(note);
    Serial.print(",");
    Serial.print(velocity);
    Serial.print(",");
    Serial.print(pitch);
    Serial.println();
  }
  }
}

void setup() {

  xTaskCreatePinnedToCore(sensors_control,"Task1",4096,NULL,1,&Task1,0);
  xTaskCreatePinnedToCore(MIDI_control,"Task2",4096,NULL,1,&Task2,1);
}

void loop() {
  
}

