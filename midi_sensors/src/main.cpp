#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEMidi.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

Adafruit_MPU6050 mpu;

TwoWire I2C1 = TwoWire(0);
TwoWire I2C2 = TwoWire(1);

#define encoder0PinA  34
volatile int NumPulsos; //variable para la cantidad de pulsos recibidos
float factor_conversion=7.5; //para convertir de frecuencia a caudal

float d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11 = 0.0;

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
  
}

void MIDI_control (void *param){
  
}

void setup() {


  Serial.begin(115200);
  
  I2C1.begin(32,33,100000UL);

  pinMode(encoder0PinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, RISING);  // encoder PIN A
  
  // Try to initialize!
  if (!mpu.begin(104U,&I2C1,0)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("BLE MIDI ESP32 1");
  Serial.println("Waiting for connections...");

  //BLEMidiServer.enableDebugging();  // Uncomment if you want to see some debugging output from the library (not much for the server class...)  
  //mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //xTaskCreatePinnedToCore(sensors_control,"Task1",4096,NULL,1,&Task1,0);
  //xTaskCreatePinnedToCore(MIDI_control,"Task2",4096,NULL,1,&Task2,1);

}

void loop() {
  
    //Datos de sensor de caudal 
    float frecuencia=ObtenerFrecuencia(); //obtenemos la Frecuencia de los pulsos en Hz
    float caudal_L_m=frecuencia/factor_conversion; //calculamos el caudal en L/m
    float caudal_L_h=caudal_L_m*60; //calculamos el caudal en L/h
  
    //Datos de acelerómetro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //-----Enviamos por el puerto serie---------------
     Serial.print ("FrecuenciaPulsos: "); 
    Serial.print (frecuencia,0); 
    Serial.print ("Hz\tCaudal: "); 
    Serial.print (caudal_L_m,3); 
    Serial.print (" L/m\t"); 
    Serial.print (caudal_L_h,3); 
    Serial.print ("L/h, "); 
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC"); 
  
    //Asingación y mapeo

    d1= map(frecuencia,0,300,0,127);
    d2= map(abs(a.acceleration.x),0,10.0,0,127);
    d3= map(abs(a.acceleration.y),0,10.0,0,127);
    d4= map(abs(a.acceleration.z),0,10.0,0,127);
    d5= map(abs(a.gyro.x),0,3.2,0,127);
    d6= map(abs(a.gyro.y),0,3.2,0,127);
    d7= map(abs(a.gyro.z),0,3.2,0,127);
    d9= map(temp.temperature,10,50,0,127);
/*
    Serial.print(d1);
    Serial.print(d2);
    Serial.print(d3);
    Serial.print(d4);
    Serial.print(d5);
    Serial.print(d6);
    Serial.print(d7);
    Serial.print(d8);
    Serial.print(d9);
    Serial.println();
*/
    if(BLEMidiServer.isConnected()) {             // If we've got a connection, we send an A4 during one second, at full velocity (127)
      
      BLEMidiServer.controlChange(0,0,d1);
      BLEMidiServer.controlChange(0,1,d2);
      BLEMidiServer.controlChange(0,2,d3);
      BLEMidiServer.controlChange(0,3,d4);
      BLEMidiServer.controlChange(0,4,d5);
      BLEMidiServer.controlChange(0,5,d6);
      BLEMidiServer.controlChange(0,6,d7);
      BLEMidiServer.controlChange(0,7,d8);
      BLEMidiServer.controlChange(0,8,d9);
  }
   
    
}

