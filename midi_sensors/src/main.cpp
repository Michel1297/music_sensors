#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEMidi.h>

//Contenedores de los dos Threads a ejecutar 
TaskHandle_t Task1; //Task encargado de leer los sensores
TaskHandle_t Task2; //Task encargado de enviar las notas via MIDI BLE

//Creamos la instancia de la IMU MPU6050
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

//Definición de variables y parámetros

#define encoder0PinA  34      //Pin del encoder del caudalómetro
volatile int NumPulsos;       //variable para la cantidad de pulsos recibidos
float factor_conversion=7.5;  //para convertir de frecuencia a caudal

int d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11 = 0; //Variables para contener datos

//Función llamada por la interrupción del encoder
void doEncoder0A(){ 
  NumPulsos++;  //incrementamos la variable de pulsos
} 

//Función que calcula el caudal
int ObtenerFrecuencia() 
{
  int frecuencia;
  NumPulsos = 0;           //Ponemos a 0 el número de pulsos
  interrupts();            //Habilitamos las interrupciones
  delay(50);               //muestra de 1 segundo
  noInterrupts();          //Desabilitamos las interrupciones
  frecuencia=NumPulsos*10; //Hz(pulsos por segundo)
  return frecuencia;
}

//Thread 1: Loop encargado de actualizar los valores de los sensores

void sensors_control (void *param){
  
  
  Serial.begin(115200);           //Inicializamos la comunicación Serial

  //Configuración de la interrupción que calcula el caudal a partir de los datos del encoder
  pinMode(encoder0PinA, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, RISING);  // encoder PIN A
  
  while(true){
    
    //Datos de sensor de caudal 
    float frecuencia=ObtenerFrecuencia();           //obtenemos la Frecuencia de los pulsos en Hz
    float caudal_L_m = frecuencia/factor_conversion;  //calculamos el caudal en L/m
    float caudal_L_h = caudal_L_m*60;                 //calculamos el caudal en L/h

    
  


    d1 = map(frecuencia,0,300,50,80);

    vTaskDelay(pdMS_TO_TICKS(5)); //Delay 1 ms
  }

  
}

void MIDI_control (void *param){
  
  //Imprimimos un mensjase de inicio de la conexión BLE MIDI
  Serial.println("Inicializando BLE MIDI");
  BLEMidiServer.begin("ESP32 MIDI SENSOR 1");
  Serial.println("Esperando conexión con dispositivo");
  
    Wire.begin(32,33);     //Inicializamos la comunicación i2C para la IMU MPU6050


  //Variables locales de la IMU
  float gyroX, gyroY, gyroZ;
  float accX, accY, accZ;
  float temperature;
  //Desviación del girosopio
  float gyroXerror = 0.07;
  float gyroYerror = 0.03;
  float gyroZerror = 0.01;

  //Inicialación de la IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");


  while(true){
    
    //Datos MPU6050
    mpu.getEvent(&a, &g, &temp);

    //Cargamos los datos actuales de los sensores de caudal e IMU en el Thread MIDI

    //Datos de prueba random (también se pueden colocar otras entradas de datos)
    //int note = random(50,80);
    //uint8_t velocity = random(0,255);
    //float pitch = random(0,127)/10.0;

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

    //Asingación y mapeo de los datos obtenidos de la IMU y el Caudal
   
    d2 = (int)map(abs(gyroX*180*PI),0,360,0,127); if(d2>127){d2 = 127;}
    d3 = (int)map(abs(gyroY*180*PI),0,360,0,127); if(d3>127){d3 = 127;}
    d4 = (int)map(abs(gyroZ*180*PI),0,360,0,127); if(d4>127){d4 = 127;}

    int note = d1;
    uint8_t velocity = d4;
    float pitch = d3/10.0;
    
    //Revisamos si el servidor está conectado
    if(BLEMidiServer.isConnected()) {     
      
      //Mostramos los valores enviados en la pantalla del computador
      Serial.print("Note: ");
      Serial.print(note);
      Serial.print(", Velocity: ");
      Serial.print(velocity);
      Serial.print(", Pitchbend:");
      Serial.print(pitch);
      Serial.println();
      
      //Si la nota a enviar es mayor a 50
      if (note > 50){

        BLEMidiServer.pitchBend(1,pitch,4.0F);
        BLEMidiServer.noteOn(1,note,velocity);
        vTaskDelay(pdMS_TO_TICKS(30));
        BLEMidiServer.noteOff(1,note,velocity);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    else{
      //Mostramos los valores enviados en la pantalla del computador
      Serial.print("Note: ");
      Serial.print(note);
      Serial.print(", Velocity: ");
      Serial.print(velocity);
      Serial.print(", Pitchbend:");
      Serial.print(pitch);
      Serial.println();

      vTaskDelay(pdMS_TO_TICKS(70));

    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  
}

void setup() {
  
  //Creamos los dos Threads principales
  //disableCore0WDT();

  xTaskCreatePinnedToCore(sensors_control,"Task1",10000,NULL,1,&Task1,0); //Control y lectura de sensores, en el núcleo 0
  xTaskCreatePinnedToCore(MIDI_control,"Task2",10000,NULL,1,&Task2,1);    //Comunicación MIDI, en el núcleo 1
}

void loop() {
  
}



