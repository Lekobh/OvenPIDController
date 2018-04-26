// Controlador PID para o forninho/cultura bactéria
// 
// Fork Alexandre <lekobh@gmail.com>
// Mudei quase tudo usando a saida um aproximador 
// NEste momento totalmente zoneado  depois vou organizar para ficar inteligivel...

/**************************
        Bibliotecas
**************************/
#include <OneWire.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>



//Define Variables E2PROM salvar valores kp,ki,kd
int address = 0;
byte value;
int addrKp = 10;
int addrKi = 20;
int addrKd = 30;

//Define variaveis para o autotune
double Setpoint, Input, Output;

// Pinos usados
const int analogInPin = A0;  // pino analogico para etapade de teste do PID
const int BresOutPin = 7; // Pino de saida bresenham

// Pino sensor temperatura ainda não usado!
OneWire  ds(2);  // on pin 10 (a 4.7K resistor is necessary)
// Definições para o sensor de temperatura DS18xx
float celsius;
#define TEMP_READ_DELAY    1000
byte  present = 0,type_s, data[12], addr[8], addr2[8];
double  temperature=10, setPoint=32;


int sensorValue = 0;        // value read from the pino analogico de teste
int outputValue = 0;        // value output to the PWM (analog out)

// Variaveis para o calculo do bresenham
int  N, I, E;
int M = 40;// 40 = 1 segundo! 200; // cada ciclo 8,33ms cada 3 ciclos 25ms...  (200*3) /120 ciclos por segundo = 5 segundos 
unsigned long int tempo, ciclos, lastTempUpdate;


// Autotune
byte ATuneModeRemember=2;
double kp=0.16,ki=0.01,kd=0.00;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=M/2, aTuneNoise=1, aTuneStartValue=M;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long  modelTime, serialTime;


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd,P_ON_M, DIRECT); //   P_ON_M,   P_ON_M specifies that Proportional on Measurement be used
PID_ATune aTune(&Input, &Output);
//set to false to connect to the real world
//boolean useSimulation = false;





void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  
  //Verifica e limpa a E2Prom
  if (EEPROM.read(0)) Limpae2prom();
  //Verifica se é a primeira vez que inicializa
  if (!EEPROM.read(1)) {
      //Verificar se realmente faz diferença
      myPID.SetTunings(myPID.GetKp(),myPID.GetKi(),myPID.GetKd(),P_ON_E); // Volta ao controle para ir mais rapido.
      changeAutoTune(); // Ajusta valores do PID
  } else Lee2prom(); // Se não é a primeira vez Carrega os valores do ultimo ajuste do PID

 
  iniciads();
        
  //initialize the variables we're linked to
  Input = analogRead(analogInPin);
  Setpoint = 512;
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  serialTime = 0;
 
  
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, M); // M é o numero de ciclos do bresenham
  tempo=0;
  ciclos=0;
  
  //Config o pino de saída
  pinMode(BresOutPin, OUTPUT);
  CAPTU(); 

  //myPID.setBangBang(M);
  //myPID2.setTimeStep(1000);
  
}

void loop() {
 if (tempo == 0) tempo = millis();
   //unsigned long now = millis();
  //Serial.print(millis());
  //Serial.print("  *  ");
  //Serial.println(tempo + (25*M));

//unsigned long currentMillis = millis();
   Input = analogRead(0);
  if(tuning)
  {
    byte val = (aTune.Runtime());
  
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
      
    }
  } else {
     myPID.Compute();

   
   if ((millis() - tempo) > 1000){

    CAPTU();  
   }
  }
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  BRES();
if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=1000;
    if(tuning) CAPTU(); 
  }
  //updateTemperature();
}

void BRES(){

 if ((millis() - ciclos) >= 25){
    ciclos = millis(); 
    if (I < M){
      if (E >= 0){
          E = E + 2*(N-M);
         // Serial.println ("1 ");
          digitalWrite(BresOutPin, HIGH);
      }
      else {
          E=E + 2*N;
         // Serial.println ("0 ");
          digitalWrite(BresOutPin, LOW);
      }
        
    }
    else I=0;

}

}

void CAPTU(){

  //    Serial.println(tempo);
  // read the analog in value:
  if(!tuning){
        Input = analogRead(0);
        myPID.Compute();
    }
  //myPID2.run(); 
  N = Output;
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);


  // print the results to the Serial Monitor:
/*  Serial.print("sensor = ");
  Serial.print(Input);
  Serial.print("\t output = ");
  Serial.println(N);

  Serial.print("\t Kp = ");
  Serial.print(myPID.GetKp());
  Serial.print("\t Ki = ");
  Serial.print(myPID.GetKi());
  Serial.print("\t Kd = ");
  Serial.println(myPID.GetKd());
*/

  
   E= 2*N - M;
  //Serial.println ("\n Algoritmo de Bresenham – distribui igualmente N ciclos dentro do peridodo de M ciclos");
     tempo = millis();
   
}

void updateTemperature() {
   if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
 
          ds.reset();
          ds.select(addr);    
          ds.write(0xBE);         // Read Scratchpad
          for (int i = 0; i < 9; i++) {           // we need 9 bytes
            data[i] = ds.read();
          }
        
          ds.reset();
          ds.select(addr);
          ds.write(0x44, 1);        // start conversion, with parasite power on at the end
          lastTempUpdate = millis();
          // Convert the data to actual temperature
          // because the result is a 16 bit signed integer, it should
          // be stored to an "int16_t" type, which is always 16 bits
          // even when compiled on a 32 bit processor.
          int16_t raw = (data[1] << 8) | data[0];
          if (type_s) {
            raw = raw << 3; // 9 bit resolution default
            if (data[7] == 0x10) {
                    // "count remain" gives full 12 bit resolution
                    raw = (raw & 0xFFF0) + 12 - data[6];
                  }
                } else {
                  byte cfg = (data[4] & 0x60);
                  // at lower res, the low bits are undefined, so let's zero them
                  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
                  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
                  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                  //// default is 12 bit resolution, 750 ms conversion time
          }
          celsius = (float)raw / 16.0;
          temperature = celsius;
          Serial.print ("T= ");
          Serial.println (temperature);

                  }
   
   }


void iniciads(){
       // Aguarda a inicialização do DS18B20
       Serial.println ("Inicia o DS18B20");
       while ( !ds.search(addr)) {
          ds.reset_search();
         delay(250);
       }
       ds.reset(); // rest 1-Wire
       ds.select(addr); // select DS18B20
       ds.write(0x4E); // write on scratchPad
       ds.write(0x00); // User byte 0 - Unused
       ds.write(0x00); // User byte 1 - Unused
       ds.write(0x0F); // set up en 9 bits 0x0F para 12 colocar (0x7F)
       ds.reset(); // reset 1-Wire
       //ds.reset();
       ds.select(addr);
       ds.write(0x44, 1);        // start conversion, with parasite power on at the end
       lastTempUpdate = millis();
   }


void changeAutoTune()
{
  Serial.println("changeAutoTune");
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    Output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
    Serial.println("****TUNING***");
  }
  else
  { //cancel autotune
    
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start){
    ATuneModeRemember = myPID.GetMode();
    Serial.print("GETMODE: ");
    Serial.println(myPID.GetMode());
  }else{
    Serial.println("*NOTUNING*");
    myPID.SetMode(ATuneModeRemember);
    myPID.SetTunings(myPID.GetKp(),myPID.GetKi(),myPID.GetKd(),P_ON_M); // Volta ao controle para ir mais rapido.
    Atualie2prom();
  }
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(Setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(Input); Serial.print(" ");
  Serial.print("output: ");Serial.print(Output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  Input = (kpmodel / taup) *(theta[0]-outputStart) + Input*(1-1/taup) + ((float)random(-10,10))/100;

}


void   Limpae2prom()
{
    for (int j = 0 ; j < EEPROM.length() ; j++) {
    EEPROM.write(j, 0);
  }
}

void   Atualie2prom()
{
      Serial.println("Escreve EEPROM");
      EEPROM.write(1, 1);
      EEPROM.put(addrKp, myPID.GetKp());
      EEPROM.put(addrKi, myPID.GetKi());
      EEPROM.put(addrKd, myPID.GetKd());
      Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
      Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
      Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  
}

void   Lee2prom()
{
      Serial.println("Lê EEPROM");
      EEPROM.get(addrKp, kp);
      EEPROM.get(addrKi, ki);
      EEPROM.get(addrKd, kd);
      Serial.print("kp: ");Serial.print(kp);Serial.print(" ");
      Serial.print("ki: ");Serial.print(ki);Serial.print(" ");
      Serial.print("kd: ");Serial.print(kd);Serial.println();
      myPID.SetTunings(kp, ki, kd, P_ON_M); // Volta ao controle para ir mais seguro...
}
