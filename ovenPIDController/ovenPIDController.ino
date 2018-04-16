// Controlador PID para o forninho/cultura bactéria
// Autor: Emiliano Sauvisky <esauvisky@gmail.com> 
// Fork Alexandre <lekobh@gmail.com>

// TODO: Adicionar controles para mudar setPoint em tempo real
// TODO: Testar ativar P_ON_M quando o gap entre input e setPoint for pequeno

/**************************
        Bibliotecas
**************************/
//#include <max6675.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PID_v1.h>

/**************************
  Definição de constantes
**************************/
// Pinos do Módulo MAX6675
//#define THERMO_GND 6
//#define THERMO_VCC 5
//#define THERMO_DO  4
//#define THERMO_CS  3
//#define THERMO_CLK 2


// Pinos do Sensor DS18B20
OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)


// Pinos do LCD
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13

// Símbolo ºC bonitinho para o LCD
uint8_t degree[8] = {140, 146, 146, 140, 128, 128, 128, 128};

// Definições Sensor de temperatura DS18B20
byte i, present = 0,type_s, data[12], addr[8], addr2[8];
float celsius, fahrenheit;

// Definições PID
double  temperature=10, setPoint=32, output;
unsigned long lastTempUpdate, windowStartTime;
#define WINDOW_SIZE        5000
#define MINIMUM_RELAY_TIME 500
#define RELAY_PIN          A0
#define ERRO_PIN           A1
#define TEMP_READ_DELAY    1000
#define KP                 45
#define KI                 0.05
#define KD                 20

/***************************
  Inicialização de Objetos
***************************/
// Cria o objeto do MAX6675
//MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_DO);

// Cria o objeto do LCD
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Cria o objeto do PID
//PID myPID(&temperature, &output, &setPoint, KP, KI, KD, P_ON_M, DIRECT);
PID myPID(&temperature, &output, &setPoint, KP, KI, KD, DIRECT);


/* updateTemperature()
    Atualiza a temperatura no menor tempo possível de acordo com o IC
    Retorna true  se a temperatura foi atualizada
    Retorna false se ainda não passou o tempo para realizar nova leitura */
void updateTemperature() {
   if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
 
          ds.reset();
          ds.select(addr);    
          ds.write(0xBE);         // Read Scratchpad
          for ( i = 0; i < 9; i++) {           // we need 9 bytes
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
         }
   }
  
  /***************************
        Pre-Configurações
  ***************************/
  void setup() {
      Serial.begin(9600);
  
  // Define a temperatura desejada
        setPoint = 32;
  
  // Define o pino do relay como um output
      pinMode(RELAY_PIN, OUTPUT);
     
      pinMode(ERRO_PIN, OUTPUT);  
 
  // Aguarda a inicialização do DS18B20
       while ( !ds.search(addr)) {
          ds.reset_search();
         delay(250);
       }
       ds.reset(); // rest 1-Wire
       ds.select(addr); // select DS18B20
       ds.write(0x4E); // write on scratchPad
       ds.write(0x00); // User byte 0 - Unused
       ds.write(0x00); // User byte 1 - Unused
       ds.write(0x7F); // set up en 9 bits 0x0F para 12 colocar (0x7F)
       ds.reset(); // reset 1-Wire
       //ds.reset();
       ds.select(addr);
       ds.write(0x44, 1);        // start conversion, with parasite power on at the end
       lastTempUpdate = millis();
  
  // Inicia o módulo do LCD (2 linhas, 16 colunas por linha)
       lcd.begin(16, 2);
  
  // Cria o símbolo de graus ao LCD
      lcd.createChar(0, degree);
      lcd.clear();

  // Seta o output do myPID entre 0 e WINDOW_SIZE.
      myPID.SetOutputLimits(0, WINDOW_SIZE);
  
  // Seta o sampling time para 125ms
      myPID.SetSampleTime(125);
  
  // Inicializa o PID
      myPID.SetMode(AUTOMATIC);

}


/***************************
       Loop Principal
***************************/
void loop() {
    // Faz controle de tempo proporcional para determinar se o relê deve ser ligado ou não
    unsigned long now = millis();
    if (now - windowStartTime > WINDOW_SIZE) {
        // Limpa o LCD a cada ciclo
        lcd.clear();
        windowStartTime += WINDOW_SIZE;
    }

    // Atualiza a temperatura
    updateTemperature();



     
    // Roda o cálculo do PID
    myPID.Compute();

    // Imprime a primeira linha do LCD (a temperatura atual)
    lcd.setCursor(0, 0);
    lcd.print("Temp:"); lcd.print(temperature);
    lcd.setCursor(14, 0);
    lcd.write((byte)0); lcd.print("C");

    // Imprime a segunda linha (o output)
    lcd.setCursor(0, 1);
    lcd.print("Output: "); lcd.print(int(output));

    // Liga o relê baseado no output do pid
    if (output > now - windowStartTime) {
        // TODO: Não gastar clicks do relê se a janela for muito pequena
        //if (output > MINIMUM_RELAY_TIME) {
        lcd.setCursor(13, 1); lcd.print(" ON");
        digitalWrite(RELAY_PIN, HIGH);
        //}
    } else {
        //if (WINDOW_SIZE - output < MINIMUM_RELAY_TIME) {
        lcd.setCursor(13, 1); lcd.print("OFF");
        digitalWrite(RELAY_PIN, LOW);
        //}
    }

    delay(50);
}
