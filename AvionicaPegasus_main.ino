/*
    BEYOND ROCKET DESIGN - UNIFEI

    Subequipe de Sistemas Eletrônicos

    Software Aviônica Pégasus
    Out/2022

    Desenvolvedores:
      - Davi José M. Cunha
      - Helder Pires Gama
      - Lucas A. Noce
      - Matheus Siston Galdino

    *****************************************************************

    Software de aquisição, armazenamento e transmissão de dados
    e acionamento do sistema de recuperação da missão Pégasus 2022.
    Baseado em 9 estados de operação, conta com uma lógica de
    detecção de uma possível perda inesperada de energia no circuito,
    voltando ao estado anterior à queda do sistema.

    Estados de operação:
      S0 - Power Up
      S1 - Inicialização
      S2 - Remove Before Flight
      S3 - Waiting Flight Start
      S4 - Subindo
      S5 - Apogeu
      S6 - Descendo
      S7 - Aguardando Recuperação
      S8 - Foguete Recuperado
*/





/*  ==========================  */
/*          INCLUDES:           */

#include "sysConfig.h"
//#include "sysFunctions.c"
//#include "MaquinaDeEstados.c"

#include "BRDpacotes.h"        // Tratamento do pacote de dados
#include "EEPROM.h"            // EEPROM interna
#include <Wire.h>              // Geral
#include <SPI.h>               // SPI for LoRa
#include <LoRa.h>              // LoRa
#include <TinyGPS++.h>         // GPS
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>   // BMP280
#include <Adafruit_MPU6050.h>  // MPU6050
#include <Adafruit_Sensor.h>





/*  ==========================  */
/*      VARIAVEIS GLOBAIS:      */


/*  ESTADOS:  */
typedef enum{
  ESTADO_S0,
  ESTADO_S1,
  ESTADO_S2,
  ESTADO_S3,
  ESTADO_S4,
  ESTADO_S5,
  ESTADO_S6,
  ESTADO_S7,
  ESTADO_S8,
  ESTADO_ERRO,  // Nunca deve chegar aqui
} estados;


/*  EEPROM INTERNA:  */
EEPROMClass SYS_CONTROL_EEPROM("eeprom0", SYS_CONTROL_EEPROM_SIZE);
EEPROMClass DADOS_EEPROM("eeprom1", DADOS_EEPROM_SIZE);
uint16_t EnderecoDadosEEPROM = 0;
uint8_t ResetsInesperados = 0;
uint8_t EstadoAtual = 0;
uint16_t ExecucaoAtual = 0;
unsigned long TempoInicialVoo = 0;


/*  SENSORES:  */

// BMP280:
Adafruit_BMP280 bmp;
float PressaoAtmLocal = 0.0;
float AltitudeInicial = 0.0;
float TemperaturaInicial = 0.0;

// MPU6050:
Adafruit_MPU6050 mpu;

// GPS:
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);


/*  PACOTE DE DADOS (PROJETO X):  */
DadosFoguete PacoteDadosFoguete;
Comando PacoteComando;
InitFoguete PacoteInitFoguete;





/*  ==========================  */
/*            SETUP:            */

void setup() {
  uint8_t stateMachine = ESTADO_S0;

  do{
    switch(stateMachine){
      case ESTADO_S0:
        estadoS0();
        break;
      case ESTADO_S1:
        estadoS1();
        break;
      case ESTADO_S2:
        estadoS2();
        break;
      case ESTADO_S3:
        estadoS3();
        break;
      case ESTADO_S4:
        estadoS4();
        break;
      case ESTADO_S5:
        estadoS5();
        break;
      case ESTADO_S6:
        estadoS6();
        break;
      case ESTADO_S7:
        estadoS7();
        break;
      case ESTADO_S8:
        estadoS8();
        break;
      default:
        estadoS8();
        break;
    }
  } while(!ESTADO_ERRO);


  /*estadoS0();

  int8_t proxEstado = estadoS1();

  if (proxEstado == 2) goto S2;
  else if (proxEstado == 3) goto S3;
  else if (proxEstado == 4) goto S4;
  else if (proxEstado == 5) goto S5;
  else if (proxEstado == 6) goto S6;
  else if (proxEstado == 7) goto S7;
  else if (proxEstado == 8) goto S8;
  else{
    // fail safe (estado idefinido)
    while(1);
  }

S2:
  estadoS2();
S3:
  estadoS3();
S4:
  estadoS4();
S5:
  estadoS5();
S6:
  estadoS6();
S7:
  estadoS7();
S8:
  estadoS8();*/
}





/*  ==========================  */
/*            LOOP:             */

void loop(){}
