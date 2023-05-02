/* Arquivo para implementação das funçoes globais */

#include "sysConfig.h"
//#include "MaquinaDeEstados.c"
//#include "AvionicaPegasus_main.ino"
#include "BRDpacotes.h"        // Tratamento do pacote de dados


/*  ======================================  */
/*                 SUMARIO:                 */
/*                                          */
/*     INICIALIZAÇAO                        */
/*         SERIAL                           */
/*         MPU6050                          */
/*         BMP280                           */
/*         // FLASH EXTERNA                 */  // TODO
/*         LORA                             */
/*         GPS                              */
/*         EEPROM INTERNA EP32              */
/*         REMOVE BEFORE FLIGHT (RBF)       */
/*         BUZZER                           */
/*         // LED                           */
/*         ELETROIMA                        */
/*         ABSTRAÇOES DE INIT               */
/*             Init Basico                  */
/*             Init Completo                */
/*     BEEPS E BLINKS                       */
/*         Beeps simples do buzzer          */
/*         Beep de erro do buzzer           */
/*         Beeps de início de voo           */  // TODO
/*         // Blinks do LED                 */
/*     CHECAGENS                            */
/*         Checar Remove Before Flight      */
/*         Checar Porta do Eletroima        */
/*     ELETROIMA                            */
/*         Abrir a porta                    */
/*         Fechar a porta                   */
/*     CALIBRAÇOES                          */
/*         CALIBRAÇAO DO BMP                */
/*             Pressao Atm Local            */
/*             Altura Inicial               */
/*             Temperatura Inicial          */
/*         CALIBRAÇAO DO MPU                */
/*             TODO                         */  // TODO
/*     LEITURA DE DADOS                     */
/*         EEPROM INTERNA                   */
/*             Resets Inesperados           */
/*             Estado anterior              */
/*             Execuçao anterior            */
/*             Apogeu anterior              */
/*             Aceleraçao Maxima            */
/*             Tempo Inicial de Voo         */
/*             Pressao Atmosferica Local    */
/*             Altura Inicial               */
/*             Temperatura Inicial          */
/*         SENSORES                         */
/*             MPU6050                      */  // TODO
/*             BMP280                       */
/*             GPS                          */
/*     ARMAZENAMENTO DE DADOS               */
/*         EEPROM INTERNA - SISTEMA         */
/*             Resets Inesperados           */
/*             Estado anterior              */
/*             Execuçao anterior            */
/*             Apogeu anterior              */
/*             Aceleraçoes Maximas          */
/*             Tempo Inicial de Voo         */
/*             Pressao Atmosferica Local    */
/*             Altura Inicial               */
/*             Temperatura Inicial          */
/*         EEPROM INTERNA - DADOS           */
/*             Dados dos Sensores           */
/*         // FLASH EXTERNA                 */
/*     COMUNICAÇAO DE DADOS                 */
/*         LORA                             */
/*             Transmite dados              */
/*             Recebe dados                 */
/*         SERIAL                           */
/*             Pacote de Init do Foguete    */
/*             Dados de Sensores da EEPROM  */
/*     DETECCOES                            */
/*         DETECÇAO DE INICIO DO VOO        */
/*             Pelo BMP                     */
/*             Pelo MPU                     */  // TODO
/*             Pelo LORA                    */
/*         DETECÇAO DE APOGEU               */
/*             Pelo BMP                     */
/*             Pelo MPU                     */  // TODO
/*             Pelo Timer                   */
/*         DETECÇAO DE POUSO                */
/*             Pelo BMP                     */  // REVIEW
/*             Pelo MPU                     */  // TODO
/*             Pelo Timer                   */
/*     USO GERAL                            */
/*         Atualiza a Execucao Atual        */
/*         Atualiza o Tempo de Voo Atual    */
/*         Inicia o Estado                  */
/*                                          */
/*  ======================================  */





/*  ==========================  */
/*        INICIALIZAÇAO:        */

// SERIAL:
void initSerial(){
  Serial.begin(115200);
}

// MPU6050:
bool initMPU(){
  uint8_t countInitMPU = 0;

  PacoteInitFoguete.init_MPU.b = true;
  
  while (!mpu.begin()){
    delay(10);
    countInitMPU++;
    if (countInitMPU >= MAX_COUNT_INIT_MPU){
      PacoteInitFoguete.init_MPU.b = false;
      break;
    }
  }

  mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
  mpu.setGyroRange(MPU_GYRO_RANGE);
  mpu.setFilterBandwidth(MPU_FILTER_BAND);

  return PacoteInitFoguete.init_MPU.b;
}

// BMP280:
bool initBMP(){
  uint8_t countInitBMP = 0;

  PacoteInitFoguete.init_BMP.b = true;
  
  while (!bmp.begin()){
    delay(10);
    countInitBMP++;
    if (countInitBMP >= MAX_COUNT_INIT_BMP){
      PacoteInitFoguete.init_BMP.b = false;
      break;
    }
  }

  /* Datasheet standard configuration for elevator/floor change detection: */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  return PacoteInitFoguete.init_BMP.b;
}

// FLASH EXTERNA:
/*void initFlashExterna(){
  PacoteInitFoguete.init_FLASH.b = true;
}*/

// LORA:
bool initLORA(){
  PacoteInitFoguete.init_LORA.b = true;

  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  
  // Replace the LoRa.begin(---E-) argument with your location's frequency 
  // - 433E6 for Asia
  // - 866E6 for Europe
  // - 915E6 for North America
  while (!LoRa.begin(915E6)) {
    PacoteInitFoguete.init_LORA.b = false;
    beepBuzzerErro();
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // Ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);

  return PacoteInitFoguete.init_LORA.b;
}

// GPS:
bool initGPS(){
  gpsSerial.begin(GPS_BAUD_RATE);
  PacoteInitFoguete.init_GPS.b = true;
  return PacoteInitFoguete.init_GPS.b;
}

// EEPROM INTERNA EP32:
bool initEEPROM(){
  uint8_t countInitEEPROM = 0;

   PacoteInitFoguete.init_EEPROM.b = true;

  while (!SYS_CONTROL_EEPROM.begin(DADOS_EEPROM_SIZE)) {
    delay(100);
    countInitEEPROM++;
    if (countInitEEPROM >= MAX_COUNT_INIT_EEPROM){
      PacoteInitFoguete.init_EEPROM.b = false;
      break;
    }
  }

  while (!DADOS_EEPROM.begin(DADOS_EEPROM_SIZE)) {
    delay(100);
    countInitEEPROM++;
    if (countInitEEPROM >= MAX_COUNT_INIT_EEPROM){
      PacoteInitFoguete.init_EEPROM.b = false;
      break;
    }
  }
  
  return PacoteInitFoguete.init_EEPROM.b;
}

// REMOVE BEFORE FLIGHT (RBF):
void initRBF(){
  pinMode(RBF0, INPUT_PULLUP);
  pinMode(RBF1, INPUT_PULLUP);
}

// BUZZER:
void initBuzzer(){
  ledcSetup(BUZZER_CANAL, BUZZER_FREQUENCIA, BUZZER_RESOLUCAO);
  ledcAttachPin(BUZZER_PINO, BUZZER_CANAL);
  // pinMode(BUZZER, OUTPUT);
  // digitalWrite(BUZZER, LOW);  // desligado
}

// LED:
/*void initBuzzer(){
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);  // desligado
}*/

// ELETROIMA:
void initEletroima(bool eletroima){
  pinMode(ELETROIMA, OUTPUT);
  pinMode(PORTA_IR, OUTPUT);  // LED IR do sensor da porta
  pinMode(PORTA_FT, INPUT);   // Fototransistor do sensor

  if (eletroima == PORTA_EI_FECHADA) digitalWrite(ELETROIMA, HIGH);  // porta fechada
  else digitalWrite(ELETROIMA, LOW);                                 // porta aberta

  digitalWrite(PORTA_IR, LOW);   // sensor desligado
}


/*  ABSTRAÇOES DE INIT:  */

// Init Basico:
void initBasico(bool eletroima){
  initEletroima(eletroima);
  initMPU();
  initBMP();
  // initFlashExterna();
}

// Init Completo:
void initCompleto(bool eletroima){
  initBasico(eletroima);
  initLORA();
  initGPS();
}





/*  ==========================  */
/*       BEEPS E BLINKS:        */

// Beeps simples do buzzer:
void beepBuzzer(int8_t n){
  for (int i=0; i<n; i++){
    ledcWriteTone(BUZZER_CANAL, 1046);
    delay(100);
    ledcWriteTone(BUZZER_CANAL, 0);
    delay(100);
  }
}

// Beep de erro do buzzer:
void beepBuzzerErro(){
  ledcWriteTone(BUZZER_CANAL, 523);
  delay(1000);
  ledcWriteTone(BUZZER_CANAL, 0);
  delay(1000);
}

// Beeps de início de voo:
void beepBuzzerNovoVoo(){
  ledcWriteTone(BUZZER_CANAL, 625);
  delay(300);
  ledcWriteTone(BUZZER_CANAL, 312);
  delay(300);
  ledcWriteTone(BUZZER_CANAL, 416);
  delay(500);
  ledcWriteTone(BUZZER_CANAL, 468);
  delay(780);
  ledcWriteTone(BUZZER_CANAL, 625);
  delay(300);
  ledcWriteTone(BUZZER_CANAL, 468);
  delay(1500);
  ledcWriteTone(BUZZER_CANAL, 0);
}


// Blinks do LED:
/*void blinkLED(int8_t n){
  for (int i=0; i<n; i++){
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }
}*/





/*  ==========================  */
/*          CHECAGENS:          */

// Checar Remove Before Flight:
uint8_t checarRBF(){
  /*  RBF0 = ON  e RBF1 = ON    =>   rbf = 2 (antes de voar)
      RBF0 = ON  e RBF1 = OFF   =>   rbf = 1 (depois de recuperado)
      RBF0 = OFF e RBF1 = ON    =>   rbf = 1 (depois de recuperado)
      RBF0 = OFF e RBF1 = OFF   =>   rbf = 0 (voando/aguardando recuperação) */
  
  uint8_t rbf = 0;

  if (digitalRead(RBF0) == RBF_CHAVE_ON) rbf++;  // chave fechada
  if (digitalRead(RBF1) == RBF_CHAVE_ON) rbf++;  // chave fechada
  
  return rbf;
}

// Checar Porta do Eletroima (aberta ou fechada):
bool checarPorta(){
  bool estadoPorta;

  digitalWrite(PORTA_IR, HIGH);  // sensor ligado
  delayMicroseconds(10);

  if (digitalRead(PORTA_FT) == LOW) estadoPorta = PORTA_EI_FECHADA;  // porta fechada
  else estadoPorta = PORTA_EI_ABERTA;

  digitalWrite(PORTA_IR, LOW);  // sensor ligado

  return estadoPorta;  // porta aberta
}





/*  ==========================  */
/*          ELETROIMA:          */

// Abrir a porta (soltar paraquedas):
void abrirPortaEI(){
  digitalWrite(ELETROIMA, PORTA_EI_ABRIR);
}

// Fechar a porta (travar paraquedas):
void fecharPortaEI(){
  digitalWrite(ELETROIMA, PORTA_EI_FECHAR);
}





/*  ==========================  */
/*         CALIBRAÇOES:         */


/*  CALIBRAÇAO DO BMP:  */

// Atualiza a Pressao Atm Local:
void atualizarPressaoAtmLocal(){
  for (int i=0; i<COUNT_CALIBRAR_PRESSAO_BMP; i++){
    PressaoAtmLocal += bmp.readPressure();
    delay(DELAY_CALIBRAR_PRESSAO_BMP);
  }
  PressaoAtmLocal /= COUNT_CALIBRAR_PRESSAO_BMP;
  PacoteInitFoguete.pressaoAtmLocal.f = PressaoAtmLocal;
}

// Atualiza a Altitude Inicial:
void atualizarAltitudeInicial(){
  for (int i=0; i<COUNT_CALIBRAR_ALTURA_BMP; i++){
    AltitudeInicial += bmp.readAltitude(PressaoAtmLocal);
    delay(DELAY_CALIBRAR_ALTURA_BMP);
  }
  AltitudeInicial /= COUNT_CALIBRAR_ALTURA_BMP;
  PacoteInitFoguete.altitudeInicial.f = AltitudeInicial;
}

// Atualiza a Temperatura Inicial:
void atualizarTemperaturaInicial(){
  for (int i=0; i<COUNT_CALIBRAR_TEMPERATURA_BMP; i++){
    TemperaturaInicial += bmp.readTemperature();
    delay(DELAY_CALIBRAR_TEMPERATURA_BMP);
  }
  TemperaturaInicial /= COUNT_CALIBRAR_TEMPERATURA_BMP;
  PacoteInitFoguete.temperaturaInicial.f = TemperaturaInicial;
}


/*  CALIBRAÇAO DO MPU:  */

// TODO





/*  ==========================  */
/*      LEITURA DE DADOS:       */


/*  EEPROM INTERNA:  */

// Le o ultimo Endereço de Dados da EEPROM interna:
uint8_t leEnderecoDadosEEPROM(){
  return SYS_CONTROL_EEPROM.readUShort(EEPROM_ULTIMO_DADO_ADDR);
}

// Le os Resets Inesperados da EEPROM interna:
uint8_t leResetsInesperadosEEPROM(){
  return SYS_CONTROL_EEPROM.readByte(EEPROM_RESETS_INESPERADOS_ADDR);
}

// Le o Estado anterior da EEPROM interna:
uint8_t leEstadoEEPROM(){
  return SYS_CONTROL_EEPROM.readByte(EEPROM_ESTADO_ATUAL_ADDR);
}

// Le a Execuçao anterior da EEPROM interna:
uint16_t leExecucaoEEPROM(){
  return SYS_CONTROL_EEPROM.readUShort(EEPROM_EXECUCAO_ADDR);
}

// Le o Apogeu anterior da EEPROM interna:
float leApogeuEEPROM(){
  return SYS_CONTROL_EEPROM.readFloat(EEPROM_APOGEU_ADDR);
}

// Le a Aceleraçao Maxima na EEPROM interna:
float leAccelMaxEEPROM(char eixo){
  switch (eixo){
    case 'x': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_X_MAX_ADDR);
    case 'X': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_X_MAX_ADDR);
    case 'y': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_Y_MAX_ADDR);
    case 'Y': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_Y_MAX_ADDR);
    case 'z': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_Z_MAX_ADDR);
    case 'Z': return SYS_CONTROL_EEPROM.readFloat(EEPROM_ACC_Z_MAX_ADDR);
    default: return -1.0;
  }
}

// Le o Tempo Inicial de Voo na EEPROM interna:
unsigned long leTempoInicialEEPROM(){
  /*unsigned long tempoInicialEEPROM = 0;

  for (char i=0; i<4; i++){
    tempoInicialEEPROM |= (DADOS_EEPROM.read(EEPROM_TEMPO_INICIAL_VOO_ADDR + i) << (8*(3-i)));
  }
  return tempoInicialEEPROM;*/
  return SYS_CONTROL_EEPROM.readULong(EEPROM_TEMPO_INICIAL_VOO_ADDR);
}

// Le a Pressao Atmosferica Local anterior da EEPROM interna:
float lePressaoAtmLocalEEPROM(){
  return SYS_CONTROL_EEPROM.readFloat(EEPROM_PRESSAO_LOCAL_ADDR);
}

// Le a Altitude Inicial anterior da EEPROM interna:
float leAltitudeInicialEEPROM(){
  return SYS_CONTROL_EEPROM.readFloat(EEPROM_ALTITUDE_INICIAL_ADDR);
}

// Le a Temperatura Inicial do BMP na EEPROM interna:
float leTemperaturaInicialBmpEEPROM(){
  return SYS_CONTROL_EEPROM.readFloat(EEPROM_TEMP_INICIAL_BMP_ADDR);
}

// Le a Temperatura Inicial do MPU na EEPROM interna:
float leTemperaturaInicialMpuEEPROM(){
  return SYS_CONTROL_EEPROM.readFloat(EEPROM_TEMP_INICIAL_MPU_ADDR);
}


/*  SENSORES:  */

// MPU6050:
void mpuRead(){
  /*  MPU9250:  */
  /*
  mpu.accelUpdate(); //Atualiza o acelerometro
  PacoteDadosFoguete.mpu_accX.i16 = mpu.accelX(); //var.i16 projX = valor lido do mpu
  PacoteDadosFoguete.mpu_accY.i16 = mpu.accelY(); //var.i16 projX = valor lido do mpu
  PacoteDadosFoguete.mpu_accZ.i16 = mpu.accelZ(); //var.i16 projX = valor lido do mpu
  mpu.gyroUpdate(); //Atualiza o giroscopio
  PacoteDadosFoguete.mpu_gyrX.i16 = mpu.gyroX(); //var.i16 projX = valor lido do mpu
  PacoteDadosFoguete.mpu_gyrY.i16 = mpu.gyroY(); //var.i16 projX = valor lido do mpu
  PacoteDadosFoguete.mpu_gyrZ.i16 = mpu.gyroZ(); //var.i16 projX = valor lido do mpu
  */


  /*  MPU6050:  */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Le os dados do Pacote de Dados:
  PacoteDadosFoguete.mpu_accX.i16 = (int16_t) (a.acceleration.x * MPU_ACCEL_SCALE);  // Inverter a conta para obter em m/s2
  PacoteDadosFoguete.mpu_accY.i16 = (int16_t) (a.acceleration.y * MPU_ACCEL_SCALE);  // Inverter a conta para obter em m/s2
  PacoteDadosFoguete.mpu_accZ.i16 = (int16_t) (a.acceleration.z * MPU_ACCEL_SCALE);  // Inverter a conta para obter em m/s2
  
  PacoteDadosFoguete.mpu_temp.i16 = (int16_t) ((temp.temperature - 36.53) * 340.0);  // Inverter a conta para obter em *C
  
  PacoteDadosFoguete.mpu_gyrX.i16 = (int16_t) (g.gyro.x * MPU_GYRO_SCALE);  // Inverter a conta para obter em deg/s
  PacoteDadosFoguete.mpu_gyrY.i16 = (int16_t) (g.gyro.y * MPU_GYRO_SCALE);  // Inverter a conta para obter em deg/s
  PacoteDadosFoguete.mpu_gyrZ.i16 = (int16_t) (g.gyro.z * MPU_GYRO_SCALE);  // Inverter a conta para obter em deg/s
  
  

  // Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  // Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_OUT, 14);

  // uint8_t buffer[14];
  // data_reg.read(buffer, 14);

  // PacoteDadosFoguete.mpu_accX.i16 = buffer[0]  << 8 | buffer[1];
  // PacoteDadosFoguete.mpu_accY.i16 = buffer[2]  << 8 | buffer[3];
  // PacoteDadosFoguete.mpu_accZ.i16 = buffer[4]  << 8 | buffer[5];

  // PacoteDadosFoguete.mpu_temp.i16 = buffer[6]  << 8 | buffer[7];

  // PacoteDadosFoguete.mpu_gyrX.i16 = buffer[8]  << 8 | buffer[9];
  // PacoteDadosFoguete.mpu_gyrY.i16 = buffer[10] << 8 | buffer[11];
  // PacoteDadosFoguete.mpu_gyrZ.i16 = buffer[12] << 8 | buffer[13];

  // temperature = (rawTemp / 340.0) + 36.53;
}

// BMP280:
void bmpRead(){
  // Le os dados do Pacote de Dados:s
  PacoteDadosFoguete.bmp_altura.f = bmp.readAltitude(PressaoAtmLocal); //var.f projX = valor lido do bmp
  PacoteDadosFoguete.bmp_temp.f = bmp.readTemperature(); //var.f projX = valor lido do bmp
}

// GPS:
void gpsRead(){
  // Le os dados do Pacote de Dados:
  PacoteDadosFoguete.gps_lat.f = gps.location.lat(); //var.f projX = valor lido do gps
  PacoteDadosFoguete.gps_lon.f = gps.location.lng(); //var.f projX = valor lido do gps
}





/*  ==========================  */
/*   ARMAZENAMENTO DE DADOS:    */


/*  EEPROM INTERNA - SISTEMA:  */

// Salva o ultimo Endereço de Dados na EEPROM interna:
void salvaEnderecoDadosEEPROM(uint8_t enderecoDadosEEPROM){
  SYS_CONTROL_EEPROM.writeUShort(EEPROM_ULTIMO_DADO_ADDR, enderecoDadosEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva os Resets Inesperados na EEPROM interna:
void salvaResetsInesperadosEEPROM(uint8_t resetsInesperadosEEPROM){
  SYS_CONTROL_EEPROM.writeByte(EEPROM_RESETS_INESPERADOS_ADDR, resetsInesperadosEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva o Estado Atual na EEPROM interna:
void salvaEstadoEEPROM(uint8_t estadoAtualEEPROM, uint16_t execucaoEEPROM){
  SYS_CONTROL_EEPROM.writeByte(EEPROM_ESTADO_ATUAL_ADDR, estadoAtualEEPROM);
  SYS_CONTROL_EEPROM.writeUShort(EEPROM_EXECUCAO_ADDR, execucaoEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva a Execuçao Atual na EEPROM interna:
void salvaExecucaoEEPROM(uint16_t execucaoEEPROM){
  /*DADOS_EEPROM.write(EEPROM_EXECUCAO_ADDR, ((execucaoEEPROM & 0xFF00) >> 8));
  DADOS_EEPROM.write((EEPROM_EXECUCAO_ADDR + 1), (execucaoEEPROM & 0x00FF));*/
  SYS_CONTROL_EEPROM.writeUShort(EEPROM_EXECUCAO_ADDR, execucaoEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva o Apogeu na EEPROM interna:
void salvaApogeuEEPROM(float apogeuEEPROM){
  SYS_CONTROL_EEPROM.writeFloat(EEPROM_APOGEU_ADDR, apogeuEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva as Aceleraçoes Maximas na EEPROM interna:
void salvaAccelMaxEEPROM(int16_t accXEEPROM, int16_t accYEEPROM, int16_t accZEEPROM){
  SYS_CONTROL_EEPROM.writeShort(EEPROM_ACC_X_MAX_ADDR, accXEEPROM);
  SYS_CONTROL_EEPROM.writeShort(EEPROM_ACC_Y_MAX_ADDR, accYEEPROM);
  SYS_CONTROL_EEPROM.writeShort(EEPROM_ACC_Z_MAX_ADDR, accZEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva o Tempo Inicial de Voo na EEPROM interna:
void salvaTempoInicialEEPROM(unsigned long tempoInicialEEPROM){
  /*for (char i=0; i<4; i++){
    DADOS_EEPROM.write((EEPROM_TEMPO_INICIAL_VOO_ADDR + i), (tempoInicialEEPROM & (0xFF << (8*(3-i)))));
  }*/
  SYS_CONTROL_EEPROM.writeULong(EEPROM_TEMPO_INICIAL_VOO_ADDR, tempoInicialEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva a Pressao Atmosferica Local na EEPROM interna:
void salvaPressaoAtmLocalEEPROM(float pressaoAtmLocalEEPROM){
  SYS_CONTROL_EEPROM.writeFloat(EEPROM_PRESSAO_LOCAL_ADDR, pressaoAtmLocalEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva a Altitude Inicial na EEPROM interna:
void salvaAltitudeInicialEEPROM(float altitudeInicialEEPROM){
  SYS_CONTROL_EEPROM.writeFloat(EEPROM_ALTITUDE_INICIAL_ADDR, altitudeInicialEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva a Temperatura Inicial do BMP na EEPROM interna:
void salvaTemperaturaInicialBmpEEPROM(float tempInicialBmpEEPROM){
  SYS_CONTROL_EEPROM.writeFloat(EEPROM_TEMP_INICIAL_BMP_ADDR, tempInicialBmpEEPROM);
  SYS_CONTROL_EEPROM.commit();
}

// Salva a Temperatura Inicial do MPU na EEPROM interna:
void salvaTemperaturaInicialMpuEEPROM(int16_t tempInicialMpuEEPROM){
  SYS_CONTROL_EEPROM.writeShort(EEPROM_TEMP_INICIAL_MPU_ADDR, tempInicialMpuEEPROM);
  SYS_CONTROL_EEPROM.commit();
}


/*  EEPROM INTERNA - DADOS:  */

// Salva os Dados dos Sensores na EEPROM interna:
bool salvaDadosEEPROM(bool salvaTVoo, bool salvaBmp, bool salvaMpu, bool salvaGps){
  if (EnderecoDadosEEPROM > DADOS_EEPROM_ADDR_MAX) return false;

  // Tempo de Voo:
  if (salvaTVoo){
    DADOS_EEPROM.writeULong(EnderecoDadosEEPROM, PacoteDadosFoguete.tempoVoo.ul);
    EnderecoDadosEEPROM += EEPROM_OFFSET_TEMPO_VOO;
  }

  // BMP:
  if (salvaBmp){
    DADOS_EEPROM.writeFloat(EnderecoDadosEEPROM, PacoteDadosFoguete.bmp_altura.f);
    EnderecoDadosEEPROM += EEPROM_OFFSET_BMP_ALTURA;
    DADOS_EEPROM.writeFloat(EnderecoDadosEEPROM, PacoteDadosFoguete.bmp_temp.f);
    EnderecoDadosEEPROM += EEPROM_OFFSET_BMP_TEMPERATURA;
  }

  // MPU:
  if (salvaMpu){
    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_accX.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_ACC_X;
    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_accY.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_ACC_Y;
    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_accZ.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_ACC_Z;

    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_temp.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_TEMP;

    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_gyrX.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_GYR_X;
    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_gyrX.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_GYR_Y;
    DADOS_EEPROM.writeShort(EnderecoDadosEEPROM, PacoteDadosFoguete.mpu_gyrX.i16);
    EnderecoDadosEEPROM += EEPROM_OFFSET_MPU_GYR_Z;
  }

  // GPS:
  if (salvaGps){
    DADOS_EEPROM.writeFloat(EnderecoDadosEEPROM, PacoteDadosFoguete.gps_lat.f);
    EnderecoDadosEEPROM += EEPROM_OFFSET_GPS_LAT;
    DADOS_EEPROM.writeFloat(EnderecoDadosEEPROM, PacoteDadosFoguete.gps_lon.f);
    EnderecoDadosEEPROM += EEPROM_OFFSET_GPS_LON;
  }

  if (salvaTVoo || salvaBmp || salvaMpu || salvaGps){
    DADOS_EEPROM.commit();
  }
  
  return true;
}





/*  ==========================  */
/*    COMUNICAÇAO DE DADOS:     */


/*  LORA:  */

// Transmite Dados por LoRa:
void transmiteDadosLORA(unsigned char* packet){
  /*unsigned char* packet;
  packet = DADOSFOGUETE.tratarPacoteDados_enviar();  //Funcao do ProjetoX
  */

  LoRa.beginPacket();  // Inicia a coleta do pacote
  for(char i=0; i<TAM_PACOTE; i++){
    LoRa.print(packet[i]);
  }
  LoRa.endPacket();  // Finaliza o pacote
}

// Recebe Dados por LoRa:
bool recebeDadosLORA(char tipoPacote){
  unsigned char* packet;
  bool flagRecebeuOK = false;
  char i = 0;

  if (LoRa.parsePacket()){
    while (LoRa.available()){
      packet[i] = LoRa.read();
      i++;
    }
    switch (tipoPacote){
      case TIPO_DFG:
        if (PacoteDadosFoguete.tratarPacoteDados_receber(packet)) flagRecebeuOK = true;
        break;
      case TIPO_CMD:
        if (PacoteComando.tratarPacoteDados_receber(packet)) flagRecebeuOK = true;
        break;
      case TIPO_INF:
        if (PacoteInitFoguete.tratarPacoteDados_receber(packet)) flagRecebeuOK = true;
        break;
      default:
        flagRecebeuOK = false;
        break;
    }
  }
  return flagRecebeuOK;
}


/*  SERIAL:  */

// Printa o Pacote de Inicializaçao do Foguete na Serial:
void printaPacoteInitFogueteSerial(){
  unsigned char* packet;
  packet = PacoteInitFoguete.tratarPacoteDados_enviar();

  Serial.println("==================================================");
  Serial.println("Inicializaçao do Foguete:");
  Serial.print(PacoteInitFoguete.init_BMP.b); Serial.print(";");
  Serial.print(PacoteInitFoguete.init_MPU.b); Serial.print(";");
  Serial.print(PacoteInitFoguete.init_GPS.b); Serial.print(";");
  Serial.print(PacoteInitFoguete.init_LORA.b); Serial.print(";");
  Serial.print(PacoteInitFoguete.init_EEPROM.b); Serial.print(";");

  Serial.print(PacoteInitFoguete.estadoAtual.ui8); Serial.print(";");
  Serial.print(PacoteInitFoguete.execucaoAtual.ui16); Serial.print(";");
  Serial.print(PacoteInitFoguete.tempoInicialVoo.ul); Serial.print(";");
  Serial.print(PacoteInitFoguete.pressaoAtmLocal.f); Serial.print(";");
  Serial.print(PacoteInitFoguete.altitudeInicial.f); Serial.println(";");
  Serial.print(PacoteInitFoguete.temperaturaInicial.f); Serial.println();

  for (int i=0; i<TAM_PACOTE; i++) Serial.print(packet[i]);
  Serial.println();
  
  Serial.println("==================================================");
}

// Printa os Dados de Sensores da EEPROM na Serial:
void printaDadosSensoresSerial(){
  unsigned char* packet;

  Serial.println("==================================================");
  Serial.println("Dados do Foguete:");
  
  Serial.println();
  Serial.println("-----------------");
  Serial.print("Ultimo Endereço de Dados: "); Serial.println(leEnderecoDadosEEPROM());
  Serial.println();
  Serial.print("Resets Inesperados:       "); Serial.println(leResetsInesperadosEEPROM());
  Serial.println();
  Serial.print("Apogeu:                   "); Serial.println(leApogeuEEPROM());
  Serial.println();
  Serial.print("Aceleraçao Max X:         "); Serial.println(leAccelMaxEEPROM('x'));
  Serial.print("Aceleraçao Max Y:         "); Serial.println(leAccelMaxEEPROM('y'));
  Serial.print("Aceleraçao Max Z:         "); Serial.println(leAccelMaxEEPROM('z'));
  Serial.println();
  Serial.print("Tempo Inicial de Voo:     "); Serial.println(leTempoInicialEEPROM());
  Serial.println();
  Serial.print("Pressao Local:            "); Serial.println(lePressaoAtmLocalEEPROM());
  Serial.println();
  Serial.print("Altura Inicial:           "); Serial.println(leAltitudeInicialEEPROM());
  Serial.println();
  Serial.print("Temperatura Inicial BMP:  "); Serial.println(leTemperaturaInicialBmpEEPROM());
  Serial.println();
  Serial.print("Temperatura Inicial MPU:  "); Serial.println(leTemperaturaInicialMpuEEPROM());
  Serial.println();

  uint16_t addr = 0;

  for (uint16_t i=0; i<DADOS_EEPROM_SIZE; i++){
    if ((i % DADOS_EEPROM_OFFSET_TOTAL) == 0){
      addr = i;

      packet = PacoteDadosFoguete.tratarPacoteDados_enviar();
      
      Serial.println();
      Serial.println("-----------------");
      Serial.print(DADOS_EEPROM.readULong(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_TEMPO_VOO;

      Serial.print(DADOS_EEPROM.readFloat(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_BMP_ALTURA;
      Serial.print(DADOS_EEPROM.readFloat(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_BMP_TEMPERATURA;

      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_ACC_X;
      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_ACC_Y;
      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_ACC_Z;

      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_GYR_X;
      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_GYR_Y;
      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_GYR_Z;

      Serial.print(DADOS_EEPROM.readShort(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_MPU_TEMP;
      
      Serial.print(DADOS_EEPROM.readFloat(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_GPS_LAT;
      Serial.print(DADOS_EEPROM.readFloat(addr)); Serial.print(";");
      addr += EEPROM_OFFSET_GPS_LON;

      Serial.println();

      for (int j=0; j<TAM_PACOTE; j++) Serial.print(packet[j]);
      Serial.println();
    }
  }

  Serial.println();
  Serial.println("==================================================");
}





/*  ==========================  */
/*          DETECCOES:          */


/*  DETECÇAO DE INICIO DO VOO:  */

// Detecçao de inicio do voo do BMP:
bool detectaInicioVooBMP(uint16_t countBMP, float altAntBMP){
  if (PacoteDadosFoguete.bmp_altura.f > altAntBMP &&
      PacoteDadosFoguete.bmp_altura.f > (AltitudeInicial + OFFSET_DETEC_INICIO_VOO_BMP))
        countBMP++; // Compara leitura atual com a anterior
  else countBMP = 0;

  altAntBMP = PacoteDadosFoguete.bmp_altura.f;  // Anterior = Atual para prox comparacao

  if(countBMP >= MAX_COUNT_DETEC_INICIO_VOO_BMP) return INICIO_VOO_DETECTADO_BMP;
  else return INICIO_VOO_NAO_DETECTADO_BMP;
}

// Detecçao de inicio do voo do MPU:
/*bool detectaInicioVooMPU(){
  //detecao por MPU

  return APOGEU_DETECTADO_MPU;
}*/

// Detecçao de inicio do voo do LORA:
bool detectaInicioVooLORA(){
  if (recebeDadosLORA(TIPO_CMD)){
    for (char i=0; i<TAM_COMANDO; i++){
      if (PacoteComando.comando[i].c != COMANDO_IGNIT[i]) return false;
    }
    return true;
  }
  return false;
}


/*  DETECÇAO DE APOGEU:  */

// Detecçao de apogeu do BMP:
bool detectaApogeuBMP(uint16_t countBMP, float altAntBMP){
  if (PacoteDadosFoguete.bmp_altura.f < altAntBMP) countBMP++; // Compara leitura atual com a anterior
  else countBMP = 0;

  altAntBMP = PacoteDadosFoguete.bmp_altura.f;  // Anterior = Atual para prox comparacao

  if (countBMP >= MAX_COUNT_DETEC_APOGEU_BMP) return APOGEU_DETECTADO_BMP;
  else return APOGEU_NAO_DETECTADO_BMP;
}

// Detecçao de apogeu do MPU:
/*bool detectaApogeuMPU(){
  //detecao por MPU

  return APOGEU_DETECTADO_MPU;
}*/

// Detecçao de apogeu do Timer:
bool detectaApogeuTimer(){
  if (millis() - TempoInicialVoo >= MAX_COUNT_DETEC_APOGEU_TIMER) return APOGEU_DETECTADO_TIMER;
  else return APOGEU_NAO_DETECTADO_TIMER;
}


/*  DETECÇAO DE POUSO:  */

// Detecçao de pouso do BMP:
bool detectaPousoBMP(uint16_t countBMP, float altAntBMP){  // REVIEW
  if (PacoteDadosFoguete.bmp_altura.f >= (altAntBMP - INTERVALO_DETEC_POUSO_BMP)) countBMP++; // Compara leitura atual com a anterior
  else if (PacoteDadosFoguete.bmp_altura.f <= (altAntBMP + INTERVALO_DETEC_POUSO_BMP)) countBMP++; // Compara leitura atual com a anterior
  else countBMP = 0;

  altAntBMP = PacoteDadosFoguete.bmp_altura.f;  // Anterior = Atual para prox comparacao

  if (countBMP >= MAX_COUNT_DETEC_POUSO_BMP) return POUSO_DETECTADO_BMP;
  else return POUSO_NAO_DETECTADO_BMP;
}

// Detecçao de pouso do MPU:
/*bool detectaPousoMPU(){
  // detecao por MPU - Aceleracao nos eixos X e Y < 1;
  // TODO
  return POUSO_DETECTADO_MPU;
}*/

// Detecçao de pouso do Timer:
bool detectaPousoTimer(){
  if (millis() - TempoInicialVoo >= MAX_COUNT_DETEC_POUSO_TIMER) return POUSO_DETECTADO_TIMER;
  else return POUSO_NAO_DETECTADO_TIMER;
}





/*  ==========================  */
/*          USO GERAL:          */

// Atualiza a Execucao Atual:
void atualizaExecucaoAtual(){
  salvaExecucaoEEPROM(ExecucaoAtual++);
  PacoteInitFoguete.execucaoAtual.ui16 = ExecucaoAtual;
  PacoteDadosFoguete.execucaoAtual.ui16 = ExecucaoAtual;
}

// Atualiza o Tempo de Voo Atual:
void atualizaTempoVooAtual(){
  PacoteDadosFoguete.tempoVoo.ul = millis();
}

// Atualiza o Estado Atual na EEPROM e nos Pacotes
void iniciaEstado(uint8_t estado){
  EstadoAtual = estado;
  ExecucaoAtual = 0;
  salvaEstadoEEPROM(EstadoAtual, ExecucaoAtual);
  PacoteInitFoguete.estadoAtual.ui8 = EstadoAtual;
  PacoteDadosFoguete.estadoAtual.ui8 = EstadoAtual;
}