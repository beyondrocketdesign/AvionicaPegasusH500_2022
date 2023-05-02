/*
  flash/eeprom interna:
    0x00 - EstadoAtual: número do estado sendo executado (ex: S1 -> 1)
    0x02 - ExecucaoAtual: número da execução atual do loop do estado
            • 0 -> antes de entrar no loop ou não há loop no estado atual
            • 1 -> primeira execução
            • 2 -> segunda execução
            • etc
    0x04 - extFlashADDR: endereço da Flash externa em que o próximo dado deve ser salvo
  
  Ex:
       executando  [...]  S0    S1    S2    S2    S2    S2    S3    S3    S3  [...]
      EstadoAtual  [...]   0     1     2     2     2     2     3     3     3  [...]
    ExecucaoAtual  [...]   0     0     0     1     2     3     0     1     2  [...]
*/

#include "sysConfig.h"
//#include "sysFunctions.ino"
//#include "AvionicaPegasus_main.ino"





/*  ==========================  */
/*              S0              */
/*           Power Up           */

uint8_t estadoS0(){
  initRBF();
  initBuzzer();

  initEEPROM();
  EnderecoDadosEEPROM = SYS_CONTROL_EEPROM.readUShort(EEPROM_ULTIMO_DADO_ADDR);
  EstadoAtual = SYS_CONTROL_EEPROM.readByte(EEPROM_ESTADO_ATUAL_ADDR);
  ResetsInesperados = SYS_CONTROL_EEPROM.readByte(EEPROM_RESETS_INESPERADOS_ADDR);

  if (EstadoAtual == 0){
    salvaEstadoEEPROM(0,0);  // EstadoAtual = 0 e ExecucaoAtual = 0 (sem loop)
    beepBuzzerNovoVoo();
    ResetsInesperados = 0;
    salvaResetsInesperadosEEPROM(ResetsInesperados);
    EnderecoDadosEEPROM = 0;
    salvaEnderecoDadosEEPROM(EnderecoDadosEEPROM);
  }
  else if (EstadoAtual == 8){  // Foguete Recuperado
    switch (checarRBF()){
      case RBF_ANTES_DO_VOO:     // Foguete já foi recuperado e este é um novo voo (reset)
        salvaEstadoEEPROM(0,0);  // EstadoAtual = 0 e ExecucaoAtual = 0 (sem loop)
        beepBuzzerNovoVoo();
        ResetsInesperados = 0;
        salvaResetsInesperadosEEPROM(ResetsInesperados);
        EnderecoDadosEEPROM = 0;
        salvaEnderecoDadosEEPROM(EnderecoDadosEEPROM);
        break;
      case RBF_RECUPERADO:       // Foguete já foi recuperado mas os dados ainda nao foram lidos
        salvaEstadoEEPROM(8,0);  // EstadoAtual = 8 e ExecucaoAtual = 0 (inicio)
        beepBuzzer(BEEP_ESTADO_S8);
        break;
      case RBF_AGUARDANDO_REC:   // Foguete ainda nao foi recuperado e os dados ainda nao foram lidos
        salvaEstadoEEPROM(7,0);  // EstadoAtual = 7 e ExecucaoAtual = 0 (inicio)
        salvaResetsInesperadosEEPROM(ResetsInesperados++);
        EnderecoDadosEEPROM = DADOS_EEPROM_ADDR_MAX;
        salvaEnderecoDadosEEPROM(EnderecoDadosEEPROM);
        break;
      default:                   // Na duvida, começa do zero
        salvaEstadoEEPROM(0,0);  // EstadoAtual = 0 e ExecucaoAtual = 0 (sem loop)
        beepBuzzerNovoVoo();
        break;
    }
  }
  else salvaResetsInesperadosEEPROM(ResetsInesperados++);
  
  return ESTADO_S1;  // executar estadoS1()
}





/*  ==========================  */
/*              S1              */
/*        Inicialização         */

uint8_t estadoS1(){
  EstadoAtual = SYS_CONTROL_EEPROM.readByte(EEPROM_ESTADO_ATUAL_ADDR);

  switch (EstadoAtual){
    case 0:  // S0 (Power Up) - andamento correto se for um novo voo
        salvaEstadoEEPROM(1,0);  // EstadoAtual = 1 e ExecucaoAtual = 0 (sem loop)
        beepBuzzer(BEEP_ESTADO_S1);
        initCompleto(PORTA_EI_ABERTA);
        return ESTADO_S1;  // executar estadoS1()
    case 1:  // S1 (Inicialização)
        initCompleto(PORTA_EI_ABERTA);
        return ESTADO_S1;  // executar estadoS1()
    case 2:  // S2 (Remove Before Flight)
        initCompleto(PORTA_EI_ABERTA);
        return ESTADO_S2;  // executar estadoS2()
    case 3:  // S3 (Waiting Flight Start)
        initBasico(PORTA_EI_FECHADA);
        return ESTADO_S3;  // executar estadoS3()
    case 4:  // S4 (Subindo)
        initBasico(PORTA_EI_FECHADA);
        if (!checarPorta()){  // porta aberta antes do tempo
          fecharPortaEI();
          delay(10);

          if (!checarPorta()){
            abrirPortaEI();
            EstadoAtual = 6;
          }
          
          /* ideias:
              - ligar o eletroima e checar a porta novamente
              - desligar o eletroima pra economizar bateria, se a porta continuar aberta
              - pular para o estado S6 pq se não pode ser difícil detectar o pouso
          */
        }
        return ESTADO_S4;  // executar estadoS4()
    case 5:  // S5 (Apogeu)
        initBasico(PORTA_EI_ABERTA);
        return ESTADO_S5;  // executar estadoS5()
    case 6:  // S6 (Descendo)
        initBasico(PORTA_EI_ABERTA);
        return ESTADO_S6;  // executar estadoS6()
    case 7:  // S7 (Aguardando Recuperação)
        initGPS();
        initLORA();
        return ESTADO_S7;  // executar estadoS7()
    case 8:  // S8 (Foguete Recuperado)
        return ESTADO_S8;  // executar estadoS8()
    default: // fail safe (estado indefinido)
        while(1);
        return ESTADO_S1;  // executar estadoS1()
  }
}





/*  ==========================  */
/*              S2              */
/*     Remove Before Flight     */

uint8_t estadoS2(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(2);

  bool flagRBF = false;
  unsigned long tempoEnviarInit = millis();

  while (!flagRBF){  // Enquanto o RBF nao foi removido
    // Transmite o estado atual para as Bases:
    if (millis() - tempoEnviarInit >= TEMPO_TRANSMITIR_INIT){
      transmiteDadosLORA(PacoteInitFoguete.tratarPacoteDados_enviar());
      tempoEnviarInit = millis();
    }

    if (checarRBF() != RBF_ANTES_DO_VOO) {
      delay(5000);
      if (checarRBF() == RBF_VOANDO){
        flagRBF = true;
        beepBuzzer(BEEP_ESTADO_S2);
      }
      else flagRBF = false;
    }
    else flagRBF = false;
  }

  return ESTADO_S3;  // executar estadoS3()
}





/*  ==========================  */
/*              S3              */
/*     Waiting Flight Start     */

uint8_t estadoS3(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(3);

  bool flagComecarVoo = false;
  bool flagSalvarGPS = false;
  uint16_t countBMP = 0;
  float altAntBMP = 0.0;
  unsigned long tempoSalvarDadosEEPROM = millis();
  unsigned long tempoAtualizarPressao = millis();
  unsigned long tempoLerGPS = millis();

  atualizarPressaoAtmLocal();
  atualizarAltitudeInicial();
  atualizarTemperaturaInicial();
  transmiteDadosLORA(PacoteInitFoguete.tratarPacoteDados_enviar());

  while(!flagComecarVoo){
    // Atualiza Execucao Atual:
    atualizaExecucaoAtual();

    // Atualiza a Pressao Atm Local e a Altura Inicial:
    if (millis() - tempoAtualizarPressao >= TEMPO_ATUALIZAR_PRESSAO){
      transmiteDadosLORA(PacoteInitFoguete.tratarPacoteDados_enviar());
      atualizarPressaoAtmLocal();
      atualizarAltitudeInicial();
      atualizarTemperaturaInicial();
      transmiteDadosLORA(PacoteInitFoguete.tratarPacoteDados_enviar());
      tempoAtualizarPressao = millis();
    }

    // Leitura, armazenamento e transmissao de dados:
    if (millis() - tempoSalvarDadosEEPROM >= TEMPO_SALVAR_DADOS_S3){
      // Leitura dos dados:
      bmpRead();
      // mpuRead();
      if (millis() - tempoLerGPS >= TEMPO_LER_GPS_S3){
        gpsRead();
        flagSalvarGPS = SALVAR_DADOS_GPS;
        tempoLerGPS = millis();
      }

      // Armazenamento dos dados (EEPROM interna):
      salvaDadosEEPROM(SALVAR_DADOS_TEMPO_VOO,
                       SALVAR_DADOS_BMP,
                       SALVAR_DADOS_MPU,
                       flagSalvarGPS);

      if (flagSalvarGPS) flagSalvarGPS = NAO_SALVAR_DADOS_GPS;

      // Transmissao dos dados:
      transmiteDadosLORA(PacoteDadosFoguete.tratarPacoteDados_enviar());

      tempoSalvarDadosEEPROM = millis();
    }

    // Detecçao de inicio do voo:
    if (detectaInicioVooBMP(countBMP, altAntBMP)) flagComecarVoo = true;
    /*if (detectaInicioVooMPU()) flagComecarVoo = true;*/
    if (detectaInicioVooLORA()) flagComecarVoo = true;
  }

  TempoInicialVoo = millis();
  PacoteInitFoguete.tempoInicialVoo.ul = TempoInicialVoo;
  transmiteDadosLORA(PacoteInitFoguete.tratarPacoteDados_enviar());

  return ESTADO_S4;  // executar estadoS4()
}





/*  ==========================  */
/*              S4              */
/*           Subindo            */

uint8_t estadoS4(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(4);

  bool apogeu = false;
  bool flagSalvarDados = SALVAR_DADOS;
  bool flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
  uint16_t countBMP = 0;
  float altAntBMP = 0.0;
  unsigned long tempoLerGPS = millis();

  while (!apogeu){
    // Atualiza Execucao Atual:
    atualizaExecucaoAtual();

    // Leitura de dados:
    atualizaTempoVooAtual();
    bmpRead();
    // mpuRead();
    if (millis() - tempoLerGPS >= TEMPO_LER_GPS_VOO){
      gpsRead();
      flagSalvarGPS = SALVAR_DADOS_GPS;
      tempoLerGPS = millis();
    }

    // Armazenamento de dados (EEPROM interna):
    if (salvaDadosEEPROM(SALVAR_DADOS_TEMPO_VOO,
                          SALVAR_DADOS_BMP,
                          SALVAR_DADOS_MPU,
                          flagSalvarGPS)){
      flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
    }

    // Transmissao de dados:
    transmiteDadosLORA(PacoteDadosFoguete.tratarPacoteDados_enviar());

    // Detecçao de apogeu:
    if (detectaApogeuBMP(countBMP, altAntBMP)) apogeu = true;
    /*if (detectaApogeuMPU()) apogeu = true;*/
    if (detectaApogeuTimer()) apogeu = true;
  }

  return ESTADO_S5;  // executar estadoS5()
}





/*  ==========================  */
/*              S5              */
/*            Apogeu            */

uint8_t estadoS5() {
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(5);

  bool portaAberta = false;
  bool flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
  uint8_t countAbrirPorta = 0;
  unsigned long tempoLerGPS = millis();

  do{
    // Atualiza Execucao Atual:
    atualizaExecucaoAtual();

    abrirPortaEI();

    countAbrirPorta++;
    if (countAbrirPorta >= MAX_COUNT_ABRIR_PORTA) portaAberta = true;  // Talvez trocar para break

    if (checarPorta() == PORTA_EI_ABERTA) portaAberta = true;
    
    // Ideias:
    //  - salvar string Tentei abrir - flash.writeStr(strAddr, "Tentei abrir");
    //  - salva e envia dados dos sensores delay(10)
    //  - servo empurra a porta

    // Leitura de dados:
    atualizaTempoVooAtual();
    bmpRead();
    // mpuRead();
    if (millis() - tempoLerGPS >= TEMPO_LER_GPS_VOO){
      gpsRead();
      flagSalvarGPS = SALVAR_DADOS_GPS;
      tempoLerGPS = millis();
    }

    // Armazenamento de dados (Flash Externa):
    if (salvaDadosEEPROM(SALVAR_DADOS_TEMPO_VOO,
                          SALVAR_DADOS_BMP,
                          SALVAR_DADOS_MPU,
                          flagSalvarGPS)){
      flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
    }

    // Transmissao de dados:
    transmiteDadosLORA(PacoteDadosFoguete.tratarPacoteDados_enviar());

  } while (!portaAberta);
  //salva que abriu - flash.writeStr(strAddr, "Consegui abrir");

  return ESTADO_S6;  // executar estadoS6()
}





/*  ==========================  */
/*              S6              */
/*           Descendo           */

uint8_t estadoS6(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(6);

  bool pouso = false;
  bool flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
  uint8_t countBMP = 0;
  float altAntBMP = 0;
  unsigned long tempoLerDados = millis();
  unsigned long tempoLerGPS = millis();
  
  while (!pouso){ //Troca o estado apenas quando detectar o apogeu
    // Atualiza Execucao Atual:
    atualizaExecucaoAtual();

    // Leitura, armazenamento e transmissao de dados:
    if (millis() - tempoLerDados >= TEMPO_SALVAR_DADOS_S6){
      // Leitura de dados:
      atualizaTempoVooAtual();
      bmpRead();
      // mpuRead();
      if (millis() - tempoLerGPS >= TEMPO_LER_GPS_VOO){
        gpsRead();
        flagSalvarGPS = SALVAR_DADOS_GPS;
        tempoLerGPS = millis();
      }

      // Armazenamento de dados (Flash Externa):
      if (salvaDadosEEPROM(SALVAR_DADOS_TEMPO_VOO,
                            SALVAR_DADOS_BMP,
                            SALVAR_DADOS_MPU,
                            flagSalvarGPS)){
        flagSalvarGPS = NAO_SALVAR_DADOS_GPS;
      }

      // Transmissao de dados:
      transmiteDadosLORA(PacoteDadosFoguete.tratarPacoteDados_enviar());
      tempoLerDados = millis();
    }
    

    // Detecçao de pouso:
    if (detectaPousoBMP(countBMP, altAntBMP)) pouso = true;
    /*if (detectaPousoMPU()) pouso = true;*/
    if (detectaPousoTimer()) pouso = true;
  }

  return ESTADO_S7;  // executar estadoS7()
}





/*  ==========================  */
/*              S7              */
/*    Aguardando Recuperação    */

uint8_t estadoS7(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(7);

  uint8_t countGPS = 0;
  unsigned long tempoTransmitirGPS = millis();

  while(checarRBF() == RBF_AGUARDANDO_REC){
    if (millis() - tempoTransmitirGPS >= TEMPO_LER_GPS_S7){
      // Atualiza Execucao Atual:
      atualizaExecucaoAtual();

      // Leitura de dados:
      atualizaTempoVooAtual();
      // bmpRead();
      // mpuRead();
      gpsRead();

      countGPS++;
      if (countGPS <= 10){
        if (!salvaDadosEEPROM(SALVAR_DADOS_TEMPO_VOO,
                             NAO_SALVAR_DADOS_BMP,
                             NAO_SALVAR_DADOS_MPU,
                             SALVAR_DADOS_GPS)) countGPS = 11;
      }

      transmiteDadosLORA(PacoteDadosFoguete.tratarPacoteDados_enviar());
    }
  }

  return ESTADO_S8;  // executar estadoS8()
}





/*  ==========================  */
/*              S8              */
/*      Foguete Recuperado      */

uint8_t estadoS8(){
  // Atualiza o Estado e Execucao Atual na EEPROM interna e nos Pacotes de Dados:
  iniciaEstado(8);

  delay(5*60000);

  unsigned long tempoPrintarDados = millis();
  uint8_t countRBF = 0;

  do{
    // Atualiza Execucao Atual:
    atualizaExecucaoAtual();

    // Checa RBF:
    if (checarRBF() == RBF_ANTES_DO_VOO){
      countRBF++;
      delay(1000);
      beepBuzzer(BEEP_ESTADO_S8);
    }
    else if (checarRBF() == RBF_RECUPERADO) countRBF = 0;

    // Printa os dados na Serial:
    if (millis() - tempoPrintarDados >= TEMPO_PRINTAR_DADOS_S8){
      printaDadosSensoresSerial();
      tempoPrintarDados = millis();
    }
  } while (countRBF <= MAX_COUNT_PRINTAR_DADOS);

  return ESTADO_S0;  // executar estadoS8()
}
