/* Arquivo para configuração do sistema e definição de macros/constantes */





/*  ==========================  */
/*           PINOUT:            */


/*  Remove Before Flight:  */
#define RBF0 34
#define RBF1 35

/*  Paraquedas:  */
#define ELETROIMA 26
#define PORTA_IR  33  // LED Infravermelho
#define PORTA_FT  25  // Fototransistor

/*  Perifericos:  */
#define BUZZER_PINO       32
#define BUZZER_CANAL      0
#define BUZZER_FREQUENCIA 2000
#define BUZZER_RESOLUCAO  10
// #define LED

/*  Memória Flash Externa (VSPI):  */
#define FLASH_CS  5
#define FLASH_CLK 18
#define FLASH_DI  23  // MOSI
#define FLASH_DO  19  // MISO

/*  MPU9250:  */
#define MPU_SCL 22
#define MPU_SDA 21

/*  BMP280:  */
#define BMP_SCL 22
#define BMP_SDA 21

/*  LORA (HSPI):  */
#define LORA_CS   15
#define LORA_CLK  14
#define LORA_MISO 12
#define LORA_MOSI 13
#define LORA_RST  33
#define LORA_DIO0 2


/*  SD Card (HSPI):  */
#define SD_CS   27
#define SD_CLK  14
#define SD_MISO 12
#define SD_MOSI 13

/*  GPS:  */
#define GPS_TX        16    // Rx do uC
#define GPS_RX        17    // Tx do uC
#define GPS_BAUD_RATE 9600  // Serial Baud Rate





/*  ==========================  */
/*    ENDEREÇOS DE MEMÓRIA:     */


/*  TAMANHO DOS TIPOS DE VARIAVEL:  */

#define SIZE_CHAR   1  // sizeof(char)
#define SIZE_UCHAR  1  // sizeof(uchar)
#define SIZE_INT8   1  // sizeof(int8_t)
#define SIZE_UINT8  1  // sizeof(uint8_t)
#define SIZE_INT16  2  // sizeof(int16_t)
#define SIZE_UINT16 2  // sizeof(uint16_t)
#define SIZE_ULONG  4  // sizeof(unsigned long)
#define SIZE_FLOAT  4  // sizeof(float)


/*  EEPROM interna ESP32:  */

// Variaveis de Controle do Sistema:
#define SYS_CONTROL_EEPROM_SIZE         0x0020  // 0d32 posiçoes
#define SYS_EEPROM_ADDR_0               0x0000

#define EEPROM_ULTIMO_DADO_ADDR         (SYS_EEPROM_ADDR_0 + SIZE_UINT16)              // uint16_t - Armazena a ultima posiçao de memoria usada para salvar dados (DADOS_EEPROM)
#define EEPROM_RESETS_INESPERADOS_ADDR  (EEPROM_ULTIMO_DADO_ADDR + SIZE_UINT8)         // uint8_t  - Armazena o numero de resets inesperados
#define EEPROM_ESTADO_ATUAL_ADDR        (EEPROM_RESETS_INESPERADOS_ADDR + SIZE_UINT8)  // uint8_t  - Armazena o ultimo estado executado
#define EEPROM_EXECUCAO_ADDR            (EEPROM_ESTADO_ATUAL_ADDR + SIZE_UINT16)       // uint16_t - Armazena a ultima iteracao executada do loop do ultimo estado
#define EEPROM_APOGEU_ADDR              (EEPROM_EXECUCAO_ADDR + SIZE_FLOAT)            // float    - Armazena o apogeu medido pelo BMP
#define EEPROM_ACC_X_MAX_ADDR           (EEPROM_APOGEU_ADDR + SIZE_INT16)              // int16_t  - Armazena a maxima aceleracao no eixo x
#define EEPROM_ACC_Y_MAX_ADDR           (EEPROM_ACC_X_MAX_ADDR + SIZE_INT16)           // int16_t  - Armazena a maxima aceleracao no eixo y
#define EEPROM_ACC_Z_MAX_ADDR           (EEPROM_ACC_Y_MAX_ADDR + SIZE_INT16)           // int16_t  - Armazena a maxima aceleracao no eixo z
#define EEPROM_TEMPO_INICIAL_VOO_ADDR   (EEPROM_ACC_Z_MAX_ADDR + SIZE_ULONG)           // ulong    - Armazena o tempo em que o voo iniciou (millis)
#define EEPROM_PRESSAO_LOCAL_ADDR       (EEPROM_TEMPO_INICIAL_VOO_ADDR + SIZE_FLOAT)   // float    - Armazena a pressao local antes do inicio do voo
#define EEPROM_ALTITUDE_INICIAL_ADDR    (EEPROM_PRESSAO_LOCAL_ADDR + SIZE_FLOAT)       // float    - Armazena a altitude local antes do inicio do voo
#define EEPROM_TEMP_INICIAL_BMP_ADDR    (EEPROM_ALTITUDE_INICIAL_ADDR + SIZE_FLOAT)    // float    - Armazena a temperatura local antes do inicio do voo (BMP)
#define EEPROM_TEMP_INICIAL_MPU_ADDR    (EEPROM_TEMP_INICIAL_BMP_ADDR + SIZE_INT16)    // int16_t  - Armazena a temperatura local antes do inicio do voo (MPU)

// Dados dos Sensores:
#define DADOS_EEPROM_SIZE              0x1000  // 0d4096 posiçoes
#define DADOS_EEPROM_ADDR_0            0x0000  // Primeiro endereço desta partiçao da EEPROM
#define DADOS_EEPROM_OFFSET_TOTAL      34      // Tamanho de uma rodada de todos os dados (soma dos offsets abaixo)
#define DADOS_EEPROM_ADDR_MAX          (DADOS_EEPROM_SIZE - DADOS_EEPROM_OFFSET_TOTAL - 1)  // Menos 1 porque vai de 0x0000 a 0x0FFF

#define EEPROM_OFFSET_TEMPO_VOO        SIZE_ULONG
#define EEPROM_OFFSET_BMP_ALTURA       SIZE_FLOAT
#define EEPROM_OFFSET_BMP_TEMPERATURA  SIZE_FLOAT
#define EEPROM_OFFSET_MPU_ACC_X        SIZE_INT16
#define EEPROM_OFFSET_MPU_ACC_Y        SIZE_INT16
#define EEPROM_OFFSET_MPU_ACC_Z        SIZE_INT16
#define EEPROM_OFFSET_MPU_TEMP         SIZE_INT16
#define EEPROM_OFFSET_MPU_GYR_X        SIZE_INT16
#define EEPROM_OFFSET_MPU_GYR_Y        SIZE_INT16
#define EEPROM_OFFSET_MPU_GYR_Z        SIZE_INT16
#define EEPROM_OFFSET_GPS_LAT          SIZE_FLOAT
#define EEPROM_OFFSET_GPS_LON          SIZE_FLOAT


/*  Flash externa:  */





/*  ==========================  */
/*          USO GERAL:          */


/*  Inicializaçao:  */

#define TEMPO_TRANSMITIR_INIT 10000  // Tempo para transmitir o pacote de inicializaçao em S2 e S3 [ms]
#define MAX_COUNT_INIT_BMP    10     // Max numero de tentativas para inicializaçao do BMP [num]
#define MAX_COUNT_INIT_MPU    10     // Max numero de tentativas para inicializaçao do MPU [num]
#define MAX_COUNT_INIT_LORA   10     // Max numero de tentativas para inicializaçao do LORA [num]
#define MAX_COUNT_INIT_GPS    10     // Max numero de tentativas para inicializaçao do GPS [num]
#define MAX_COUNT_INIT_EEPROM 10     // Max numero de tentativas para inicializaçao do GPS [num]


/*  Geral:  */

#define TEMPO_SALVAR_DADOS_S3  500    // Tempo para salvar os dados na EEPROM em S3 [ms]
#define TEMPO_LER_GPS_S3       5000   // Tempo para ler os dados do gps em S3 [ms]
#define TEMPO_LER_GPS_S7       10000  // Tempo para ler os dados do gps em S7 [ms]
#define TEMPO_LER_GPS_VOO      1000   // Tempo para ler os dados do gps em S4, S5 e S6 [ms]
#define TEMPO_SALVAR_DADOS_S6  500    // Tempo para salvar os dados na EEPROM em S6 [ms]
#define TEMPO_PRINTAR_DADOS_S8 30000  // Tempo para printar os dados na Serial em S8 [ms]


/*  Detecçao de inicio do voo:  */

#define TEMPO_ATUALIZAR_PRESSAO        60000  // Atualiza a pressao atm local enquanto nao detecta o inicio do voo (para evitar deteçcao errada) [ms]
#define OFFSET_DETEC_INICIO_VOO_BMP    2.0    // Offset na altura para detecçao de apogeu por BMP [m]
#define MAX_COUNT_DETEC_INICIO_VOO_BMP 5      // Contagem para detecçao de inicio do voo por BMP [num]
#define INICIO_VOO_DETECTADO_BMP       true
#define INICIO_VOO_NAO_DETECTADO_BMP   false


/*  Detecçao de apogeu:  */

#define MAX_COUNT_DETEC_APOGEU_BMP   10     // Contagem para detecçao de apogeu por BMP [num]
#define MAX_COUNT_DETEC_APOGEU_MPU   10     // Contagem para detecçao de apogeu por MPU [num]
#define MAX_COUNT_DETEC_APOGEU_TIMER 13000  // Tempo desde o inicio do voo (TempoInicialVoo) para detecçao de apogeu por timer [ms]

#define APOGEU_DETECTADO_BMP       true
#define APOGEU_DETECTADO_MPU       true
#define APOGEU_DETECTADO_TIMER     true
#define APOGEU_NAO_DETECTADO_BMP   false
#define APOGEU_NAO_DETECTADO_MPU   false
#define APOGEU_NAO_DETECTADO_TIMER false


/*  Abertura da porta do paraquedas:  */

#define MAX_COUNT_ABRIR_PORTA   20     // Contagem para detecçao de apogeu por BMP [num]


/*  Detecçao de pouso:  */

#define MAX_COUNT_DETEC_POUSO_BMP   10      // Contagem para detecçao de pouso por BMP [num]
#define MAX_COUNT_DETEC_POUSO_MPU   10      // Contagem para detecçao de pouso por MPU [num]
#define MAX_COUNT_DETEC_POUSO_TIMER 120000  // Tempo desde o inicio do voo (TempoInicialVoo) para detecçao de pouso por timer [ms]

#define POUSO_DETECTADO_BMP       true
#define POUSO_DETECTADO_MPU       true
#define POUSO_DETECTADO_TIMER     true
#define POUSO_NAO_DETECTADO_BMP   false
#define POUSO_NAO_DETECTADO_MPU   false
#define POUSO_NAO_DETECTADO_TIMER false

#define INTERVALO_DETEC_POUSO_BMP 1.0  // Novas leituras tem que estar dentro de +/- esse intervalo [m]


/*  Abertura da porta do paraquedas:  */

#define MAX_COUNT_PRINTAR_DADOS 30  // Contagem para printar os dados na Serial apos a recuperaçao e comando de reinicio (RBF = 2) [num]


/*  Número de beeps do buzzer:  */

#define BEEP_NOVO_VOO  3
#define BEEP_ESTADO_S0 1
#define BEEP_ESTADO_S1 1
#define BEEP_ESTADO_S2 2
#define BEEP_ESTADO_S3 3
#define BEEP_ESTADO_S4 4
#define BEEP_ESTADO_S5 5
#define BEEP_ESTADO_S6 6
#define BEEP_ESTADO_S7 7
#define BEEP_ESTADO_S8 8


/*  Calibraçao de Sensores:  */

// BMP:
#define COUNT_CALIBRAR_PRESSAO_BMP     10  // [num]
#define COUNT_CALIBRAR_ALTURA_BMP      10  // [num]
#define COUNT_CALIBRAR_TEMPERATURA_BMP 10  // [num]

#define DELAY_CALIBRAR_PRESSAO_BMP     10  // [ms]
#define DELAY_CALIBRAR_ALTURA_BMP      10  // [ms]
#define DELAY_CALIBRAR_TEMPERATURA_BMP 10  // [ms]

// MPU:
#define MPU_ACCEL_RANGE  MPU6050_RANGE_16_G
#define MPU_GYRO_RANGE   MPU6050_RANGE_1000_DEG
#define MPU_FILTER_BAND  MPU6050_BAND_94_HZ

#define MPU_ACCEL_SCALE 2048   // if MPU_ACCEL_RANGE = MPU6050_RANGE_16_G
// #define MPU_ACCEL_SCALE 4096   // if MPU_ACCEL_RANGE = MPU6050_RANGE_8_G
// #define MPU_ACCEL_SCALE 8192   // if MPU_ACCEL_RANGE = MPU6050_RANGE_4_G
// #define MPU_ACCEL_SCALE 16384  // if MPU_ACCEL_RANGE = MPU6050_RANGE_2_G

// #define MPU_GYRO_SCALE 131   // if MPU_GYRO_RANGE = MPU6050_RANGE_250_DEG
// #define MPU_GYRO_SCALE 65.5  // if MPU_GYRO_RANGE = MPU6050_RANGE_500_DEG
#define MPU_GYRO_SCALE 32.8  // if MPU_GYRO_RANGE = MPU6050_RANGE_1000_DEG
// #define MPU_GYRO_SCALE 16.4  // if MPU_GYRO_RANGE = MPU6050_RANGE_2000_DEG

#define COUNT_CALIBRAR_ACELERACAO_MPU  10  // [num]
#define COUNT_CALIBRAR_GIROSCOPIO_MPU  10  // [num]
#define COUNT_CALIBRAR_TEMPERATURA_MPU 10  // [num]

#define DELAY_CALIBRAR_ACELERACAO_MPU  10  // [ms]
#define DELAY_CALIBRAR_GIROSCOPIO_MPU  10  // [ms]
#define DELAY_CALIBRAR_TEMPERATURA_MPU 10  // [ms]





/*  ==========================  */
/*           MACROS:            */


/*  PORTA ELETROIMA:  */

#define PORTA_EI_ABERTA  false
#define PORTA_EI_FECHADA true
#define PORTA_EI_ABRIR   LOW
#define PORTA_EI_FECHAR  HIGH


/*  REMOVE BEFORE FLIGHT:  */

#define RBF_CHAVE_OFF HIGH
#define RBF_CHAVE_ON  LOW

#define RBF_ANTES_DO_VOO   2
#define RBF_VOANDO         0
#define RBF_AGUARDANDO_REC 0
#define RBF_RECUPERADO     1


/*  ARMAZENAMENTO:  */

#define SALVAR_DADOS           true
#define SALVAR_DADOS_TEMPO_VOO true
#define SALVAR_DADOS_BMP       true
#define SALVAR_DADOS_MPU       true
#define SALVAR_DADOS_GPS       true

#define NAO_SALVAR_DADOS           false
#define NAO_SALVAR_DADOS_TEMPO_VOO false
#define NAO_SALVAR_DADOS_BMP       false
#define NAO_SALVAR_DADOS_MPU       false
#define NAO_SALVAR_DADOS_GPS       false