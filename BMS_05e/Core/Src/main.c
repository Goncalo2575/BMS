/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pl455.h"
#include "CO_app_STM32.h"
#include "OD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SAFE = 0,
	ENGAGED,
	CHARGING,
	NOT_SAFE
} BMS_STATE_EnumTypedef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_CELLS_SLAVE 15
#define NUM_TEMPERATURES 6
#define NUM_CELLS 30
#define NUM_SLAVES 2
#define SLAVE_RX_BUFF_SIZE 128

#define OVER_VOLT 4.2
#define UNDER_VOLT 2.5

#define VOLT_SEND_OFFSET 2.2

#define OVER_TEMPERATURE 70.0 // CELCIUS
#define WARNING_TEMPERATURE 65.0 //CELCIUS

#define DEBOUNCE_DELAY 100

#define COMS_SLAVES 10000 //10 seconds without communications, trigger BMS_OK to false

// For CANOpen
#define ARR_SEQ_LEN  8 // 6tpdos × 17 ms + 1×898 ms
#define NUM_TICKS ARR_SEQ_LEN

//Variable CanOpen TPDO Erros
#define canOpen_no_issues 0b00
#define canOpen_overvoltage 0b01
#define canOpen_undervoltage 0b10
#define canOpen_overcurrent 0b11
#define canOpen_warningTemperature 0b01
#define canOpen_overTemperature 0b10

#define canOpen_safe 0b00
#define canOpen_engaged 0b01
#define canOpen_charging 0b10
#define canOpen_notsafe 0b11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim6_up;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
/* BMS STATE */
BMS_STATE_EnumTypedef BMS_state;
bool_t BMS_state_change;

/* BMS OK */
bool_t BMS_OK;
/* IMD OK */
bool_t IMD_OK;

/* ESDB ON */
bool_t ESDB_flag;
/* MEASURE ESDB */
bool_t ESDB_watchdog;

/* ESDB_charger ON */
bool_t ESDB_charge_flag;
/* MEASURE ESDB_charger */
bool_t ESDB_charge_watchdog;

/* TSMS (KSI) ON */
bool_t TSMS_flag;
/* MEASURE KSI (TSMS) */
bool_t TSMS_watchdog;

/* CHARGER Optocoupler */
bool_t charger_optocoupler_flag;
/* charger Optocoupler Watch Dog */
bool_t charger_optocoupler_watchdog;
/* charger relay enable variable */
bool_t charger_relay_enable;

/* Pre-charge and Dis_charge variable */
bool_t pre_charge_enable;
bool_t dis_charge_enable;
/* Charge enable */
bool_t charge_enable;

/* ERRORS */
uint32_t under_volt, over_volt, can_auxiliar;
uint8_t over_temp;
uint8_t warning_temp;
bool_t over_current;
bool_t coms_slave;
uint8_t last_cells_error;
uint8_t lastlast_cells_error;

uint32_t coms_slaves_time;

/* SLAVES */
float v_slave1[NUM_CELLS_SLAVE];
float v_slave2[NUM_CELLS_SLAVE];
float temperatures[NUM_TEMPERATURES];

float B_plus;
float B_plus_fuse;

float HV_plus;

/* M-S COMMS */
char msg_debug[512]; //message between Master and PC for debug
size_t msg_debug_len; //message with the length of msg_debug

uint8_t msg_slave[SLAVE_RX_BUFF_SIZE]; //message between master and slave
uint16_t msg_slave_len;

bool_t MS_send_flag;
bool_t MS_receive_flag;

/* CAN BUS */
uint16_t timestampGrupo = 0;

volatile uint8_t flagEnvioTPDO = 0;
uint8_t blocoAtual = 0;
uint8_t contadorTicks = 0;

/*Debug state char*/
char *BMS_STATE_STRINGS[] = {
    "SAFE",
    "ENGAGED",
    "CHARGING",
	"NOT_SAFE"
};

uint32_t debounce_start;
uint8_t debounce_pending;

// For CANOpen
static const uint16_t arrSeq[ARR_SEQ_LEN] =
		{ 169, 169, 169, 169, 169, 169, 169, 8809 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //FIXME: quando forem mudar a placa da master, alguns watchdogs pinos estão diferentes na versão velha e na nova, tem que alterar isso no ponto IOC
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BMS_state = NOT_SAFE;
  BMS_state_change = true;

  under_volt = 0;
  over_volt = 0;
  can_auxiliar = 0;
  over_temp = 0;
  warning_temp = 0;
  over_current = false;
  coms_slave = true;

  coms_slaves_time = HAL_GetTick();

  BMS_OK = true;
  IMD_OK = true;

  ESDB_watchdog = false;
  ESDB_flag = true;

  ESDB_charge_watchdog = false;
  ESDB_charge_flag = true;

  TSMS_watchdog = false;
  TSMS_flag = false;

  charger_optocoupler_flag = false;
  charger_optocoupler_watchdog = false;
  charger_relay_enable = false;

  pre_charge_enable = false;
  dis_charge_enable = false;
  charge_enable = false;

  MS_send_flag = false;
  MS_receive_flag = false;

  debounce_pending = 0;

  last_cells_error = 0x00;
  lastlast_cells_error = 0x00;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /*SET BMS_RELAY INITIAL CLOSE */
  HAL_GPIO_WritePin(BMS_relay_GPIO_Port, BMS_relay_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMS_charge_GPIO_Port, BMS_charge_Pin, GPIO_PIN_SET); //FIXME: Acrescentei isto uma vez que estes dois relays funcionam como se fossem um só e começam como se tudo estivesse bem

  //FIXME: Acrescentei isto que é o controlo do relay do charger que é controlado pelo carregador, por essa mesma razão, este começa a false
  HAL_GPIO_WritePin(charge_GPIO_Port, charge_Pin, GPIO_PIN_RESET);
  charger_relay_enable = false;

  HAL_Delay(100); //Delay just to ensure that the current will

  /* READ INITIAL VALUES OF THE WATCHDOGS */
  if( HAL_GPIO_ReadPin(status_GPIO_Port, status_Pin) == GPIO_PIN_SET){
  	  IMD_OK = true;
    }
    else{
  	  IMD_OK = false;
    }

  if( HAL_GPIO_ReadPin(KSI_monitor_GPIO_Port, KSI_monitor_Pin) == GPIO_PIN_SET){
	  TSMS_watchdog = true;
  }
  else{
	  TSMS_watchdog = false;
  }

  if( HAL_GPIO_ReadPin(ESDB_monitor_GPIO_Port, ESDB_monitor_Pin) == GPIO_PIN_SET){
	  ESDB_watchdog = true;
  }
  else{
	  ESDB_watchdog = false;
  }

  if( HAL_GPIO_ReadPin(ESDB_charger_monitor_GPIO_Port, ESDB_charger_monitor_Pin) == GPIO_PIN_SET){
  	  ESDB_charge_watchdog = true;
    }
    else{
  	  ESDB_charge_watchdog = false;
    }

  if( HAL_GPIO_ReadPin(charger_signal_GPIO_Port, charger_signal_Pin) == GPIO_PIN_SET) {
	  charger_optocoupler_watchdog = true;
  }
  else {
	  charger_optocoupler_watchdog = false;
  }

  //Message to debug, verify if the connection between BMS and PC are good
  msg_debug_len = sprintf(msg_debug, "BMS Master Initialized with success.\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 1000);
  memset(msg_debug, 0, msg_debug_len);

  //FIXME: Tem que descomentar o codigo do wakeup e setups dos slaves
  /* WakeUp and Setup Routine for BQ76PL455A-Q1 */
  HAL_Delay(500);
  wakeUp(); //Wake up the master slave and he will wake up the others

  HAL_Delay(500);
  setupSlave(); //Setup all the slave
  //FIXME: Alterei uma cena no balancing setup
  balancing_setup();

  //This will receive the data from slave (uart) via DMA - activate after the setup is done
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, msg_slave, SLAVE_RX_BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT); //This deactivates the callback on half the message receive from slave

  /* CanOPEN initiation */
  CANopenNodeSTM32 canopenNodeSTM32;
  canopenNodeSTM32.CANHandle = &hcan1;
  canopenNodeSTM32.HWInitFunction = MX_CAN1_Init;
  canopenNodeSTM32.timerHandle = &htim7;
  canopenNodeSTM32.desiredNodeID = 32; //TODO: a modificar
  canopenNodeSTM32.baudrate = 1000;
  canopen_app_init(&canopenNodeSTM32);
  HAL_DMA_Start(&hdma_tim6_up, (uint32_t) arrSeq, (uint32_t) &TIM6->ARR, ARR_SEQ_LEN);
  __HAL_TIM_ENABLE_DMA(&htim6, TIM_DMA_UPDATE);
  HAL_TIM_Base_Start_IT(&htim6);

  /* SET THE PWM FOR GREEN LED*/
  TIM2->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* SET THE TIMERS TO START RUN*/
  HAL_TIM_Base_Start_IT(&htim4);

  //powerDown();

  HAL_Delay(5000); //Wait for IMD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while(1)

  while (1)
  {

	  // First thing ask for voltages (so it hopefully doesnt block)
	  if (MS_send_flag == true) {
		  MS_send_flag = false;
		  WriteReg(0, 2, 0x01, 1, FRMWRT_ALL_R);   //method 1-broadcast

		  //Debug, como isto acontece de 1 em 1 segundo, vou dar print do estado das coisas da BMS aqui
// ESDB_watchdog == (BMS_OK && IMD_OK && !charger_relay_enable
		  msg_debug_len = sprintf(msg_debug,
				  "------------------------------------------------\n\n"
				  "\t[BMS STATES]\n"
				  "\tESDB_flag = %d\n"
				  "\tESDB_charge_flag = %d\n"
				  "\tTSMS_flag = %d\n"
				  "\tESDB_wd = %d\n"
				  "\tESDB_charge_wd = %d\n"
				  "\tBMS_OK = %d\n"
				  "\tIMD_OK = %d\n"
				  "\tcharger_relay_enable = %d\n",
				  ESDB_flag,
				  ESDB_charge_flag,
				  TSMS_flag,
				  ESDB_watchdog,
				  ESDB_charge_watchdog,
				  BMS_OK,
				  IMD_OK,
				  charger_relay_enable);
		  HAL_UART_Transmit(&huart2, (uint8_t *)msg_debug, msg_debug_len, 100);


	  }


	  if(debounce_pending == 1 && (HAL_GetTick() - debounce_start) >= DEBOUNCE_DELAY){

		  debounce_pending = 0;

		  if( HAL_GPIO_ReadPin(status_GPIO_Port, status_Pin) == GPIO_PIN_SET){
			  IMD_OK = true;
		  }
		  else{
			  IMD_OK = false;
		  }

		  if( HAL_GPIO_ReadPin(KSI_monitor_GPIO_Port, KSI_monitor_Pin) == GPIO_PIN_SET){
			  TSMS_watchdog = true;
		  }
		  else{
			  TSMS_watchdog = false;
		  }

		  if( HAL_GPIO_ReadPin(ESDB_monitor_GPIO_Port, ESDB_monitor_Pin) == GPIO_PIN_SET){
			  ESDB_watchdog = true;
		  }
		  else{
			  ESDB_watchdog = false;
		  }
		  //FIXME: Descomentei isto
		  if( HAL_GPIO_ReadPin(ESDB_charger_monitor_GPIO_Port, ESDB_charger_monitor_Pin) == GPIO_PIN_SET){
			  ESDB_charge_watchdog = true;
		  }
		  else{
			  ESDB_charge_watchdog = false;
		  }

		  if( HAL_GPIO_ReadPin(charger_signal_GPIO_Port, charger_signal_Pin) == GPIO_PIN_SET) {
			  charger_optocoupler_watchdog = true;
		  }
		  else {
			  charger_optocoupler_watchdog = false;
		  }
	  }

	  //TODO: Adicionei isto: Quando é detetado que o carregador foi ligado, tem que se ligar o Charger_Relay
	  /* Charger Watchdog */

	  //Quando o carregador é ligado, ativar o relay do charger na placa dos relays
	  if( (charger_optocoupler_flag = charger_optocoupler_watchdog) == true ) {
		  if (charger_relay_enable == false) {
			  HAL_GPIO_WritePin(charge_GPIO_Port, charge_Pin, GPIO_PIN_SET);
			  charger_relay_enable = true;
		  }
	  }
	  else {
		  if(charger_relay_enable == true) {
			  HAL_GPIO_WritePin(charge_GPIO_Port, charge_Pin, GPIO_PIN_RESET);
			  charger_relay_enable = false;
		  }
	  }
	  //FIXME: Outro ponto, pode estar a acontecer de o optoucoupler, quando o carregador não está ligado, não tem o gnd de um lado e fica a flutuar na saida dele, entrada da master, neste caso, acho que isto é facilmente resolvido com um pull down interno neste pino da master que está a ler isto.

	  //FIXME: Mudei isto para: os dois relays da BMS é que funcionam como um e não o BMS_relay e o charger_Pin, ou seja, o BMS_relay e BMS_charge
	  /* BMS OK, if not -> open BMS relay */
	  if( (BMS_OK = !(under_volt || over_volt || over_temp || over_current || !coms_slave)) == false || IMD_OK == false ) {
		  HAL_GPIO_WritePin(BMS_relay_GPIO_Port, BMS_relay_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(BMS_charge_GPIO_Port, BMS_charge_Pin, GPIO_PIN_RESET);
	  }

	  /* TSMS and ESDB */
	  ESDB_flag = ESDB_watchdog == (BMS_OK && IMD_OK); // evitar mandar erro do ESDB quando a BMS != OK ou IMD != OK
	  TSMS_flag = ESDB_flag && TSMS_watchdog;

	  //TODO: Adicionei: Condição para verificar o ESDB_charger
	  /* ESDB_charger */
	  ESDB_charge_flag = true;

	  /* check if B_plus is at <5.0V */
	  if (dis_charge_enable == true && B_plus <= 5.0){
		  HAL_GPIO_WritePin(ctr_discharge_GPIO_Port, ctr_discharge_Pin, GPIO_PIN_RESET);
	  	  dis_charge_enable = false;
	  }

	  /* check if pre-check is being done and if yes, it's at maximum 90% of the total tension of the pack */
	  if (pre_charge_enable == true && B_plus >= (HV_plus * 0.9)){
		  HAL_GPIO_WritePin(pre_charge_GPIO_Port, pre_charge_Pin, GPIO_PIN_RESET);
		  pre_charge_enable = false;
	  }

	  /* SWITCH STATE */
	  //TODO: Mudei isto para incluir a logica do carregador
	  /* Basicamente, se o carregador tiver ligado, o relay_charger_enable vai estar a true e por essa razão, o ESDB_charger_flag pode ter medições certas,
	   * caso este não esteja conectado, as medições deste são inuteis e não são tidas em conta para o estado Not Safe.
	   * Caso, esteja tudo okay, entramos no outro else, onde a primeira condição que verificamos é se o charger_optoucoupler_flag == true,
	   * isto significa, que o carregador está ligado à mota e entramos no caso do charging, nunca podendo entrar no caso do safe ou engaged uma vez
	   * que não queremos que a roda começa a girar caso a mota esteja a carregar
	   */
	  if (BMS_OK == false || IMD_OK == false || ESDB_flag == false) { // Adicionei a condição do charger
		  // NOT SAFE
		  if (BMS_state != NOT_SAFE) {
			  BMS_state = NOT_SAFE;
			  BMS_state_change = true;
		  }
	  }
	  else {
		  if (charger_optocoupler_flag == true){
			  if (BMS_state != CHARGING){
				  BMS_state = CHARGING;
				  BMS_state_change = true;
			  }
		  }
		  else if (TSMS_flag == false) {
			  // SAFE
			  if (BMS_state != SAFE) {
				  BMS_state = SAFE;
				  BMS_state_change = true;
		  	  }
		  }
		  else {
			  // ENGAGED
			  if (BMS_state != ENGAGED) {
				  BMS_state = ENGAGED;
				  BMS_state_change = true;
			  }
		  }
	  }
	  //TODO: Continua comentado - Eu acho que esta lógica está mal, vou implementar a logica como eu acho que faz sentido em cima
	  /*
	  if (BMS_state == CHARGING) {
		  if (ESDB_charge_flag == false) { // falta uma condição para o charger
			  // NOT SAFE
			  if (BMS_state != NOT_SAFE) {
				  BMS_state = NOT_SAFE;
				  BMS_state_change = true;
			  }
		  }

	  }
	  else if (charger_flag == true) {
		  // charger turned ON
		  if (BMS_state != CHARGING) {
			  BMS_state = CHARGING;
			  BMS_state_change = true;
		  }
	  }*/

	  /* STATE DEFINE */
	  if (BMS_state_change == true) {

		  switch (BMS_state) {
			  case NOT_SAFE:

				  HAL_GPIO_WritePin(pre_charge_GPIO_Port, pre_charge_Pin, GPIO_PIN_RESET);
				  pre_charge_enable = false;
				  //FIXME: Eu acho que isto que está comentado, não faz sentido porque:
				  // caso seja o BMS_charge pino, este deve ser controlado ao mesmo tempo que o pino do BMS_relay
				  // caso seja o charge_relay pino, este deve ser controlado pelo controlador
				  // portanto, em nenhum caso, deve ser alterado aqui
				  /*HAL_GPIO_WritePin(BMS_charge_GPIO_Port, BMS_charge_Pin, GPIO_PIN_RESET);
				  charge_enable = false;*/

				  if (dis_charge_enable == false && B_plus >= 5.0){
					  HAL_GPIO_WritePin(ctr_discharge_GPIO_Port, ctr_discharge_Pin, GPIO_PIN_SET);
					  dis_charge_enable = true;
				  }

				  /* LED CLUSTER */
				  TIM2->CCR1 = 0;
				  HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_SET);
				  break;

			  case SAFE:

				  /* Turn off pre-charge circuit for safety reasons*/
				  HAL_GPIO_WritePin(pre_charge_GPIO_Port, pre_charge_Pin, GPIO_PIN_RESET);
				  pre_charge_enable = false;
				  //FIXME: Aqui é a mesma justificação que o caso Not Safe
				 /* HAL_GPIO_WritePin(BMS_charge_GPIO_Port, BMS_charge_Pin, GPIO_PIN_RESET);
				  charge_enable = false;*/

				  if (dis_charge_enable == false && B_plus >= 5.0){
					  HAL_GPIO_WritePin(ctr_discharge_GPIO_Port, ctr_discharge_Pin, GPIO_PIN_SET);
					  dis_charge_enable = true;
				  }

				  /* LED CLUSTER */
				  HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_RESET);

				  TIM2->CCR1 = 5000;
				  break;

			  case ENGAGED:

				  /* Turn off discharge circuit for safety reasons*/
				  HAL_GPIO_WritePin(ctr_discharge_GPIO_Port, ctr_discharge_Pin, GPIO_PIN_RESET);
				  dis_charge_enable = false;
				  //FIXME: Aqui é a mesma justificação que o case Not Safe
				  /*HAL_GPIO_WritePin(BMS_charge_GPIO_Port, BMS_charge_Pin, GPIO_PIN_RESET);
				  charge_enable = false;*/
				  HAL_Delay(750);
				  if (pre_charge_enable == false && B_plus < (HV_plus * 0.9)) {
					  HAL_GPIO_WritePin(pre_charge_GPIO_Port, pre_charge_Pin, GPIO_PIN_SET);
					  pre_charge_enable = true;
				  }

				  /* LED CLUSTER */
				  TIM2->CCR1 = 10001;
				  HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_RESET);
				  break;

			  case CHARGING:

				  /* Turn off pre-charge circuit for safety reasons */
				  HAL_GPIO_WritePin(pre_charge_GPIO_Port, pre_charge_Pin, GPIO_PIN_RESET);
				  pre_charge_enable = false;
				  //FIXME: Isto aqui acho que desaparece, porque passamos a fazer isto assim que detetamos que o charger_optocoupler_watchdog (optoucoupler) é lido.
				  //FIXME: Caso isto fosse feito aqui, o que aconteceria, é que iriamos entrar no estado do Charging, mas sempre no Not Safe porque a primeira vez a flag do ESDB_Charger estava a falso e nunca mais sai porque iamos sempre estar no NOT Safe
				  /*HAL_GPIO_WritePin(charge_GPIO_Port, charge_Pin, GPIO_PIN_SET);
				  charge_enable = true;*/

				  if (dis_charge_enable == false && B_plus >= 5.0){
					  HAL_GPIO_WritePin(ctr_discharge_GPIO_Port, ctr_discharge_Pin, GPIO_PIN_SET);
					  dis_charge_enable = true;
				  }

				  /* LED CLUSTER */
				  TIM2->CCR1 = 0;
				  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(led_blue_GPIO_Port, led_blue_Pin, GPIO_PIN_SET);
				  break;

			  default:
				  break;

		  }
		  BMS_state_change = false;

	  }

	  // Get voltages/temperatures
	  if (MS_receive_flag == true) {
		  MS_receive_flag = false;
		  dataSlave(msg_slave);

		  coms_slaves_time = HAL_GetTick();

		  // CHECK UNDER/OVER VOLTAGE, OVER TEMPERATURE and OVER CURRENT

		  //Reset das variaveis de erro
		  over_volt = 0;
		  under_volt = 0;
		  can_auxiliar = 0;
		  warning_temp = 0;
		  over_temp = 0;
		  over_current = false;
		  coms_slave = true;

		  //UNDER/OVER VOLTAGE
		  for (uint8_t i = 0; i < NUM_CELLS_SLAVE; i++) {

			  if (v_slave1[i] > OVER_VOLT)
				  over_volt |= 0b1 << i;
			  else if (v_slave1[i] < UNDER_VOLT) {
				  if (i == 0 && v_slave1[0] < UNDER_VOLT - 0.5) {
					  under_volt |= 0b1;
				  }
				  else {

					  under_volt |= 0b1 << i;
				  }
			  }

			  if (v_slave2[i] > OVER_VOLT)
				  over_volt |= 0b1 << (i + 15);

			  else if (v_slave2[i] < UNDER_VOLT)
				  under_volt |= 0b1 << (i + 15);

		  }

		  //OVER TEMPERATURE
		  for (uint8_t i = 0; i < NUM_TEMPERATURES; i++) {

			  if (temperatures[i] > OVER_TEMPERATURE)
				  over_temp |= 0b1 << i;

			  else if (temperatures[i] > WARNING_TEMPERATURE){
				  warning_temp |= 0b1 << i;
			  }
		  }

		  //FIXME: Neste momento isto nunca vai ser ativado porque o fusivel está no B- e não no B+, então caso o fusivel rebente, no B+ não notamos nada
		  //OVER CURRENT TODO: Neste momento isto não está a ser visto porque o fusivel está no B-, ou seja esta condição é inutil
		  if( B_plus == 0 && BMS_state == ENGAGED && pre_charge_enable == false ) {
			  over_current = true;
		  }

		  /* CHARGING - BALACING */
	  	  if (BMS_state == CHARGING) {
	  		  //balancing();
		  }
	  }
	  else if( (HAL_GetTick() - coms_slaves_time) >= COMS_SLAVES){

		  coms_slave = false;
		  coms_slaves_time = HAL_GetTick();

	  }

	  /* CAN BUS */
	  /*ACHO QUE O CAN DEVE SER A ULTIMA CENA A SER FEITA NO CODIGO PARA SE MANDAR TAMBÈM LOGO OS ERROS*/
	  canopen_app_process();

	  if (flagEnvioTPDO) {
		flagEnvioTPDO = 0;

		if (contadorTicks == 0) {
			timestampGrupo = (HAL_GetTick() / 1000) % 8192;

		}

		if (contadorTicks < (NUM_TICKS - 1)) {

			uint16_t header = ((blocoAtual & 0x07) << 13)
					| (timestampGrupo & 0x1FFF);

			//FIXME: DESATIVAR QUANDO SE COMEÇAR A USAR O TPDO DOS ERROS
			//OD_PERSIST_COMM.x6000_V_C.header = header;

			/*FIXME: ATIVAR QUANDO SE COMEÇAR A USAR O TPDO TAMBEM DOS ERROS*/
			if (contadorTicks == (NUM_TICKS - 2)){
				OD_PERSIST_COMM.x6001_errors_State.header = header;
			}
			else{
				OD_PERSIST_COMM.x6000_V_C.header = header;
			}


			switch (blocoAtual) {
				case 0:
					for (int i = 0; i < 6; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t)((v_slave1[NUM_CELLS_SLAVE - 1 - i] - 2.2) * 100.0);
					}
					break;
				case 1:
					for (int i = 0; i < 6; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t)((v_slave1[NUM_CELLS_SLAVE - 1 - 6 - i] - 2.2) * 100.0);
					}
					break;

				case 2:
					for (int i = 0; i < 3; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t)((v_slave1[NUM_CELLS_SLAVE - 1 - 12 - i] - 2.2) * 100.0);
					}
					for (int i = 0; i < 3; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i + 3] =
								(uint8_t)((v_slave2[NUM_CELLS_SLAVE - 1 - i] - 2.2) * 100.0);
					}
					break;

				case 3:
					for (int i = 0; i < 6; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t)((v_slave2[NUM_CELLS_SLAVE - 1 - 3 - i] - 2.2) * 100.0);
					}
					break;

				case 4:
					for (int i = 0; i < 6; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t)((v_slave2[NUM_CELLS_SLAVE - 1 - 9 - i] - 2.2) * 100.0);
					}
					break;

				case 5:
					for (int i = 0; i < 6; i++) {
						((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i] =
								(uint8_t) (temperatures[i]) * 2.0;
					}
					break;

				case 6:

					uint8_t sum_cells_error = 0;

					if (under_volt > 0) sum_cells_error++;
					if (over_volt > 0) sum_cells_error++;
					if (over_current == true) sum_cells_error++;

					if(under_volt > 0 && (sum_cells_error == 1 || (sum_cells_error >= 2 && last_cells_error != canOpen_undervoltage) || (sum_cells_error == 3 && lastlast_cells_error != canOpen_undervoltage))){
						under_volt |= canOpen_undervoltage << 30;
						OD_PERSIST_COMM.x6001_errors_State.voltage_current_vals = under_volt;

					} else if(over_volt > 0 && (sum_cells_error == 1 || (sum_cells_error >= 2 && last_cells_error != canOpen_overvoltage) || (sum_cells_error == 3 && lastlast_cells_error != canOpen_overvoltage))){
						over_volt |= canOpen_overvoltage << 30;
						OD_PERSIST_COMM.x6001_errors_State.voltage_current_vals = over_volt;

					} else if(over_current == true && (sum_cells_error == 1 || (sum_cells_error >= 2 && last_cells_error != canOpen_overcurrent) || (sum_cells_error == 3 && lastlast_cells_error != canOpen_overcurrent))){
						can_auxiliar |= canOpen_overcurrent << 30;
						OD_PERSIST_COMM.x6001_errors_State.voltage_current_vals = can_auxiliar;

					} else {
						can_auxiliar = canOpen_no_issues << 30;
						OD_PERSIST_COMM.x6001_errors_State.voltage_current_vals = can_auxiliar;

					}


					if (over_temp > 0) {
						over_temp |= canOpen_overTemperature << 6;
						OD_PERSIST_COMM.x6001_errors_State.temperatures = over_temp;
					}
					else if (warning_temp > 0) {
						warning_temp |= canOpen_warningTemperature << 6;
						OD_PERSIST_COMM.x6001_errors_State.temperatures = warning_temp;
					}
					else {
						can_auxiliar = canOpen_no_issues << 6;
						OD_PERSIST_COMM.x6001_errors_State.temperatures = can_auxiliar;
					}

					OD_PERSIST_COMM.x6001_errors_State.states =
							(BMS_state << 6) | ((uint8_t) !IMD_OK << 5) | ((uint8_t) (!(ESDB_flag) || !(ESDB_charge_flag)) << 4) | ((uint8_t) TSMS_flag << 3) |
							((uint8_t) !coms_slave << 2) | ((uint8_t) !BMS_OK << 1); //TODO: porque é que aqui estão as negações em certas cenas?


					lastlast_cells_error = last_cells_error;
					last_cells_error = (OD_PERSIST_COMM.x6001_errors_State.voltage_current_vals >> 30) & 0x3;

					break;


				default:
					break;

			}


			/// UNCOMMENT TO DEBUG MESSAGE SENT ///
			/*
			msg_debug_len = sprintf(msg_debug, "[BLOCK %d]\n", blocoAtual);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg_debug, msg_debug_len, 100);

			for (int i = 0; i < 6; i++) {
				msg_debug_len = sprintf(msg_debug, "&OD_PERSIST_COMM.x6000_V_C.val1[%d] = %d \n", i, ((uint8_t*) &OD_PERSIST_COMM.x6000_V_C.val1)[i]);
				HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, msg_debug_len, 100);
			}
			*/
			//TODO: FALTA LOGICA DE PREENCHIMENTO DO TPDO
			/*para saber que variaveis estão no dicionario
			 *  OD_PERSIST_COMM.x6001_errors_State.isto é a base para aparecer o nome das variaveis é so depois quando se está a escrever carregar no ctrl + space
			 * voltage_current_vals -> var de 32 bits necessario definir os dois mais significativos
			 * temperatures -> 1 byte
			 * states -> 1 byte
			 */
			//TODO: FALTA SELECIONAR QUAL O CO_TPDOSENDREQUEST QUE SE USA CO_TPDOsendRequest(&canopenNodeSTM32.canOpenStack->TPDO[1]);
			if (contadorTicks == (NUM_TICKS - 2)){
				CO_TPDOsendRequest(&canopenNodeSTM32.canOpenStack->TPDO[1]);
			}
			else{
				CO_TPDOsendRequest(&canopenNodeSTM32.canOpenStack->TPDO[0]);
			}

			blocoAtual++;
		}

		contadorTicks++;

		if (contadorTicks >= NUM_TICKS) {
			contadorTicks = 0;
			blocoAtual = 0;
		}
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1344 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 62500 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 170 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 250000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 250000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, pre_charge_Pin|charge_Pin|ctr_discharge_Pin|BMS_charge_Pin
                          |led_red_Pin|led_blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WAKEUP_GPIO_Port, WAKEUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, measure_Pin|BMS_relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pre_charge_Pin charge_Pin ctr_discharge_Pin BMS_charge_Pin
                           led_red_Pin led_blue_Pin */
  GPIO_InitStruct.Pin = pre_charge_Pin|charge_Pin|ctr_discharge_Pin|BMS_charge_Pin
                          |led_red_Pin|led_blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKEUP_Pin */
  GPIO_InitStruct.Pin = WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KSI_monitor_Pin */
  GPIO_InitStruct.Pin = KSI_monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KSI_monitor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : status_Pin */
  GPIO_InitStruct.Pin = status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(status_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : measure_Pin BMS_relay_Pin */
  GPIO_InitStruct.Pin = measure_Pin|BMS_relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : charger_signal_Pin ESDB_charger_monitor_Pin */
  GPIO_InitStruct.Pin = charger_signal_Pin|ESDB_charger_monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESDB_monitor_Pin */
  GPIO_InitStruct.Pin = ESDB_monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ESDB_monitor_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	debounce_start = HAL_GetTick();
	debounce_pending = 1;

	/*if (GPIO_Pin == status_Pin) {
	    	if (IMD_OK == false)
	    		IMD_OK = true;
	    	else
	    		IMD_OK = false;
	    }

    if (GPIO_Pin == KSI_monitor_Pin) {
    	if (TSMS_watchdog == false)
    		TSMS_watchdog = true;
    	else
    		TSMS_watchdog = false;
    }

    if (GPIO_Pin == ESDB_monitor_Pin) {
    	if (ESDB_watchdog == false)
    		ESDB_watchdog = true;
    	else
    		ESDB_watchdog = false;
    }

    if (GPIO_Pin == charger_signal_Pin) {
    	if (charger_watchdog == false)
    		charger_watchdog = true;
    	else
    		charger_watchdog = false;
    }*/

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/*if (htim == &htim2) {
		HAL_GPIO_TogglePin(led_green_GPIO_Port, led_green_Pin);
	}*/
	if (htim == &htim4) {
		MS_send_flag = true;
	}

	if (htim->Instance == TIM6) {
		flagEnvioTPDO = 1;
	}

	if (htim == canopenNodeSTM32->timerHandle) {
		canopen_app_interrupt();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if (huart->Instance == UART4){
		msg_slave_len = Size;
		MS_receive_flag = true;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, msg_slave, SLAVE_RX_BUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
