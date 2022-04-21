#include <Arduino.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif
#define BAUDRATE_DXL 57600

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 12;
const uint8_t DXL_ID_LIST[12] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data
{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data
{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

DynamixelShield dxl;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

long timer = 0;
long previous_timer = 0;

// single motor

int32_t goal_position1[6] = {0, 1024, -2048, 3072, 4095, 6000};
int32_t goal_position2[6] = {0, -1024, -2048, 2048, 4096, 12000};
uint8_t goal_position_index = 0;
int goal_position = 0;
int present_position = 0;
bool goal_achieved = 0;

// Multi_motor
int goal_position_sequence1[12] = {1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
int goal_position_sequence2[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int goal_position_sequence_repos[12] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};

int goal_position_sequence_coucou1[12] = {50, 50, 50, -1024, 1024, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou2[12] = {50, 50, 50, -2048, 1024, 50, 50, 50, 50, 50, 50, 50};
int goal_position_sequence_coucou3[12] = {50, 50, 50, -1024, 2028, 50, 50, 50, 50, 50, 50, 50};

int goal_position_tab[12];
int present_position_tab[12];
bool goal_achieved_tab[12];
int goal_achieved_result = 0;

// le tableau doit acceillir tout les goal pour chacun des moteurs.
void go_to_positions_multiple_motors(int *tab)
{
  uint8_t i, recv_cnt;

  // Insert a new Goal Position to the SyncWrite Packet
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    sw_data[i].goal_position = tab[i];
  }
  sw_infos.is_info_changed = true;

  /**
   * Sync Write, changement de la valeur de goal_position en fonction du tableau en entrée
   */
  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    for (i = 0; i < sw_infos.xel_count; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print(" Goal Position: ");
      DEBUG_SERIAL.println(sw_data[i].goal_position);
      goal_position_tab[i] = sw_data[i].goal_position;
    }
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }

  DEBUG_SERIAL.println();

  previous_timer = millis();

  /**
   * Sync read, On releve l'information de la position. Lorsque l'objectif est atteint (goal-present < 10) pour chacun des moteurs on peut passer à un prochain mouvement
   */

  while (goal_achieved == 0)
  {
    recv_cnt = dxl.syncRead(&sr_infos);
    if (recv_cnt > 0)
    {
      for (i = 0; i < recv_cnt; i++)
      {
        present_position_tab[i] = sr_data[i].present_position;
        /*
        DEBUG_SERIAL.print("  ID: ");
        DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
        DEBUG_SERIAL.print(" Present Position: ");
        DEBUG_SERIAL.print(present_position);
        DEBUG_SERIAL.print(" Goal Position: ");
        DEBUG_SERIAL.println(goal_position);
        */

        if (abs(goal_position_tab[i] - present_position_tab[i]) < 20)
        {

          DEBUG_SERIAL.print(" ID: ");
          DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
          DEBUG_SERIAL.print(" Goal achieved with error: ");
          DEBUG_SERIAL.println(abs(goal_position_tab[i] - present_position_tab[i]));

          goal_achieved_tab[i] = 1;
          // delay(100);
        }
        if (goal_achieved_tab[0] == 1 && goal_achieved_tab[1] == 1 && goal_achieved_tab[3] && goal_achieved_tab[4] && goal_achieved_tab[5] && goal_achieved_tab[6] && goal_achieved_tab[7] && goal_achieved_tab[8] && goal_achieved_tab[9] && goal_achieved_tab[10] && goal_achieved_tab[11] && goal_achieved_tab[12])
        {
          DEBUG_SERIAL.print(" finished ");
          goal_achieved = 1;
        }
      }
    }
  }

  /**
   * Affichage des résultats et mise à zero des variable de comptage.
   *
   */
  for (int i = 0; i < DXL_ID_CNT; i++)
  {
    goal_achieved_tab[i] = 0;
  }
  goal_achieved = 0;

  DEBUG_SERIAL.print(" Goal achieved timer: ");
  DEBUG_SERIAL.println(millis() - previous_timer);

  for (i = 0; i < sw_infos.xel_count; i++)
  {
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
    DEBUG_SERIAL.print(" final Position: ");
    DEBUG_SERIAL.print(sr_data[i].present_position);
    DEBUG_SERIAL.print(" error Position: ");
    DEBUG_SERIAL.println(abs(sr_data[i].present_position - sw_data[i].goal_position));
  }

  DEBUG_SERIAL.println("=======================================================");
}

void go_to_position(int position, int ID_moteur)
{
  uint8_t i, recv_cnt;
  // ID_moteur--;

  // Insert a new Goal Position to the SyncWrite Packet
  sw_data[ID_moteur].goal_position = position;
  sw_infos.is_info_changed = true;

  /**
   * Sync Write, changement de la valeur de goal_position en fonction du tableau en entrée
   */

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(sw_infos.p_xels[ID_moteur].id);
    DEBUG_SERIAL.print(" Goal Position: ");
    goal_position = sw_data[ID_moteur].goal_position;
    DEBUG_SERIAL.println(sw_data[ID_moteur].goal_position);
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  previous_timer = millis();

  /**
   * Sync read, On releve l'information de la position. Lorsque l'objectif est atteint (goal-present < 10) pour chacun des moteurs on peut passer à un prochain mouvement
   */

  while (goal_achieved == 0)
  {
    recv_cnt = dxl.syncRead(&sr_infos);
    if (recv_cnt > 0)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[ID_moteur].id);
      DEBUG_SERIAL.print(" Present Position: ");
      DEBUG_SERIAL.print(present_position);
      DEBUG_SERIAL.print(" Goal Position: ");
      DEBUG_SERIAL.println(goal_position);
      present_position = sr_data[ID_moteur].present_position;

      if (abs(goal_position - present_position) < 40)
      {
        DEBUG_SERIAL.print(" Goal achieved with error: ");
        DEBUG_SERIAL.print(abs(goal_position - present_position));
        goal_achieved = 1;
      }
    }
  }

  goal_achieved = 0;
  DEBUG_SERIAL.print(" Goal achieved timer: ");
  DEBUG_SERIAL.println(millis() - previous_timer);

  DEBUG_SERIAL.println("=======================================================");
}

void set_velocity_accel(int vel, int accel, int ID_moteur)
{
  ID_moteur--;
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_LIST[ID_moteur], vel);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[ID_moteur], accel);
}

void coucou()
{
  for (int i = 0; i < 12; i++)
  {
    set_velocity_accel(200, 0, i);
  }

  //delay(5000);
  go_to_positions_multiple_motors(goal_position_sequence_coucou1);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  go_to_positions_multiple_motors(goal_position_sequence_coucou2);
  go_to_positions_multiple_motors(goal_position_sequence_coucou3);
  // go_to_positions_multiple_motors(goal_position_sequence_coucou1);
  //delay(2000);
}

void setup()
{
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(BAUDRATE_DXL);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  for (i = 0; i < DXL_ID_CNT; i++)
  {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_EXTENDED_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t *)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
 
  for (int i = 0; i < 12; i++)
  {
    go_to_position(goal_position_sequence_repos[i], i);
  }

  coucou();

  go_to_positions_multiple_motors(goal_position_sequence_repos);


}

void loop()
{

  /*
   go_to_positions_multiple_motors(goal_position_sequence1);
   delay(1000);
   go_to_positions_multiple_motors(goal_position_sequence2);
   delay(1000);
   */
}