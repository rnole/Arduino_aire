
#define CO2_TX    A0  
#define CO2_RX    A1  
#define DATAPIN    2
#define CLOCKPIN   3
#define O3_TX      7
#define O3_RX      6
#define NO2_TX     9     
#define NO2_RX     8
#define SO2_TX    11
#define SO2_RX    10
#define CO2_TX    A0
#define CO2_RX    A1
#define PM_TX     A2
#define PM_RX     A3
#define NO_ANALOG A7

typedef struct{
  uint8_t frame_length[2];
  uint8_t data1[2];
  uint8_t data2[2];
  uint8_t data3[2];
  uint8_t data4[2];
  uint8_t data5[2];
  uint8_t data6[2];
  uint8_t data7[2];
  uint8_t data8[2];
  uint8_t data9[2];
  uint8_t data10[2];
  uint8_t data11[2];
  uint8_t data12[2];
  uint8_t data13[2];
  uint8_t check_code[2];
}PMS7003_struct;

#define TIMEOUT_VALUE         10000
#define WAIT_ONE_MINUTE       60000
#define WAIT_TWELVE_SECONDS    4000 
#define FIVE_MINUTES         300000 
