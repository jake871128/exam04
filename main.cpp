#include "mbed.h"
#include "mbed_rpc.h"
RawSerial pc(USBTX, USBRX);
RawSerial xbee(D12, D11);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue2;
Thread t;
float velocity = 0;
int tmp = 0;
float xvalue[15];

#define UINT14_MAX        16383
// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
// FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7
I2C i2c( PTD9,PTD8);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);
void threeaxis();


void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

void acce_value(Arguments *in, Reply *out);
RPCFunction rpc_acce(&acce_value, "acce_value");



Timer t1;
Timer t2;



  
Thread tt;

int main(){

  tt.start(threeaxis);

  pc.baud(9600);
  char xbee_reply[4];
  // XBee setting
  xbee.baud(9600);
  xbee.printf("+++");
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
  }
  xbee.printf("ATMY 0x240\r\n");
  reply_messange(xbee_reply, "setting MY : <REMOTE_MY>");
  xbee.printf("ATDL 0x241\r\n");
  reply_messange(xbee_reply, "setting DL : <REMOTE_DL>");
  xbee.printf("ATID 0x0\r\n");
  reply_messange(xbee_reply, "setting PAN ID : <PAN_ID>");
  xbee.printf("ATWR\r\n");
  reply_messange(xbee_reply, "write config");
  // xbee.printf("ATMY\r\n");
  // check_addr(xbee_reply, "MY");
  // xbee.printf("ATDL\r\n");
  // check_addr(xbee_reply, "DL");
  xbee.printf("ATCN\r\n");
  reply_messange(xbee_reply, "exit AT mode");
  xbee.getc();
  // start
  pc.printf("start\r\n");



  t.start(callback(&queue, &EventQueue::dispatch_forever));
  // Setup a serial interrupt function of receiving data from xbee
  
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);

}


void threeaxis(){

   t1.start(); 
   pc.baud(9600);
   t2.start();

   uint8_t who_am_i, data[2], res[6];
   int16_t acc16;
   float t[3];

   // Enable the FXOS8700Q
   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
   data[1] |= 0x01;
   data[0] = FXOS8700Q_CTRL_REG1;

   FXOS8700CQ_writeRegs(data, 2);
   // Get the slave address
   FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);

  
  

  while(true){

      tmp = tmp + 1; 
      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);


      acc16 = (res[0] << 6) | (res[1] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[0] = ((float)acc16) / 4096.0f;


      acc16 = (res[2] << 6) | (res[3] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[1] = ((float)acc16) / 4096.0f;


      acc16 = (res[4] << 6) | (res[5] >> 2);

      if (acc16 > UINT14_MAX/2)

         acc16 -= UINT14_MAX;

      t[2] = ((float)acc16) / 4096.0f;

      xvalue[tmp] = t[0];
      pc.printf("tmp = %d, xdata = %1.4f\r\n", tmp ,xvalue[tmp]);
      wait(0.1);

      if(tmp == 10){

        int j ;
        for(j = 1 ; j <= 10 ; j++){   
          velocity = (xvalue[j]*0.1) + velocity; 
        }

        velocity = velocity / 10.0;
        tmp = 0 ;
        pc.printf("velocity = %1.4f\r\n\n", velocity);
      } 
      //pc.printf("X=%1.4f Y=%1.4f Z=%1.4f\r\n", t[0], t[1], t[2]);
      
      
  }
 
}




void xbee_rx_interrupt(void)
{
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}
void xbee_rx(void)
{
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    memset(buf, 0, 100);
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    
    RPC::call(buf, outbuf);
    // pc.printf("%s\r\n", outbuf);
    wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}
void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}
void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}

void acce_value(Arguments *in, Reply *out){
 
  uint8_t who_am_i, data[2], res[6];
  int16_t acc16;
  float t[3];
  // Enable the FXOS8700Q
  FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
  data[1] |= 0x01;

  data[0] = FXOS8700Q_CTRL_REG1;
  //data[0] = datacount;

  FXOS8700CQ_writeRegs(data, 2);
  FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
  FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
  acc16 = (res[0] << 6) | (res[1] >> 2);
  if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
  t[0] = ((float)acc16) / 4096.0f;
  acc16 = (res[2] << 6) | (res[3] >> 2);
  if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
  t[1] = ((float)acc16) / 4096.0f;
  acc16 = (res[4] << 6) | (res[5] >> 2);
  if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
  t[2] = ((float)acc16) / 4096.0f;


  //xbee.printf("X=%1.4f Y=%1.4f Z=%1.4f\r\n", t[0], t[1], t[2]);

  xbee.printf("velocity = %1.4f\r\n",velocity);

}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}
void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}