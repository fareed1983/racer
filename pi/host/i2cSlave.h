#ifndef __I2C_SLAVE__
#define __I2C_SLAVE__

bool slaveInit();
int slaveRead(uint8_t *buf, size_t bufSize);
void slaveClose();


#endif