/*
 * File:   main.c
 * Author: ganga
   / ___|  / \  | \ | |   | |_ _|
  | |  _  / _ \ |  \| |_  | || | 
  | |_| |/ ___ \| |\  | |_| || | 
   \____/_/   \_\_| \_|\___/|___| 
 * Created on January 14, 2025, 6:19 PM
 */


#include <16F1789.h>
#FUSES NOWDT NOBROWNOUT
#use delay(clock=16Mhz, crystal=16Mhz)

#use rs232(baud=9600, parity=N, xmit=PIN_B1, rcv=PIN_B2, bits=8, stream=EXT) //COM DATA ACQUISITION
#use spi(MASTER, CLK=PIN_C3, DI=PIN_C4, DO=PIN_C5,  BAUD=10000, BITS=8, STREAM=COM_FM, MODE=0) //COM shared flash memory port

char bichigcom[24] = "test data update of COM"; //test data for testing 

//mt25q flash memory command assigment
#define READ_ID              0x9F
#define READ_STATUS_REG      0x05 
#define READ_DATA_BYTES      0x13  //0x03 for byte
#define ENABLE_WRITE         0x06
#define WRITE_PAGE           0x12  //0x02 for 3byte 
#define ERASE_SECTOR         0xDC  //0xD8 for 3byte
#define ERASE_4KB_SUBSECTOR  0x21
#define ERASE_32KB_SUBSECTOR 0x5C
#define DIE_ERASE            0xC4
#define FAST_READ            0x0B

//Flash memory chip select pins and mux control  
#define CS_PIN_COM          PIN_D6  //COM_CHIP_SELECT
#define CS_PIN_COM_SHARED   PIN_C2  //COM_CHIP_SELECT
#define CS_PIN_RFM          PIN_D7  // Chip Select Pin for RFM

//RFM98PW  command assigment
#define REG_OPMODE         0x01
#define REG_BITRATE_MSB    0x02
#define REG_BITRATE_LSB    0x03
#define REG_FRF_MSB        0x06
#define REG_FRF_MID        0x07
#define REG_FRF_LSB        0x08
#define REG_PA_CONFIG      0x09
#define REG_OOK_PEAK       0x14
#define REG_OOK_FIX        0x15
#define REG_FIFO           0x00
#define REG_IRQ_FLAGS1     0x3E
#define REG_IRQ_FLAGS2     0x3F
#define TEMP_R             0x3c


void startup_freeze(){
    delay_ms(2000);
    fprintf(EXT, "POWER ON!\n");
    }

void READ_CHIP_ID_OF_COM() {
    int8 chip_id[8];
    output_low(CS_PIN_COM);  // Lower the CS PIN
    spi_xfer(COM_FM, READ_ID);  // READ ID COMMAND (0x9F)
    
    // Receive 8 bytes of chip ID
    for (int i = 0; i < 8; i++) {
        chip_id[i] = spi_xfer(COM_FM, 0x00);  // Send dummy bytes to receive data
        fprintf(EXT, "%02X ", chip_id[i]);
    }
    fprintf(EXT,"\n");

    output_high(CS_PIN_COM);  // Raise CS PIN back
}
void READ_CHIP_ID_OF_COM_SHARED() {
    int8 chip_id[8];
    output_low(CS_PIN_COM_SHARED);  // Lower the CS PIN
    spi_xfer(COM_FM, READ_ID);  // READ ID COMMAND (0x9F)
    
    // Receive 8 bytes of chip ID
    for (int i = 0; i < 8; i++) {
        chip_id[i] = spi_xfer(COM_FM, 0x00);  // Send dummy bytes to receive data
        fprintf(EXT, "%02X ", chip_id[i]);
    }
    fprintf(EXT,"\n");
    output_high(CS_PIN_COM_SHARED);  // Raise CS PIN back
}

void READ_TEMP_RFM(void){
    int8 regVal;
    int8 temp;
    output_low(CS_PIN_RFM);  // Lower the CS PIN
    spi_xfer(COM_FM, TEMP_R);  // READ ID COMMAND (0x9F)
    
    // Receive 8 bytes of chip ID
    for (int i = 0; i < 1; i++) {
        regVal = spi_xfer(COM_FM, 0x00);  // Send dummy bytes to receive data
        fprintf(EXT, "%02X ", regVal);
    }
    fprintf(EXT,"\n");
    temp = regVal & 0x7f;
    
    fprintf(EXT,"temperature is now:%d\n", temp);
    output_high(CS_PIN_RFM);  // Raise CS PIN back
}

// Helper Functions
void RFM_Write_Register(unsigned int8 reg, unsigned int8 value) {
    output_low(CS_PIN_RFM);
    spi_xfer(reg | 0x80);  // Write command (MSB set)
    spi_xfer(value);
    output_high(CS_PIN_RFM);
}

unsigned int8 RFM_Read_Register(unsigned int8 reg) {
    unsigned int8 value;
    output_low(CS_PIN_RFM);
    spi_xfer(reg & 0x7F);  // Read command (MSB cleared)
    value = spi_read(0x00);
    output_high(CS_PIN_RFM);
    return value;
}

// Initialize RFM for OOK Communication
void RFM_OOK_Init(unsigned int8 frequency) {
    // Enter Sleep Mode
    RFM_Write_Register(REG_OPMODE, 0x00);
    delay_ms(10);
    
    // Set OOK Modulation
    RFM_Write_Register(REG_OPMODE, 0x20);  // OOK mode
    
    // Set Bit Rate (4.8 kbps example)
    RFM_Write_Register(REG_BITRATE_MSB, 0x1A);
    RFM_Write_Register(REG_BITRATE_LSB, 0x0B);
    
    // Set Frequency (Calculate FRF value)
    unsigned int8 frf = (unsigned int8)((frequency * 524288UL) / 32000000UL);
    RFM_Write_Register(REG_FRF_MSB, (frf >> 16) & 0xFF);
    RFM_Write_Register(REG_FRF_MID, (frf >> 8) & 0xFF);
    RFM_Write_Register(REG_FRF_LSB, frf & 0xFF);
    
    // Configure OOK Parameters
    RFM_Write_Register(REG_OOK_PEAK, 0x28);  // Peak mode, 0.5dB step
    RFM_Write_Register(REG_OOK_FIX, 0x0C);   // Fixed threshold
    
    // Configure PA Settings (+17dBm output)
    RFM_Write_Register(REG_PA_CONFIG, 0x8F);
    
    // Enter Standby Mode
    RFM_Write_Register(REG_OPMODE, 0x01);
    delay_ms(10);
}

// Transmit OOK Data
void RFM_OOK_Transmit(unsigned int8 *data, unsigned int8 length) {
    // Enter TX Mode
    RFM_Write_Register(REG_OPMODE, 0x03);
    
    // Wait for TX Ready
    while(!(RFM_Read_Register(REG_IRQ_FLAGS1) & 0x08));
    
    // Write Data to FIFO
    RFM_Write_Register(REG_FIFO, length);
    for(unsigned int8 i = 0; i < length; i++) {
        RFM_Write_Register(REG_FIFO, data[i]);
    }
    
    // Wait for Transmission Complete
    while(!(RFM_Read_Register(REG_IRQ_FLAGS2) & 0x08));
    
    // Return to Standby
    RFM_Write_Register(REG_OPMODE, 0x01);
}

// Basic Receive Example (Polling Mode)
unsigned int8 RFM_OOK_Receive(unsigned int8 *buffer) {
    unsigned int8 length;
    
    // Enter RX Mode
    RFM_Write_Register(REG_OPMODE, 0x05);
    
    // Wait for Payload Ready
    while(!(RFM_Read_Register(REG_IRQ_FLAGS2) & 0x04));
    
    // Read Received Data
    length = RFM_Read_Register(0x13);  // Read payload length
    RFM_Read_Register(REG_FIFO);       // Dummy read
    for(unsigned int8 i = 0; i < length; i++) {
        buffer[i] = RFM_Read_Register(REG_FIFO);
    }
    
    // Return to Standby
    RFM_Write_Register(REG_OPMODE, 0x01);
    
    return length;
}



void main(void) {
    
    startup_freeze();
//    RTC_initialize();
    int extdatabuff[9];
    unsigned int8 tx_data[] = {0xAA, 0x55, 0x01, 0x02};
    unsigned int8 rx_buffer[64];
    unsigned int8 rx_length;
    
    RFM_OOK_Init(434000000);  // Initialize at 434MHz
    fprintf(EXT, "Reading chip ID of COM\n");
    READ_CHIP_ID_OF_COM();
    delay_ms(1000);
    fprintf(EXT, "Reading chip ID of COM SHARED\n");
    READ_CHIP_ID_OF_COM_SHARED();
    fprintf(EXT, "Done reading chip ID of COM system!\n");
    delay_ms(1000);
    fprintf(EXT, "Starting RFM module initialization\n");
    RFM_OOK_Init(434000000);  // Initialize at 434MHz
    READ_TEMP_RFM();
    fprintf(EXT, "Starting RFM module initialization Done!\n");
    
    while(1){
        delay_ms(1000);
        fprintf(EXT, "data transmiteding! \n");
        // Transmit Example
        RFM_OOK_Transmit(tx_data, sizeof(tx_data));
        fprintf(EXT, "data transmited \n");
        delay_ms(1000);
        
        // Receive Example
        rx_length = RFM_OOK_Receive(rx_buffer);
        // Process received data..
    if(kbhit(EXT)){
        fprintf(EXT, "Keyboard hit detected!\n");
        for (int i=0; i<9; i++){
            extdatabuff[i] = getc(ext);
            putc(extdatabuff[i], ext);
        }
        for (int i=0; i<9; i++){
            fprintf(EXT, "%02c", extdatabuff[i]);
        }
        fprintf(EXT, "\n");
        fprintf(EXT, "Finished!\n");
        fprintf(EXT, "Displaying RF Parameters\n");
       
        
        
    }
    }
}
