/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "ltc_message.h"
#include <SPI.h>
#include "EthernetInterface.h"
#include "bmp3.h"

/*---------------------- Initial setup ----------------------*/
// Serial port
Serial       pc(USBTX, USBRX);
// Ticker for logging from LTC at a fixed frequency
Ticker LTC_log;
// Ticker for logging from force sensor, sending ethernet message at a fixed frequency
Ticker eth_send;
bool LTC_ticker_activated = false;
bool eth_ticker_activated = false;
void log_LTC_data(){
    LTC_ticker_activated = true;
} 
void send_eth_data(){
    eth_ticker_activated = true;
}

/*---------------------- Ethernet setup ----------------------*/
const int SERVER_PORT = 11223;
const char* SERVER_ADDRESS = "192.168.1.200";    // Adress of the other Mbed (Mbed B) ... this is pc
const int LOCAL_PORT = 11223;
const char* ip = "192.168.1.1";     // Mbed A = 1; Mbed B = 2
const char* mask = "255.255.255.0";
const char* gateway = "192.168.1.10";
EthernetInterface eth; // network stack
SocketAddress local; // local address
SocketAddress client; // client address (connection to other board)
UDPSocket server; // UDP socket (peripheral on this board)

// Messages to fill and send over ethernet
// buffer length is 6*6 + 11*8 + 14 = 138 
char send_buf[138];

// timer to include dt in messages
Timer msg_timer;


/*---------------------- IMU setup ----------------------*/
// TODO: add this so that we can collect hand-held data



/*---------------------- LTC chip setup ----------------------*/
#define LTC_MOSI    PB_2
#define LTC_MISO    PC_11
#define LTC_CLK     PC_10
#define LTC_CS      PA_15
#define BSY         PB_5
#define CNV         PB_6

#define BUFF_SIZE   18 //24   // 24, 8-bit words will come through
#define NUM_CHANNELS   6 //8   // 8 total channels
#define CH_SIZE   3   // 3 words / channel

SPI LTC_chip(LTC_MOSI, LTC_MISO, LTC_CLK);
DigitalOut cs_LTC(LTC_CS);
DigitalIn bsy_LTC(BSY);
DigitalOut CNV_PIN(CNV);

float CONV_FACTOR = 0.00031294782f; //--> for -10.24 --> 10.24V
// receive and store data from LTC chip
uint8_t rx_buff[BUFF_SIZE]; //each is an 8-bit word 
ltc_spi adc_data;
int16_t ati_data[6];
float ati_filt[6];
float FILT_COEF = 1.0f; //0.10f;

// calibration matrix for ATI sensor
float Fxc[6] = {-0.1971322934773, -0.04349257334311, 2.298051028435, -80.35044049387, 1.362983909976, 78.23673392118};
float Fyc[6] = {-0.855555082028, 90.04004739944, -0.2236363056212, -46.22515556189, 0.4634720862657, -45.33866366008};
float Fzc[6] = {126.0118743229, -3.400673797001, 125.6239720415, -3.58428375801, 124.6128824882, -3.121863244239};
float Mxc[6] = {-0.03257086475743, 1.078228404461, -4.281073433774, -0.4388170286617, 4.26206973335, 7 - 0.6391561102933}; // check that these rows are x,y,z too
float Myc[6] = {5.013689449541, -0.1348267445261, -2.487858919058, 1.036624778844, -2.465023328927, -0.8776820303935};
float Mzc[6] = {0.03045090196646, -2.681788264229, 0.06994993822276, -2.787067635975, -0.04822780843519, -2.696991001959};
// bias terms
float bias[6];
// convert forces and torques
float ft_data[6];

// send 6 configuration words with the 6 channel numbers, still the default softspan 7 conversion
void config_LTC(){
    uint8_t discard;
    cs_LTC=0;
    // set sampling order for channels 0-5
    discard = LTC_chip.write(0b10000111); // byte is 7:V, 6:0, 5-3:ID, 2-0:mode
    discard = LTC_chip.write(0b10001111);
    discard = LTC_chip.write(0b10010111);
    discard = LTC_chip.write(0b10011111);
    discard = LTC_chip.write(0b10100111);
    discard = LTC_chip.write(0b10101111);   
    cs_LTC=1;
}

void read_LTC_data() {
    
    // right now, programmed LTC chip to only sample 6 channels, starting at channel 0 on next conversion (in config_LTC)...also changed number of channels variables
    // TODO: could also decode channel ID from bits 4-6 of the info byte in message packet, assign to correct adc channel
    // TODO: could also include next channel for sampling in each conversion message
    
    // sample from all channels
    for (int i = 0; i<NUM_CHANNELS; i++) {
        //request conversion
        CNV_PIN=1;
        wait_ns(60); // wait for 60ns
        CNV_PIN=0;
        //WAIT FOR BSY --> bsy_LTC
        while(bsy_LTC==1){}
        //debugging ONLY
        wait_us(1);
        //then ask for data
        cs_LTC=0;   
        //read data
        int bytecount = CH_SIZE*i;
        while(bytecount < CH_SIZE*(1+i)){
            rx_buff[bytecount] = LTC_chip.write(0x00);
            bytecount++;
        }
        cs_LTC=1; //lift CS
    }
    // fill adc data struct with received bytes    
    for(int i = 0; i < BUFF_SIZE; i++){
        ((uint8_t*)(&adc_data))[i] = rx_buff[i];
    }
    // pull out voltage data here
    ati_data[0] = adc_data.channel[0].cnv_upper<<8 | adc_data.channel[0].cnv_lower;
    ati_data[1] = adc_data.channel[1].cnv_upper<<8 | adc_data.channel[1].cnv_lower;
    ati_data[2] = adc_data.channel[2].cnv_upper<<8 | adc_data.channel[2].cnv_lower;
    ati_data[3] = adc_data.channel[3].cnv_upper<<8 | adc_data.channel[3].cnv_lower;
    ati_data[4] = adc_data.channel[4].cnv_upper<<8 | adc_data.channel[4].cnv_lower;
    ati_data[5] = adc_data.channel[5].cnv_upper<<8 | adc_data.channel[5].cnv_lower;
}

void filter_LTC_data(){
    // filter the incoming LTC data
    for(int i=0; i<6; i++){
        ati_filt[i] = FILT_COEF*(float)ati_data[i] + (1.0-FILT_COEF)*ati_filt[i];
    }
}

// convert integer readings to voltages to f/t values
void convert_LTC_data(){
    // dummy buffer to store converted ADC vals
    float buff[6];
    for(int i=0; i<6; i++){
        //buff[i] = CONV_FACTOR*((float)ati_data[i]-bias[i]); // bias[] is in same units as msg[] // for non-filtered LTC data
        buff[i] = CONV_FACTOR*(ati_filt[i]-bias[i]); // for filtered data
        ft_data[i] = 0; // also zero out ft_data here
    }
    // convert each f/t value separately
    for(int i=0; i<6; i++){
        ft_data[0] += Fxc[i]*buff[i];   
        ft_data[1] += Fyc[i]*buff[i];  
        ft_data[2] += Fzc[i]*buff[i];  
        ft_data[3] += Mxc[i]*buff[i];  
        ft_data[4] += Myc[i]*buff[i];  
        ft_data[5] += Mzc[i]*buff[i];  
    }
}


/*---------------------- Force sensor setup ----------------------*/
#define SN_MOSI     PC_3
#define SN_MISO     PC_2
#define SN_CLK      PB_10
#define SN_CS       PB_12
#define SN_A        PG_15
#define SN_B        PG_10
#define SN_C        PG_12

SPI sn_spi(SN_MOSI, SN_MISO, SN_CLK); //Sensor SPI - mosi, miso, sclk
DigitalOut dec_enable(SN_CS);
DigitalOut dec_bit0(SN_A); 
DigitalOut dec_bit1(SN_B); 
DigitalOut dec_bit2(SN_C);

// structs for individual sensor devices
struct bmp3_dev s1; // sets up dev as a 'bmp3_dev structure' w/ associated variables
struct bmp3_dev s2;
struct bmp3_dev s3;
struct bmp3_dev s4;
struct bmp3_dev s5;
struct bmp3_dev s6;
struct bmp3_dev s7;
struct bmp3_dev s8;
// Structs for sensor data
struct bmp3_data sn_data1;
struct bmp3_data sn_data2;
struct bmp3_data sn_data3;
struct bmp3_data sn_data4;
struct bmp3_data sn_data5;
struct bmp3_data sn_data6;
struct bmp3_data sn_data7;
struct bmp3_data sn_data8;
// Store sensor output data (pressures)
int pr_data[8];
// Configure data from sensor
uint8_t sensor_comp = uint8_t(1)| uint8_t(1<<1); // sensor_comp = BMP3_PRESS | BMP3_TEMP;
    
void writeLow(uint8_t pin){ // modified for just 2 sensors
    dec_enable = 0;
    if (pin == 1){
        dec_bit0 = 0;
        dec_bit1 = 0;
        dec_bit2 = 0;
    }
    else if (pin == 2){
        dec_bit0 = 1;
        dec_bit1 = 0;
        dec_bit2 = 0;
    }
    else if (pin == 3){
        dec_bit0 = 0;
        dec_bit1 = 1;
        dec_bit2 = 0;
    }
    else if (pin == 4){
        dec_bit0 = 1;
        dec_bit1 = 1;
        dec_bit2 = 0;
    }
    else if (pin == 5){
        dec_bit0 = 0;
        dec_bit1 = 0;
        dec_bit2 = 1;
    }
    else if (pin == 6){
        dec_bit0 = 1;
        dec_bit1 = 0;
        dec_bit2 = 1;
    }
    else if (pin == 7){
        dec_bit0 = 0;
        dec_bit1 = 1;
        dec_bit2 = 1;
    }
    else if (pin == 8){
        dec_bit0 = 1;
        dec_bit1 = 1;
        dec_bit2 = 1;
    }
}

void writeHigh(){
    dec_enable = 1; // write all pins high by disabling the decoder
}

// General Read and Write functions
// read function: |0x80 done in library, dummy byte taken care of in library
static int8_t user_spi_read(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow(cspin);
    sn_spi.write(reg_addr); // send read command to chip_id register (reg 0x00)
    for(int i = 0; i < len; i++){
        *(reg_data+i) = sn_spi.write(0x00); // read in 2nd byte = chip_id
    }
    writeHigh();
    return 0;
}

static int8_t user_spi_write(uint8_t cspin, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    writeLow(cspin);
    sn_spi.write(reg_addr);
    if (len>1) {
        for(int i = 0; i < len-1; i++){
            sn_spi.write(*(reg_data+i)); // send alternating register address and register bytes in multi write
        }
    }
    else{
        sn_spi.write(reg_data[0]);
    }    
    writeHigh();
    return 0;
}

void user_delay_ms(uint32_t msec){ //delay in milliseconds
    wait_ms(msec); 
}

void config_bmp_dev(struct bmp3_dev *dev){
    int8_t rslt=0;//BMP3_OK; // get error with rslt = BMP3_OK;
    
    dev -> intf = BMP3_SPI_INTF;
    dev -> read = &user_spi_read; // what to set here??? should be pointer to read function? &spi_read
    dev -> write = &user_spi_write;// what to set here??? should be pointer to write function? &spi_write
    dev -> delay_ms = &user_delay_ms;// what to set here???  delay in ms
    rslt = bmp3_init(dev);
    pc.printf("* initialize sensor result = 0x%x *\r\n", rslt);
    wait(0.25);
    
    // ***** Configuring settings of sensor
    // Normal Mode - bmp3_set_op_mode
    // Temp En, Press En 
    // OSR = no oversampling temp, press
    // ODR = 200Hz temp, press
    // IRR = no IRR filter
    // ^^^all 4 above =  bmp3_set_sensor_settings
    
    // Set sensor settings (press en, temp en, OSR, ODR, IRR)
    dev -> settings.press_en = 0x01; // BMP3_ENABLE
    dev -> settings.temp_en = 0x01; //BMP3_ENABLE
    dev -> settings.odr_filter.press_os = 0x00; //BMP3_NO_OVERSAMPLING
    dev -> settings.odr_filter.temp_os = 0x00; //BMP3_NO_OVERSAMPLING
    dev -> settings.odr_filter.odr = 0x00; //BMP3_ODR_200_HZ
    dev -> settings.odr_filter.iir_filter = 0x00; //BMP3_IIR_Filter_disable
    
    uint16_t settings_sel;
    //settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_IIR_FILTER_SEL | BMP3_ODR_SEL;
    settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2) | uint16_t(1 << 4) | uint16_t(1 << 5) | uint16_t(1 << 6) | uint16_t(1 << 7);
    //settings_sel = uint16_t(1 << 1) | uint16_t(1 << 2);
    rslt = bmp3_set_sensor_settings(settings_sel, dev);
    
    // Set operating (power) mode
    dev -> settings.op_mode = 0x03; /// normal mode = 0x03
    rslt = bmp3_set_op_mode(dev);
    
    // Check settings
    rslt = bmp3_get_sensor_settings(dev);  
}

void read_bmp_data(){
    // read data from sensors
    bmp3_get_sensor_data(sensor_comp, &sn_data1, &s1);       
    bmp3_get_sensor_data(sensor_comp, &sn_data2, &s2);
    bmp3_get_sensor_data(sensor_comp, &sn_data3, &s3);
    bmp3_get_sensor_data(sensor_comp, &sn_data4, &s4);
    bmp3_get_sensor_data(sensor_comp, &sn_data5, &s5);
    bmp3_get_sensor_data(sensor_comp, &sn_data6, &s6);
    bmp3_get_sensor_data(sensor_comp, &sn_data7, &s7);
    bmp3_get_sensor_data(sensor_comp, &sn_data8, &s8);
}


/*---------------------- Neural Network Evaluation ----------------------*/

// TODO: add this to test fingertip sensor
/*
// sensor data
uint16_t sense_data_raw[8];
float sense_data_decode[5];

// internal layers to be evaluated on-line
float l1[12];
float l2[25];

// weights and biases for NN (constant)...add more digits here to improve prediction accuracy?
const float b1[12] = {};
const float b2[25] = {};
const float b3[5] =  {};
const float w1[8][12] = { { }, { }, { }, { }, { }, { }, { }, { } };
const float w2[12][25] = {{ }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } };                         
const float w3[25][5] = { { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { },
                          { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } };

// values for scaling sensor readings and NN outputs
const float minims[] = { -79.14952894807573f, -59.34503867427718f, -190.91288f,  -0.7863755477785648f, -0.7863755477785648f, -831.0f, -997.0f, -974.0f, -986.0f, -998.0f, -964.0f, -136.0f, -881.0f};
const float maxims[] = { 145.75214841512442f, 147.80758667427716f,  194.5331016f, 1.5781267096532723f,  1.5765908199115173f, 4079.0f, 3156.0f, 3734.0f, 4001.0f, 3463.0f, 3463.0f, 2621.0f, 3370.0f};
const float maxS = 4079.0;
float in_vec[8];
float out_vec[5];
int out_vec_pr[5];
uint16_t offsets[8];

// sensor workflow from before
//// get sensor data
//readSensor(sense_data_raw);
//
//// scale sensor data
//for (int i=0; i<8; i++){
//    in_vec[i] = 0.0f;
//    in_vec[i] = (((float)(sense_data_raw[i]-offsets[i])) + abs(minims[i+5])) / maxS;
//}
//
//// evaluate NN
//decodeSensor(in_vec,sense_data_decode); 
//
//// post-process, re-scale decoded data
//for (int i=0; i<5; i++) {
//    out_vec[i] = 0.0f;
//    out_vec[i] = (sense_data_decode[i]*maxims[i]) - abs(minims[i]);
//    out_vec_pr[i] = (int)(out_vec[i]*100.0f);
//}    


void calibrateSensor(uint16_t* offsets){ // not sure if this is still necessary?
    // calculate sensor offsets
    float temp_offsets[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int samps = 10;
    for (int i=0; i<samps; i++){
        for (int j=0; j<8; j++){
            temp_offsets[j] += (float)spi3.binary(j);
        }
        wait_ms(1);
    }
    for (int i=0; i<8; i++){
        temp_offsets[i] = temp_offsets[i]/((float)samps); // get overall offset
        offsets[i] = (uint16_t)temp_offsets[i]; // convert to int
    }
}

void decodeSensor(float input[8], float* output) {    
    // decode sensor data here....521*4 operations (multiply,add,activation,add)
    
    // TODO: do normalization here? would need to change input[8] to raw sensor data for function input
    
    
    // reset values
    for (int i = 0; i<12; i++){
        l1[i] = 0.0f;
    }
    for (int i = 0; i<25; i++){
        l2[i] = 0.0f;
    }
    for (int i = 0; i<5; i++){
        output[i] = 0.0f;
    }
        
    // layer 1
    for(int i = 0; i<12; i++){ // for each node in the next layer
        for(int j = 0; j<8; j++){ // add contribution of node in prev. layer
            l1[i] +=  (w1[j][i]*input[j]); 
        }
        l1[i] += b1[i]; // add bias
        l1[i] = fmaxf(0.0f, l1[i]); // relu activation
    }
        
    // layer 2
    for(int i = 0; i<25; i++){ // for each node in the next layer
        for(int j = 0; j<12; j++){ // add contribution of node in prev. layer
            l2[i] += (w2[j][i]*l1[j]);
        }
        l2[i] += b2[i]; // add bias
        if (l2[i]<0.0f) { // elu activation
            l2[i] = exp(l2[i]) - 1.0f; // alpha implicitly set to 1.0 here
        }
    }   
    
    // layer 3
    for(int i = 0; i<5; i++){ // for each node in the next layer
        for(int j = 0; j<25; j++){ // add contribution of node in prev. layer
            output[i] += w3[j][i]*l2[j];
            
        }
        output[i] += b3[i];// add bias
        output[i] = 1.0f/(1.0f + exp(-output[i])); // sigmoid activation 
    }  
    
    // TODO: undo normalization here
    
    }
*/



/*---------------------- Main function ----------------------*/
int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    CNV_PIN=0;
    
    // Set up serial port
    pc.baud(115200);
    pc.printf("Initializing.\n\r");
    
    // Set up LTC chip
    LTC_chip.format(8, 0);
    LTC_chip.frequency(2000000); //10MHz? //60Mhz clock frequency 
    wait_ms(1);
    config_LTC(); // programs sample order
    wait_ms(1);
    
    // Set up ethernet
    eth.set_network(ip, mask, gateway);
    eth.connect();
    wait_ms(1);
    client.set_port(SERVER_PORT);
    client.set_ip_address(SERVER_ADDRESS);
    local.set_port(LOCAL_PORT);
    local.set_ip_address(ip);
    int code = server.open(&eth);
    if(code!=0) { pc.printf("Error from opening server = %d\n\r",code); }    
    code = server.bind(local);
    if(code!=0) { pc.printf("Error from binding socket = %d\n\r",code); }
    
    // Set up force sensor and decoder
    dec_enable = 1;    
    // Initialize bmp device 1
    s1.dev_id = 1;  // tells which cs pin associated with device
    config_bmp_dev(&s1);
    //Initialize bmp device 2
    s2.dev_id = 2;  // tells which cs pin associated with device
    config_bmp_dev(&s2);
    //Initialize bmp device 3
    s3.dev_id = 3;  // tells which cs pin associated with device
    config_bmp_dev(&s3);
    //Initialize bmp device 4
    s4.dev_id = 4;  // tells which cs pin associated with device
    config_bmp_dev(&s4);
    //Initialize bmp device 5
    s5.dev_id = 5;  // tells which cs pin associated with device
    config_bmp_dev(&s5);
    //Initialize bmp device 6
    s6.dev_id = 6;  // tells which cs pin associated with device
    config_bmp_dev(&s6);
    //Initialize bmp device 7
    s7.dev_id = 7;  // tells which cs pin associated with device
    config_bmp_dev(&s7);
    //Initialize bmp device 8
    s8.dev_id = 8;  // tells which cs pin associated with device
    config_bmp_dev(&s8);
    
    // Calculate bias voltages for ATI sensor
    for(int i=0; i<100; i++){ // read 100 times and take average
        read_LTC_data();  
        for(int j=0; j<6; j++){
            bias[j] += 0.01*(float)ati_data[j];
        }
        wait_ms(1);
    }
    pc.printf("ATI sensor is calibrated with biases of: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n\r", bias[0],bias[1],bias[2],bias[3],bias[4],bias[5]);
     
    // TODO: Add calibration for force sensor? Only necessary if the force sensor neural network is being evaluated online
    //calibrateSensor(offsets);
      
      
    // Attach sampling and sending interrupts
    pc.printf("Beginning to sample.\n\r");    
    LTC_log.attach_us(&log_LTC_data,1000); // 1kHz // 1000us = 1ms (10000 = 10 ms = 100 Hz)
    eth_send.attach_us(&send_eth_data,10000); // 100Hz
    msg_timer.start();
    
    while (true) {
        
        /*if(LTC_ticker_activated == true) {
            // Clear flag
            LTC_ticker_activated = false;
            // Sample from LTC chip, filter data
            read_LTC_data();
            filter_LTC_data();
        }   
        
        if(eth_ticker_activated == true) { 
            // Clear flag
            eth_ticker_activated = false;
            
            convert_LTC_data(); // TODO: maybe remove this eventually, if we convert the data after sending over ethernet
            // Print received data
            //pc.printf("%6d,%6d,%6d,%6d,%6d,%6d\n\r", ati_data[0],ati_data[1],ati_data[2],ati_data[3],ati_data[4],ati_data[5]);
            // Print converted LTC data
//            pc.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,  ", ft_data[0],ft_data[1],ft_data[2],ft_data[3],ft_data[4],ft_data[5]);
            
            // Sample from force sensor
            read_bmp_data();
            // Convert to relative pressures
            pr_data[0] = int(sn_data1.pressure)-100000; // pressure is returned in Pa, could subtract actual sea level pressure here
            pr_data[1] = int(sn_data2.pressure)-100000;
            pr_data[2] = int(sn_data3.pressure)-100000;
            pr_data[3] = int(sn_data4.pressure)-100000;
            pr_data[4] = int(sn_data5.pressure)-100000;
            pr_data[5] = int(sn_data6.pressure)-100000;
            pr_data[6] = int(sn_data7.pressure)-100000;
            pr_data[7] = int(sn_data8.pressure)-100000;
            // Print received data
//            pc.printf("%d,%d,%d,%d,%d,%d,%d,%d\n\r", pr_data[0],pr_data[1],pr_data[2],pr_data[3],pr_data[4],pr_data[5],pr_data[6],pr_data[7]);      
            
            // Pack and send the ethernet message
            //sprintf(send_buf, "%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d\n", msg_timer.read(),ft_data[0],ft_data[1],ft_data[2], pr_data[0],pr_data[1],pr_data[2],pr_data[3],pr_data[4],pr_data[5],pr_data[6],pr_data[7]);       
//            sprintf(send_buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", ati_data[0],ati_data[1],ati_data[2],ati_data[3],ati_data[4],ati_data[5], pr_data[0],pr_data[1],pr_data[2],pr_data[3],pr_data[4],pr_data[5],pr_data[6],pr_data[7]);       
            //server.sendto(client, send_buf, sizeof(send_buf)); // send message, look for '\n' character when decoding the string
            
         }*/
         
         // Receive a message from server (Mbed B)
        char buf[256];
        int n = server.recvfrom(&client, buf, 256); // could also use server.recv(buf, 256)?
        buf[n] = '\0';
        //pc.printf("Received message back from server: '%s'\n\r", buf);  
        
        if (buf[0]=='z') { // re-zero ATI sensor        
            for(int i=0; i<100; i++){ // read 100 times and take average
                read_LTC_data();  
                for(int j=0; j<6; j++){
                    bias[j] += 0.01*(float)ati_data[j];
                }
                wait_ms(1);
            }
        }
        
        if (buf[0]=='r') {
            read_LTC_data();
            filter_LTC_data();
            convert_LTC_data();
            read_bmp_data();
            pr_data[0] = int(sn_data1.pressure)-100000; // pressure is returned in Pa, could subtract actual sea level pressure here
            pr_data[1] = int(sn_data2.pressure)-100000;
            pr_data[2] = int(sn_data3.pressure)-100000;
            pr_data[3] = int(sn_data4.pressure)-100000;
            pr_data[4] = int(sn_data5.pressure)-100000;
            pr_data[5] = int(sn_data6.pressure)-100000;
            pr_data[6] = int(sn_data7.pressure)-100000;
            pr_data[7] = int(sn_data8.pressure)-100000;    
            sprintf(send_buf, "%f,%f,%f,%f,%f, %f,%f, %d,%d,%d,%d,%d,%d,%d,%d\n", msg_timer.read(),ft_data[0],ft_data[1],ft_data[2],ft_data[3],ft_data[4], ft_data[5], pr_data[0],pr_data[1],pr_data[2],pr_data[3],pr_data[4],pr_data[5],pr_data[6],pr_data[7]);       
            server.sendto(client, send_buf, sizeof(send_buf)); // send message, look for '\n' character when decoding the string
        } 
        
    }
        
    // Terminate connection (if you want)
    server.close();
    eth.disconnect();
    
}
