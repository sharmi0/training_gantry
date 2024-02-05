#include "mbed.h"
#include "crc.h"
#include "string.h"
#include "dynamixel_XM430.h"
#include "math.h"
#include <cmath>
#include <cstdint>
#include "actuator_transformation.h"
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "ltc_message.h"
#include <SPI.h>
#include "EthernetInterface.h"


using namespace std;


/*---------------------- Ethernet setup ----------------------*/
const int SERVER_PORT = 11223; 
const char* SERVER_ADDRESS = "192.168.1.200";    //This is the address of the PC
const int LOCAL_PORT = 11223;
const char* ip = "192.168.1.1";     //This is the ipo f the mbed board
SocketAddress ip_1 = "192.168.1.1";     //This is the ip of the mbed board
SocketAddress mask = "255.255.255.0"; 
SocketAddress gateway = "192.168.1.10";
EthernetInterface eth; // network stack
SocketAddress local; // local address 
SocketAddress client; // client address (address of PC))
UDPSocket server; // UDP socket (peripheral on this board)

// Messages to fill and send over ethernet
// buffer length is 6*6 + 11*8 + 14 = 138
char send_buf[138];
char ok_response_buf[10];
char recv_buf[256];
int n;
int code;

//ethernet timer
Ticker eth_send;
bool eth_ticker_activated = false;
void send_eth_data(){
    eth_ticker_activated = true;
}

// timer to include dt in messages
Timer msg_timer;


/*DYNAMIXEL SETUP*/
#define WAIT_TIME_MS 1
#define LEN 100 
#define wait_ms(x) wait_us(x*1000)
#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)

bool state_change = false;
bool servo_on = false;

RawSerial uart(PF_7, PF_6);
DigitalInOut RTS(PF_9);



uint8_t dxl_ID[] =  {1, 2};
uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);

// Debugging LEDs
// DigitalOut led_pwr(LED1);
// DigitalOut led_mot(LED2);
// DigitalOut led_com(LED3);

// Initialize serial port
RawSerial pc(USBTX, USBRX, 115200);
Timer t;
int loop_time;
int servo_time;
int* dxl_commands; //x, y, z, theta, phi

double dt = 0.002; //2ms
uint32_t time_step = 0;
Ticker motor_cmd;
int8_t motor_cmd_flag = 0;
int32_t dxl_time;

// Initial Positions

uint32_t multiHomePos[2] = {2048, 2048};
uint32_t rtPos[2] = {0,0};

float pulse_to_rad = (2.0f*PI)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*PI)/60.0f; // = 0.0239

double current_limit = 1000;

int32_t dxl_position[2];
int32_t dxl_velocity[2];
int16_t dxl_current[2];
double desired_current[2];

uint16_t current_command[2];

XM430_bus dxl_bus(2000000, PF_7, PF_6, PF_9); // baud, tx, rx, rts


//Unpack Ethernet Commands
int* charToIntArray(char buf[]) {
	char *p;
	static int intArray[5];
	char temp[10];
	int tempIndex = 0;
	int intArrayIndex = 0;
	int charArrayLen = strlen(buf);

	// Empty the temp char array
	for(int j=0; j<10; j++) {
		temp[j] = ' ';
	}

	for(int i=0; i<charArrayLen; i++) {
		// Meets a space char
		if(buf[i] == ' ') {
			// Convert the temp char array into an integer, and add the integer into integer array
			intArray[intArrayIndex] = (int) strtol(temp, &p, 10);
			intArrayIndex += 1;

			// Empty the temp char array
			for(int j=0; j<10; j++) {
				temp[j] = ' ';
			}
			tempIndex = 0;
		}
		else {
			// Record into temp char array
			temp[tempIndex] = buf[i];
			tempIndex += 1;
		}
	}
	return intArray;

}



int main() {

    wait_us(10000);
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetTorqueEn(dxl_ID[i],0x00);
        dxl_bus.SetRetDelTime(dxl_ID[i],0x05); // 4us delay time?
        dxl_bus.SetControlMode(dxl_ID[i], POSITION_CONTROL);
        wait_ms(100);
        dxl_bus.TurnOnLED(dxl_ID[i], 0x01);
        //dxl_bus.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
        dxl_bus.SetTorqueEn(dxl_ID[i],0x01); //to be able to move 
        wait_ms(100);
        }
    for (int i=0; i<idLength; i++) {
        dxl_bus.SetVelocityProfile(dxl_ID[i], 0); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
        dxl_bus.SetAccelerationProfile(dxl_ID[i], 0); // 80(17166) rev/min^2
        }

    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, multiHomePos);
    wait_ms(2000);
    dxl_bus.SetMultGoalPositions(dxl_ID, idLength, rtPos);
    wait_ms(2000);
    dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
    pc.printf("%d, %d\n\r",dxl_position[0],dxl_position[1]);

//     //Set up Ethernet
    eth.set_network(ip_1, mask, gateway);
    eth.connect();
 

    wait_us(10000);
    client.set_port(SERVER_PORT);
    client.set_ip_address(SERVER_ADDRESS);
    local.set_port(LOCAL_PORT);
    local.set_ip_address(ip);

    // wait_us(10000);
    
    code = server.open(&eth);
    // pc.printf("opening server code = %d\n\r",code);

    if(code!=0) { pc.printf("Error from opening server = %d\n\r",code); }    
    code = server.bind(local);
    if(code!=0) { pc.printf("Error from binding socket = %d\n\r",code); }

    // eth_send.attach_us(&send_eth_data,10000); // 100Hz

    printf("Beginning to sample.\n\r");    

   while (true) {
        
        n = server.recvfrom(&client, recv_buf, 256); 
   

     
        
        
        if (n > 0) {
            // Null-terminate the received data
            recv_buf[n] = '\0';
         
            // Print received data to serial
            printf("Received command: %s\n", recv_buf);

     
            if (recv_buf[0]=='r') {
                
                dxl_bus.GetMultPositions(dxl_position, dxl_ID, 2); 
                printf("returned position 1: %i\n\r", dxl_position[0]);
                // printf("returned position 2: %i\n\r", dxl_position[1]);
                sprintf(send_buf, "1.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0\n");
                server.sendto(client, send_buf, sizeof(send_buf)); // send message, look for '\n' character when decoding the string
            } 

        //otherwise convert the string to an array of ints
        //TODO: probably should be sure this is an array of numbers so add that in an if statement
            else{
                
                
                dxl_commands = charToIntArray(recv_buf);
                
                printf("first elem %u\n\r",dxl_commands[0]);
                printf("second elem %u\n\r",dxl_commands[1]);
            
                wait_us(50000);
                
                sprintf(ok_response_buf, "done\n");
                server.sendto(client,ok_response_buf,sizeof(ok_response_buf));
            }
        
        }

    }
    // motor_cmd.attach_us(&motorCommand,2000);
    // t.reset();
    // t.start();
    // while (true) {

    //  if (motor_cmd_flag){
    //         motor_cmd_flag = 0;
    //     dxl_bus.SetMultGoalPositions(dxl_ID, idLength, rtPos);
    //     dxl_bus.GetMultPositions(dxl_position, dxl_ID, idLength);
    //     pc.printf("%d, %d\n\r",dxl_position[0],dxl_position[1]);
    //     }
    // }
    // t.stop();
    // dxl_time = t.read_ms();


}


