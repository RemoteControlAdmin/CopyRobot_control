/* Circular Robot Research Group
   Authors: yongyut@pit.ac.th
*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <chrono>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <errno.h>
#include "udp_connect.hpp"
//==========Declaration the kinmetic variable==========
// --------------Kinemic of 3-Wheels Omni Mobile Robot-----------------
// R = 0.0500;       // R     is Radius of Wheel            ; m
// L = 0.1150;       // L     is Radius of Robot            ; m
// Wr+max = +14.34 rad/s @12V, Wr-max = -14.34 rad/s @-12V
// Vr = +1.904 m/s (+1.6490, -0.9519)
// Vr = +1.904 m/s (+0.0000, +1.9040)
// Ww = 32.9754 rad/s (314.8920 rpm) @12V Maximum

double InvVgMatrix[3];//[IVg]
/*Inverse Gobal Velocity_Matrix =		  |IVgx|
                      		    	  	  |IVgy|
                      		    	  	  |IWg|*/
double InvTransMatrix[3][3];//[R]^-1
/*Inv_Tran_Matrix =						  |+sin(Angle+90)   -cos(Angle+90)  0|*(1/cos(Angle)*sin(Angle+90)-sin(Angle)*cos(Angle+90))
										  |+sin(Angle)      -cos(Angle)     0|
										  |0                0               1|*/
double InvVrMatrix[3];//[IVr]=[R]^-1*[IVg]
/*Inverse Robot Velocity_Matrix =		  |+sin(Angle+90)   -cos(Angle+90)  0|*(1/cos(Angle)*sin(Angle+90)-sin(Angle)*cos(Angle+90))*|IVgx|
										  |+sin(Angle)      -cos(Angle)     0|														*|IVgy|
										  |0                0               1|														*|IWg |*/
double InvMatrix[3][3];//[C]^-1
/*InvMatrix =    		 			(3/R)*|+1/6      +1/(2*sqrt(3)) L/3|
                        		    	  |-1/3      +0             L/3|
                         		    	  |+1/6      -1/(2*sqrt(3)) L/3|*/
double InvWwMatrix[3];//[IWw]=[C]^-1*[IVr]
/*Inverse Wheel Velocity_Matrix =   (3/R)*|+1/(2*sqrt(3))	-1/6    L/3|*|IVrx|
                        		    	  |+0				+1/3	L/3|*|IVry|
                         		    	  |-1/(2*sqrt(3))	-1/6	L/3|*|IWr|*/
double ForWwMatrix[3];//[FWw]
/*Forward Wheel Velocity_Matrix =		  |FWw1|
										  |FWw2|
										  |FWw3|*/
double ForMatrix[3][3];//[C]
/*ForMatrix =			  	  		(R/3)*|+sqrt(3)	 +0		-sqrt(3)|
										  |-1		 +2			-1	|
										  |+1/L      +1/L      +1/L |*/
double ForVrMatrix[3];//[FVr]=[C]*[FWw]
/*Forward Robot Velocity_Matrix =	(R/3)*|+1        -2     +1      |*|Fvrx|
										  |+sqrt(3)  +0     -sqrt(3)|*|Fvry|
										  |+1/L      +1/L      +1/L |*|FWr|*/
double ForTransMatrix[3][3];//[R]
/*For_Tran_Matrix =						  |cos(A)   cos(A+90)   0|
										  |sin(A)   sin(A+90)   0|
										  |0        0           1|*/
double ForVgMatrix[3];//[FVg]=[R]*[FVr]
/*Forward Global_Velocity_Matrix =		  |cos(A)   cos(A+90)   0|*|FVgx|
                						  |sin(A)   sin(A+90)   0|*|FVgy|
										  |0        0           1|*|FWg |*/

double VmMatrix[3]={0,0,0};
/*Voltage_Motor_Matrix =				  |Vm1|
										  |Vm2|
										  |Vm3|*/

double BRPositionReceiving();
double MRobot_Manual_PositionCal();
double MRobot_Linear_PositionCal();
double MRobot_Rotate_PositionCal();
double MRobot_Square_PositionCal();
double MRobot_Circle_PositionCal();
double MRobot_Eight_PositionCal();
double MRobot_Heart_PositionCal();
double CRobotPositionCal();
double ERobotPositionCal();
double OLPositionControllerCal();
double CLPositionControllerPIDCal();
double ConvertWWtoVmCal();
double VelocityLimitationCal();
double InvGlobalVelocityCal();
double InvTransMatrixCal();
double InvRobotVelocityCal();
double InvMatrixCal();
double InvWheelVelocityCal();
double LinkIWwtoFWwCal();
double ForWheelVelocityCal();
double ForMatrixCal();
double ForRobotVelocityCal();
double ForTransMatrixCal();
double ForGlobalVelocityCal();
double WwtoVoltGain = 12/32.9754;	//WwtoVoltGain Volt/(rad/s)
int i,j,nP;
//==========================================================

//==========Declaration the controller variable=========
//Ziegler-Nichols Kp =  8.625, Ki = 16.587, Kd =  1.121
//Fine tune Kp =  6.625, Ki = 0.015, Kd =  25.500 (Old Servo board)
//Fine tune Kp =  6.625, Ki = 0.015, Kd =  25.500 (New Servo board)
double Kp = 1.050;
double Ki = 0.015;
double Kd = 0.00;
double Pd[16] = {0,0,0};				//Variable position desired 
double Pg[32] = {0,0,0,0,0,0,0,0,0};				//Variable position global
double Pe[16] = {0,0,0};				//Variable position error
double Le[16] = {0,0,0};				//Last error
double Ww[16] = {0,0,0};				//Variable for the forward kinematic command testing
double Vd[16] = {0,0,0};				//Variable velocity desired
double Vg[16] = {0,0,0};				//Variable velocity global
double Ve[16] = {0,0,0};				//Variable velocity error
double Vc[16] = {0,0,0};				//Variable velocity control
double Integral[16]	= {0.0,0.0,0.0,0.0,0.0};			//Integral of error
double Derivative[16]= {0.0,0.0,0.0,0.0,0.0};			//Derivative of error
double Vlc,Wlc,Wcmin,Wcmax;
//==========================================================

//========Declaration the servo board communication=========
double EnableMotorDrive();
double DisableMotorDrive();
char DObufdat[2048];
char Pathbufdat[2048];
int EN1=26, EN2=65, EN3=46;
int pwm_1=7, pwm_2=1, pwm_3=4; //PWM Chip No. 7,1,4
int DCM1A=0; //P8_19 PWM_2A_(7:0) //Motor 1
int DCM1B=1; //P8_13 PWM_2B_(7:1) //Motor 1
int DCM2A=0; //P9_31 PWM_0A_(1:0) //Motor 2
int DCM2B=1; //P9_29 PWM_0B_(1:1) //Motor 2
int DCM3A=0; //P9_14 PWM_1A_(4:0) //Motor 3
int DCM3B=1; //P9_16 PWM_1B_(4:1) //Motor 3
int frequency_drive = 1000; //1kHz
int period;
//==========================================================

//==========Declaration the camera communication============
//On CR side (Position Receiver)
//CRA1=100.77.38.11 (40011)
//CRA2=100.77.38.12 (40012)
//CRB1=100.77.38.21 (40021)
//CRB2=100.77.38.22 (40022)
//CRC1=100.77.38.31 (40031)
//CRC2=100.77.38.32 (40032)
int ChkPosdat;
int Possock;
struct sockaddr_in addr1;
socklen_t addr1_size;
char PosBufdat[2048];
double *Posdat;
int BRPosition_Port = 40011;
//==========================================================

//==============Declaration the file management==============
FILE *fpt = fopen("CRA1.csv","w+");
int SamplingTime = 10000;
double Elapsed=0.0;
struct timespec Start_time, End_time;

//============Declaration the display monitoring============
double DisplayInformation();
double FWw1,FWw2,FWw3,FVrx,FVry,FWr,FVgx,FVgy,FWg,Vm1,Vm2,Vm3;
double IWw1,IWw2,IWw3,IVrx,IVry,IWr,IVgx,IVgy,IWg;
double Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg;
int Vm1I,Vm2I,Vm3I;
int ChkStop;
//==========================================================

//==========Sub function for DC Motor drive variable==========
//Set the pin mode of a GPIO pin
void pinMode(int pin, const char *direction) {
    snprintf(Pathbufdat, sizeof(Pathbufdat), "%s%d/direction", "/sys/class/gpio/gpio", pin);
    int fd_direction = open(Pathbufdat, O_WRONLY);
    if (fd_direction == -1) {
        perror("Error opening direction file");
        exit(EXIT_FAILURE);
    }

    if (write(fd_direction, direction, strlen(direction)) == -1) {
        perror("Error writing to direction");
        exit(EXIT_FAILURE);
    }
    close(fd_direction);
}
//Digital write a value to a GPIO pin
void digitalWrite(int pin, int value) {
    snprintf(Pathbufdat, sizeof(Pathbufdat), "%s%d/value", "/sys/class/gpio/gpio", pin);
    int fd_value = open(Pathbufdat, O_WRONLY);
    if (fd_value == -1) {
        perror("Error opening value file");
        exit(EXIT_FAILURE);
    }
    snprintf(DObufdat, sizeof(DObufdat), "%d", value);

    if (write(fd_value, DObufdat, strlen(DObufdat)) == -1) {
        perror("Error writing to value");
        exit(EXIT_FAILURE);
    }
    close(fd_value);
}
//Frequency write a period of a PWM pin
void frequencyWrite(int chip, int channel, int Period) {
    char path[50];
    sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/period", chip, chip, channel);
    int period_fd = open(path, O_WRONLY);
    if (period_fd == -1) {
        perror("Error opening period file");
        exit(EXIT_FAILURE);
    }
    dprintf(period_fd, "%d", Period);
    close(period_fd);
}
//Analog write a duty of a PWM pin
void analogWrite(int chip, int channel, int duty_cycle) {
    char path[50];
    sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/duty_cycle", chip, chip, channel);
    int duty_cycle_fd = open(path, O_WRONLY);
    if (duty_cycle_fd == -1) {
        perror("Error opening duty_cycle file");
        exit(EXIT_FAILURE);
    }
    dprintf(duty_cycle_fd, "%d", duty_cycle);
    close(duty_cycle_fd);
}
//Set the enable of a PWM pin
void enable_pwm(int chip, int channel) {
    char path[50];
    sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/enable", chip, chip, channel);    
    int enable_fd = open(path, O_WRONLY);
    if (enable_fd == -1) {
        perror("Error opening enable file");
        exit(EXIT_FAILURE);
    }
    dprintf(enable_fd, "1");
    close(enable_fd);
}
//Set the disable of a PWM pin
void disable_pwm(int chip, int channel) {
    char path[50];
    sprintf(path, "/sys/class/pwm/pwmchip%d/pwm-%d:%d/enable", chip, chip, channel); 
    int enable_fd = open(path, O_WRONLY);
    if (enable_fd == -1) {
        perror("Error opening enable file for disabling PWM");
        exit(EXIT_FAILURE);
    }
    dprintf(enable_fd, "0");
    close(enable_fd);
}
// Signal handler to stop the loop gracefully on Ctrl+C
void handle_termination(int signum) {
    printf("\nTermination signal by (Ctrl+C).\n\n");
    //You can perform cleanup or other actions before exiting
    //For example, freeing allocated memory, closing files, etc.

	//Disable a Robot
	DisableMotorDrive();
	close(Possock);
    exit(signum);
}

//Main loop
int main(int argc, char *argv[]){ 

    // IPアドレスとポート番号を指定して、UdpConnectインスタンスを作成
    udp_lib::UdpConnect udpConnection("192.168.11.29", 4102, 6);

    if (signal(SIGINT, handle_termination) == SIG_ERR) {
        perror("Error setting up signal handler");
        return 1;
    }
	
    //Call script file to setup PWM pin
    system("bash config_pin.sh");
	//printf("\nPlease wait! the robot is moving soon................... in 3 seconds\n\n");
	//sleep(3);

    //Set period for each channel
    period=1.0/(frequency_drive*1e-9);
    frequencyWrite(pwm_1, DCM1A, period);
    frequencyWrite(pwm_1, DCM1B, period);
    frequencyWrite(pwm_2, DCM2A, period);
    frequencyWrite(pwm_2, DCM2B, period);
    frequencyWrite(pwm_3, DCM3A, period);
    frequencyWrite(pwm_3, DCM3B, period);

    //Position create socket
    Possock = socket(AF_INET, SOCK_DGRAM, 0);
    if(Possock == -1){
    	perror("Error creating socket");
        exit(EXIT_FAILURE);
    }
	//Position initialize server address structure
    memset(PosBufdat, 0, sizeof(PosBufdat));
	addr1.sin_family = AF_INET;
    addr1.sin_port = htons(BRPosition_Port);		//Set port 40084 to receive the MRC's position from the camera on location C
    addr1.sin_addr.s_addr = INADDR_ANY;				//IP address (any address)

    //Position bind the socket to the server address
    bind(Possock, (struct sockaddr *)&addr1, sizeof(addr1));
	usleep(50000);									//Settle time for worker thread start

	printf("\nPlease wait! the robot is moving soon................... in 3 seconds\n\n");
	sleep(3);

    std::chrono::system_clock::time_point  Start, End;	//Time initializes
	Start = std::chrono::system_clock::now();			//Start time

    while (1){
    //Getting time
    clock_gettime(CLOCK_REALTIME, &Start_time);
	//Both Robot's position receiving
    BRPositionReceiving();
    
		//Position check the communication
        if(nP=64){
            Posdat=(double*)PosBufdat;
            ChkPosdat=(int)Posdat[0];

            Elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(End-Start).count(); //Elapse time in second unit

			//Input/Output part
			//---------------------------------------------------------------------------------------------
			MRobot_Manual_PositionCal();		//Get MasterRobot's Position for manual path trajectory
			//MRobot_Linear_PositionCal();		//Get MasterRobot's Position for linear path trajectory
			//MRobot_Rotate_PositionCal();		//Get MasterRobot's Position for rotate path trajectory
			//MRobot_Square_PositionCal();		//Get MasterRobot's Position for square path trajectory
			//MRobot_Circle_PositionCal();		//Get MasterRobot's Position for circle path trajectory
			//MRobot_Eight_PositionCal();		//Get MasterRobot's Position for Eight path trajectory
			//MRobot_Heart_PositionCal();		//Get MasterRobot's Position for Heart path trajectory
			CRobotPositionCal();				//Get CopyRobot's Position
			ERobotPositionCal();				//Get ErrorRobot's Position

			//Controller part
			//---------------------------------------------------------------------------------------------			
			//OLPositionControllerCal();			//Send Global Velocity command [Vc] by [IVg] for Open loop controller
			CLPositionControllerPIDCal();			//Send Global Velocity command [Vc] by [IVg] for PID Closed loop controller
			
			//Velocity Limitation part
			//---------------------------------------------------------------------------------------------		
			VelocityLimitationCal();

			//Kinematic part
			//---------------------------------------------------------------------------------------------		
			//Inverse Kinematic side
			//For tesing Inverse kinematic, velocity of robot [Vg]   	
			InvGlobalVelocityCal();		//Get Inverse Global Velocity [IVg]			
			InvTransMatrixCal();		//Build Inverse Transformation Matrix by Ag variable
			InvRobotVelocityCal();		//Get Inverse Robot Velocity [IVr]
			InvMatrixCal();				//Build Inverse Matrix constant
			InvWheelVelocityCal();		//Get Inverse Wheel Velocity [IWw]
			//Link Forward Kinematic with Forward Kinematic
			LinkIWwtoFWwCal();			//Link Inverse Wheel Velocity [IWw] with Forward Wheel Velocity [FWw] by [Ww]
			ConvertWWtoVmCal();			//Convert Velocity of Wheel to Voltage of DC Motor
			EnableMotorDrive();			//Enable the DC motors
			//Forward Kinematic side
			//For tesing Forward kinematic, velocity of wheel [Ww]   
			ForWheelVelocityCal();		//Get Forward Wheel Velocity [IWw]
			ForMatrixCal();				//Build Forward Matrix constant
    		ForRobotVelocityCal();		//Get Forward Robot Velocity [FVr]
			ForTransMatrixCal();		//Build Forward Transformation Matrix by Ag variable
    		ForGlobalVelocityCal();		//Get Forward Global Velocity [FVg]
			
			Le[0] = Pe[0];				//Build last error x for derivative controller
			Le[1] = Pe[1];				//Build last error y for derivative controller
			Le[2] = Pe[2];				//Build last error a for derivative controller	
			//---------------------------------------------------------------------------------------------

			//Data Management part
			//---------------------------------------------------------------------------------------------				
			//Real time display the information
			DisplayInformation();
			//Generate the Log file *.csv
			//fprintf(fpt,"%.0f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.2f, %+.2f, %+.2f, %+d, %+d, %+d, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f\n",Elapsed,Pd[0],Pd[1],Pd[2],FVgx,FVgy,FWg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,Vm1I,Vm2I,Vm3I,Pg[0],Pg[1],Pg[2],Vgx,Vgy,Wg,Pe[0],Pe[1],Pe[2]);
			//MATLAB Plot
			//[PMR PCR PE VMR VC VCR Ww Vm VmI]
            std::vector<double> dataToSend = {Posdat[1], Posdat[2],Posdat[3],Posdat[4],Posdat[5],Posdat[6]};
            udpConnection.udp_send(dataToSend, (int)Posdat[7]);
			fprintf(fpt,"%.2f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+d, %+d, %+d, %+d\n",Elapsed/1000,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,Vm1I,Vm2I,Vm3I,ChkStop);				
 
			End = std::chrono::system_clock::now();			//End time 1 cycle
			usleep(SamplingTime);				//Microsescond
            }
    }
    usleep(50000);
    close(Possock);
    return 0;
}
//=================================================================

//==========Sub function for the kinmetic variable==========
//Force receiving
double BRPositionReceiving() {
    nP=recvfrom(Possock, PosBufdat, sizeof(PosBufdat), 0, (struct sockaddr *)&addr1, &addr1_size);
	return 0;
}
//[Pd] Manual movement
double MRobot_Manual_PositionCal(){
	//If flag = 1 collect the master robot's data
	if(ChkPosdat==1){
    	Pd[0]=Posdat[1]/1000.0;			//Pdx converted to m
    	Pd[1]=Posdat[2]/1000.0;			//Pdy converted to m
    	Pd[2]=fmod(Posdat[3]*M_PI/180.0+M_PI,2*M_PI)-M_PI;	//Wrap to the range from [0,360] to [-pi, +pi]
		Pd[4]=Posdat[4]/1000.0;			//Vdx converted to m/s
    	Pd[5]=Posdat[5]/1000.0;			//Vdy converted to m/s
    	Pd[6]=Posdat[6]*M_PI/180.0;		//Wd  converted to rad/s
	}return 0;
}
//[Pd] Linear movement
double MRobot_Linear_PositionCal(){
	if     ((Elapsed>=0000)&&(Elapsed<= 5000))	{Pd[0]=0.4;Pd[1]=0.4;Pd[2]=M_PI/2;}
	else if((Elapsed> 5000)&&(Elapsed<= 8000))	{Pd[0]=0.6;Pd[1]=0.6;Pd[2]=M_PI/2;}
	else										{Pd[0]=0.4;Pd[1]=0.4;Pd[2]=M_PI/2;}
	return 0;
}
//[Pd] Rotate movement
double MRobot_Rotate_PositionCal(){
	if     ((Elapsed>=0000)&&(Elapsed<= 5000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(1*M_PI)/4;}
	else if((Elapsed> 5000)&&(Elapsed<= 8000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(3*M_PI)/4;}
	else if((Elapsed> 8000)&&(Elapsed<=11000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(5*M_PI)/4;}
	else if((Elapsed>11000)&&(Elapsed<=14000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(7*M_PI)/4;}
	else if((Elapsed>14000)&&(Elapsed<=17000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(5*M_PI)/4;}
	else if((Elapsed>17000)&&(Elapsed<=20000))	{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(3*M_PI)/4;}
	else										{Pd[0]=0.5;Pd[1]=0.5;Pd[2]=(1*M_PI)/4;}
	return 0;
}
//[Pd] Square movement
double MRobot_Square_PositionCal(){
	if     ((Elapsed>=0000)&&(Elapsed<= 4000))	{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
	else if((Elapsed> 4000)&&(Elapsed<= 8000))	{Pd[0]=0.60;Pd[1]=0.40;Pd[2]=M_PI/2;}
	else if((Elapsed> 8000)&&(Elapsed<=12000))	{Pd[0]=0.60;Pd[1]=0.60;Pd[2]=M_PI/2;}
	else if((Elapsed>12000)&&(Elapsed<=16000))	{Pd[0]=0.40;Pd[1]=0.60;Pd[2]=M_PI/2;}
	else if((Elapsed>16000)&&(Elapsed<=20000))	{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
	else										{Pd[0]=0.40;Pd[1]=0.40;Pd[2]=M_PI/2;}
	return 0;
}
//[Pd] Circle movement
double MRobot_Circle_PositionCal(){
	double F=0.05;
	double Wc=2.0*M_PI*F*(Elapsed/1000);
	if((Elapsed>=0)&&(Elapsed<=5000))			{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
	else if ((Elapsed>5000)&&(Elapsed<15000))	{Pd[0]=(0.10*cos(1*Wc))+0.50;Pd[1]=(0.10*sin(1*Wc))+0.50;Pd[2]=M_PI/2.0;}
	else										{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
	return 0;
}
//[Pd] Eight movement
double MRobot_Eight_PositionCal(){
	double F=0.05;
	double Wc=2.0*M_PI*F*(Elapsed/1000);
	if((Elapsed>=0)&&(Elapsed<=5000))			{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
	else if ((Elapsed>5000)&&(Elapsed<25000))	{Pd[0]=(0.10*cos(1*Wc))+0.50;Pd[1]=(0.10*sin(2.0*Wc))+0.50;Pd[2]=M_PI/2.0;}
	else										{Pd[0]=0.5,Pd[1]=0.5,Pd[2]=M_PI/2.0;}
	return 0;
}
//[Pd] Heart movement
double MRobot_Heart_PositionCal(){
	double F=0.05;
	double Wc=2.0*M_PI*F*(Elapsed/1000);
	if ((Elapsed>0)&&(Elapsed<20000))			{Pd[0]=0.02*16.0*pow(sin(Wc),3)+0.5,Pd[1]= 0.02*(13.0*cos(Wc)-5.0*cos(2.0*Wc)-2.0*cos(3.0*Wc)-1.0*cos(4.0*Wc))+0.55,Pd[2]=M_PI/2.0;}
	else										{Pd[0]=0.50,Pd[1]=0.65,Pd[2]=M_PI/2.0;}
	return 0;
}
//[Pg]
double CRobotPositionCal(){
	//If flag = 2 collect the copy robot's data
    if(ChkPosdat==2){
    	Pg[0]=Posdat[1]/1000.0;		//Pgx
    	Pg[1]=Posdat[2]/1000.0;		//Pgy
    	Pg[2]=fmod(Posdat[3]*M_PI/180.0+M_PI,2*M_PI)-M_PI;	//Wrap to the range from [0,360] to [-pi, +pi]
		Pg[4]=Posdat[4]/1000.0;		//Vgx
    	Pg[5]=Posdat[5]/1000.0;		//Vgy
    	Pg[6]=Posdat[6]*M_PI/180.0;	//Wg
	}return 0;
}
//[Pe]=[Pd]-[Pg]
double ERobotPositionCal(){
    for(i=0;i<3;i++){
        Pe[i]=Pd[i]-Pg[i];
    }return 0;
}
//Inverse Gobal Velocity_Matrix [IVg]
double InvGlobalVelocityCal(){
	//printf("\nInverse Gobal Velocity_Matrix [IVg]);
	for(i=0;i<3;i++){
		InvVgMatrix[i]=	Vc[i];
    //    printf("%+.4f\n",InvVgMatrix[i]);
    }return 0;
}
//Inverse Transformation Matrix [R]^-1 
double InvTransMatrixCal(){
    InvTransMatrix[0][0]=(+sin(Pg[2]+(M_PI/2)))/(cos(Pg[2])*sin(Pg[2]+(M_PI/2))-sin(Pg[2])*cos(Pg[2]+(M_PI/2)));
    InvTransMatrix[0][1]=(-cos(Pg[2]+(M_PI/2)))/(cos(Pg[2])*sin(Pg[2]+(M_PI/2))-sin(Pg[2])*cos(Pg[2]+(M_PI/2)));
    InvTransMatrix[0][2]=0;
    InvTransMatrix[1][0]=(-sin(Pg[2]))/(cos(Pg[2])*sin(Pg[2]+(M_PI/2))-sin(Pg[2])*cos(Pg[2]+(M_PI/2)));
    InvTransMatrix[1][1]=(+cos(Pg[2]))/(cos(Pg[2])*sin(Pg[2]+(M_PI/2))-sin(Pg[2])*cos(Pg[2]+(M_PI/2)));
    InvTransMatrix[1][2]=0;
    InvTransMatrix[2][0]=0;
    InvTransMatrix[2][1]=0;
    InvTransMatrix[2][2]=(cos(Pg[2])*sin(Pg[2]+(M_PI/2))-sin(Pg[2])*cos(Pg[2]+(M_PI/2)))/((cos(Pg[2])*sin(Pg[2]+(M_PI/2)))-(sin(Pg[2])*cos(Pg[2]+(M_PI/2))));
	/*    printf("\nInverse Transformation Matrix [R]^-1 (Angle): %+0.2f rad",Pg[2]);
	for(i=0;i<3;i++){
        printf("\n");
        for(j=0;j<3;j++){
        	printf("%+.4f\t",InvTransMatrix[i][j]);
        }
    }
	printf("\n");*/
	return 0;
}
//Inverse Robot Velocity_Matrix [IVr]=[R]^-1*[IVg]
double InvRobotVelocityCal(){
	double MatrixSum,MatrixRes;
	//printf("Inverse Global Velocity Matrix [IVr]");
    for(i=0;i<3;i++){    
		//	printf("\n");  
        for(j=0;j<3;j++){
            MatrixSum=(InvTransMatrix[i][j]*InvVgMatrix[j]);
			MatrixRes = MatrixRes+MatrixSum;
        }
		InvVrMatrix[i]=MatrixRes;
		MatrixRes=0;
    //    printf("%+.4f\t", InvVrMatrix[i]);
    }return 0;
}
//Inverse Matrix [C]^-1
double InvMatrixCal(){
	InvMatrix[0][0]=(3/0.050)*(1.0/(2.0*sqrt(3)));
	InvMatrix[0][1]=(3/0.050)*(-1.0/6.0);
	InvMatrix[0][2]=(3/0.050)*(0.115/3.0);
	InvMatrix[1][0]=(3/0.050)*0.0;
	InvMatrix[1][1]=(3/0.050)*(1.0/3.0);
	InvMatrix[1][2]=(3/0.050)*(0.115/3.0);
	InvMatrix[2][0]=(3/0.050)*(-1.0/(2.0*sqrt(3)));
	InvMatrix[2][1]=(3/0.050)*(-1.0/6.0);
	InvMatrix[2][2]=(3/0.050)*(0.115/3.0);
	/*    printf("\nInverse Matrix [C]^-1");
	for(i=0;i<3;i++){
        printf("\n");
        for(j=0;j<3;j++){
        	printf("%+.4f\t",InvMatrix[i][j]);
        }
    }
	printf("\n");*/
	return 0;
}
//Inverse Wheel Velocity_Matrix [IWw]=[C]^-1*[IVr]
double InvWheelVelocityCal(){
	double MatrixSum,MatrixRes;
	//printf("Inverse Wheel Velocity Matrix [IWw]");
    for(i=0;i<3;i++){ 
	//	printf("\n");
        for(j=0;j<3;j++){
            MatrixSum=(InvMatrix[i][j]*InvVrMatrix[j]);
			MatrixRes = MatrixRes+MatrixSum;
        }
		InvWwMatrix[i]=MatrixRes;
		MatrixRes=0;
    //    printf("%+.4f\t", InvWwMatrix[i]);
    }return 0;
}
//Forwrard Wheel Velocity_Matrix = Inverse Wheel Velocity_Matrix [FWw]=[IWw]
double LinkIWwtoFWwCal(){
	for(i=0;i<3;i++){ 
		Ww[i]=InvWwMatrix[i];
	}return 0;
}
//Forwrard Wheel Velocity_Matrix [FWw]
double ForWheelVelocityCal(){
	//printf("\nForwrard Wheel Velocity_Matrix [FWw]");
	for(i=0;i<3;i++){
		ForWwMatrix[i]=	Ww[i];
    //	printf("%+.4f\n",ForWwMatrix[i]);
    }return 0;
}
//Forward Matrix [C]
double ForMatrixCal(){
	ForMatrix[0][0]=(0.050/3)*sqrt(3);
	ForMatrix[0][1]=(0.050/3)*0.0;
	ForMatrix[0][2]=(0.050/3)*-sqrt(3);
	ForMatrix[1][0]=(0.050/3)*-1.0;
	ForMatrix[1][1]=(0.050/3)*2.0;
	ForMatrix[1][2]=(0.050/3)*-1.0;
	ForMatrix[2][0]=(0.050/3)*(1.0/0.115);
	ForMatrix[2][1]=(0.050/3)*(1.0/0.115);
	ForMatrix[2][2]=(0.050/3)*(1.0/0.115);
	/*    printf("\nForward Matrix [C]");
	for(i=0;i<3;i++){
        printf("\n");
        for(j=0;j<3;j++){
        	printf("%+.4f\t",ForMatrix[i][j]);
        }
    }
	printf("\n");*/
	return 0;
}
//Forward Robot Velocity_Matrix [FVr]=[C]*[FWw]
double ForRobotVelocityCal(){
	double MatrixSum,MatrixRes;
    //printf("Forward Robot Velocity Matrix");
	for(i=0;i<3;i++){
	//	printf("\n");
        for(j=0;j<3;j++){
        	MatrixSum = ForMatrix[i][j]*ForWwMatrix[j];
			MatrixRes = MatrixRes+MatrixSum;
		}
		ForVrMatrix[i]=MatrixRes;
		MatrixRes=0;
    //    printf("%+.4f\t",ForVrMatrix[i]);
    }return 0;
}
//Forward Transformation Matrix [R]
double ForTransMatrixCal(){
    ForTransMatrix[0][0]=+cos(Pg[2]);
    ForTransMatrix[0][1]=+cos(Pg[2]+(M_PI/2));
    ForTransMatrix[0][2]=0;
    ForTransMatrix[1][0]=+sin(Pg[2]);
    ForTransMatrix[1][1]=+sin(Pg[2]+(M_PI/2));
    ForTransMatrix[1][2]=0;
    ForTransMatrix[2][0]=0;
    ForTransMatrix[2][1]=0;
    ForTransMatrix[2][2]=1;
	/*	printf("\nForward Transformation Matrix [R] (Angle): %+0.2f rad",Pg[2]);
	for(i=0;i<3;i++){
        printf("\n");
        for(j=0;j<3;j++)
            printf("%+.4f\t",ForTransMatrix[i][j]);
        }
    }
	printf("\n");*/
	return 0;
}
//Forward Global Velocity_Matrix [FVg]=[R]*[FVr]
double ForGlobalVelocityCal(){
	double MatrixSum,MatrixRes;
	//printf("Forward Global Velocity Matrix");
    for(i=0;i<3;i++){ 
	//	printf("\n");
        for(j=0;j<3;j++){
            MatrixSum =(ForTransMatrix[i][j]*ForVrMatrix[j]);
			MatrixRes = MatrixRes+MatrixSum;
        }
		ForVgMatrix[i]=MatrixRes;
		MatrixRes=0;
	//	printf("%+.4f\t",ForVgMatrix[i]);
    }return 0;
}
//=================================================================

//==========Sub function for closed-loop control variable==========
//[Vd] or [Vc] Open Loop Desired/Control Velocity Command
double OLPositionControllerCal(){
	//printf("Global Velocity Matrix Command");
	for(i=0;i<3;i++){
		Vc[i]=Vd[i];
	//	printf("%+.4f\n",Vcg[i]);
	}return 0;
}
//[Vc] PID Closed Loop Control Velocity Command
double CLPositionControllerPIDCal(){
	//printf("Global Velocity Matrix Command");
	for(i=0;i<3;i++){
	//	printf("\n");
	//	[Vc]=Kp*[Pe] + Ki*integral([Pe]) + Kd*derivative([Pe]) 
		Integral[i]		+= Pe[i];								//Calculate the integral term
		Derivative[i]	= Pe[i] - Le[i];						//Calculate the derivative term
    	Vc[i] = Kp*Pe[i] + Ki*Integral[i] + Kd*Derivative[i];	//Calculate the output term
	//	printf("%+.4f",Vc[i]);
	}return 0;
}
//[Vlc] Velocity Limitation by Boundary Cone
double VelocityLimitationCal(){
	//[Wcmax]=[-m]*[Vlc]+[Wgmax],[Wcmin]=[+m]*[Vlc]-[Wgmax]
	Vlc=sqrt((Vc[0]*Vc[0])+(Vc[1]*Vc[1]));
	Wlc=Vc[2];
    Wcmax=-8.6962*Vlc+14.3400;
    Wcmin=+8.6962*Vlc-14.3400;

	if(((Vlc>=0)&&(Vlc<=1.6490))&&((Wlc>=Wcmin)&&(Wlc<=Wcmax))){
		Vc[0] = Vc[0];
		Vc[1] = Vc[1];
		Vc[2] = Vc[2];
		ChkStop=0;
	}
	else{
		Vc[0] = 0;
		Vc[1] = 0;
		Vc[2] = 0;
		ChkStop=1;
	}return 0;
}
//[Vm] 
double ConvertWWtoVmCal(){
	//printf("Motor Voltage Matrix");
	for(i=0;i<3;i++){
	//	printf("\n");
	//	[Vm]=[WwtoVoltGain]*[Ww]
    	VmMatrix[i] = ForWwMatrix[i]*WwtoVoltGain;
	//	printf("%+d",VmMatrix[i]);
	}return 0;
}
//Enable DC motor drive
double EnableMotorDrive(){
     // Set voltage power supply for a DC Motor   
    double Vss = 24;
    // Set GPIO directions
    pinMode(EN1, "out");
    pinMode(EN2, "out");
    pinMode(EN3, "out");
    // Write digital value
    digitalWrite(EN1, 1);
    digitalWrite(EN2, 1);
    digitalWrite(EN3, 1);

    int V1A_peri = abs(((+(0.5*VmMatrix[0])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
    int V1B_peri = abs(((-(0.5*VmMatrix[0])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
    int V2A_peri = abs(((+(0.5*VmMatrix[1])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
    int V2B_peri = abs(((-(0.5*VmMatrix[1])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
    int V3A_peri = abs(((+(0.5*VmMatrix[2])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)
    int V3B_peri = abs(((-(0.5*VmMatrix[2])+6.0)/Vss)*period); //Period unit that take a absolute, because the PWM is a positive side only)

    // Enable PWM channels
    enable_pwm(pwm_1, DCM1A);
    enable_pwm(pwm_1, DCM1B);
    enable_pwm(pwm_2, DCM2A);
    enable_pwm(pwm_2, DCM2B);
    enable_pwm(pwm_3, DCM3A);
    enable_pwm(pwm_3, DCM3B);
    // Set duty cycle for each channel
    analogWrite(pwm_1, DCM1A, V1A_peri);
    analogWrite(pwm_1, DCM1B, V1B_peri);
    analogWrite(pwm_2, DCM2A, V2A_peri);
    analogWrite(pwm_2, DCM2B, V2B_peri);
    analogWrite(pwm_3, DCM3A, V3A_peri);
    analogWrite(pwm_3, DCM3B, V3B_peri);  
    return 0;  
}
//Disable DC motor drive
double DisableMotorDrive(){
    disable_pwm(pwm_1, DCM1A);
    disable_pwm(pwm_1, DCM1B);
    disable_pwm(pwm_2, DCM2A);
    disable_pwm(pwm_2, DCM2B);
    disable_pwm(pwm_3, DCM3A);
    disable_pwm(pwm_3, DCM3B);
    digitalWrite(EN1, 0);
    digitalWrite(EN2, 0);
    digitalWrite(EN3, 0);
    return 0;  
}
//=================================================================

//==============Sub function for display information===============
double DisplayInformation(){
	//===Reverse side===
    IVgx=InvVgMatrix[0];		    	// Translational velocity of robot in X direction based on global frame ;m/s
    IVgy=InvVgMatrix[1];		    	// Translational velocity of robot in Y direction based on global frame ;m/s  
    IWg =InvVgMatrix[2];		    	// Rotational velocity of robot based on global frame                   ;rad/s
	IVrx=InvVrMatrix[0];		    	// Translational velocity of robot in X direction based on robot frame  ;m/s
    IVry=InvVrMatrix[1];		    	// Translational velocity of robot in Y direction based on robot frame  ;m/s 
    IWr =InvVrMatrix[2];		    	// Rotational velocity of robot based on robot frame                    ;rad/s
	IWw1=InvWwMatrix[0];				// Rotational velocity of wheel 1										;rad/s
	IWw2=InvWwMatrix[1];				// Rotational velocity of wheel 2										;rad/s
	IWw3=InvWwMatrix[2];				// Rotational velocity of wheel 3										;rad/s
	//===Forward side===
	FWw1=ForWwMatrix[0];				// Rotational velocity of wheel 1										;rad/s
	FWw2=ForWwMatrix[1];				// Rotational velocity of wheel 2										;rad/s
	FWw3=ForWwMatrix[2];				// Rotational velocity of wheel 3										;rad/s
	FVrx=ForVrMatrix[0];		    	// Translational velocity of robot in X direction based on robot frame  ;m/s
    FVry=ForVrMatrix[1];		    	// Translational velocity of robot in Y direction based on robot frame  ;m/s 
    FWr =ForVrMatrix[2];		    	// Rotational velocity of robot based on robot frame                    ;rad/s
    FVgx=ForVgMatrix[0];		    	// Translational velocity of robot in X direction based on global frame ;m/s
    FVgy=ForVgMatrix[1];		    	// Translational velocity of robot in Y direction based on global frame ;m/s  
    FWg =ForVgMatrix[2];		    	// Rotational velocity of robot based on global frame                   ;rad/s
	//===Convert Velocity of Wheel to Voltage of DC Motor===	
	Vm1=ForWwMatrix[0]*WwtoVoltGain;	// Voltage of motor 1	float number									;volt
	Vm2=ForWwMatrix[1]*WwtoVoltGain;	// Voltage of motor 2	float number									;volt
	Vm3=ForWwMatrix[2]*WwtoVoltGain;	// Voltage of motor 3	float number									;volt
	Vm1I=VmMatrix[0];					// Voltage of motor 1	integer number									;volt
	Vm2I=VmMatrix[1];					// Voltage of motor 2	integer number									;volt
	Vm3I=VmMatrix[2];					// Voltage of motor 3	integer number									;volt

	Pdx=Pd[0]*1000; Pdy=Pd[1]*1000; Ad =Pd[2];
	Pgx=Pg[0]*1000; Pgy=Pg[1]*1000; Ag =Pg[2];
    Pex=Pe[0]*1000; Pey=Pe[1]*1000; Ae =Pe[2];

	Vdx=Pd[4]; Vdy=Pd[5]; Wd =Pd[6];
	Vcx=Vc[0]; Vcy=Vc[1]; Wc =Vc[2];
	Vgx=Pg[4]; Vgy=Pg[5]; Wg =Pg[6];

	//Reverse kinematic parameters display
	//printf("Inverse Parameters | IVd: %+.4f %+.4f %+.4f | IVr: %+.4f %+.4f %+.4f | IWw: %+.4f %+.4f %+.4f | IVm: %+.1f %+.1f %+.1f | Ag: %+.4f\n",Vd[0],Vd[1],Vd[2],IVrx,IVry,IWr,IWw1,IWw2,IWw3,Vm1,Vm2,Vm3,Ag);
	//Forward kinematic parameters display
	//printf("Forward Parameters | FWw: %+.4f %+.4f %+.4f | FVm: %+.1f %+.1f %+.1f | FVr: %+.4f %+.4f %+.4f | FVg: %+.4f %+.4f %+.4f | Ag: %+.4f\n\n",FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,FVrx,FVry,FWr,FVgx,FVgy,FWg,Ag);
	//All kinematic parameters display
	//printf("| Vd: %+.4f %+.4f %+.4f | Ww: %+.4f %+.4f %+.4f | Vm: %+.4f %+.4f %+.4f | Vr: %+.4f %+.4f %+.4f | Vg: %+.4f %+.4f %+.4f | Ag: %+.4f\n",Vd[0],Vd[1],Vd[2],FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,FVrx,FVry,FWr,FVgx,FVgy,FWg,Ag);
	//Controller display
	//Position display
	//printf("T: %.0f | Pd: %+.2f %+.2f %+.2f | Pg: %+.2f %+.2f %+.2f | Pe: %+.2f %+.2f %+.2f | \n",Elapsed,Pd[0],Pd[1],Pd[2],Pg[0],Pg[1],Pg[2],Pe[0],Pe[1],Pe[2]);				
	//Position and velocities display
	//printf("T: %.2f | Pd: %+.2f %+.2f %+.2f | Pg: %+.2f %+.2f %+.2f | Pe: %+.2f %+.2f %+.2f | Vd: %+.2f %+.2f %+.2f | Vc: %+.2f %+.2f %+.2f | Vg: %+.2f %+.2f %+.2f | Ww: %+.2f %+.2f %+.2f | Vm: %+.2f %+.2f %+.2f | ChkStop: %+d |\n",Elapsed/1000,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,ChkStop);																					
	printf("T: %.2f | Pd: %+.0f %+.0f %+.2f | Pg: %+.0f %+.0f %+.2f | Pe: %+.0f %+.0f %+.2f | Vd: %+.2f %+.2f %+.2f | Vc: %+.2f %+.2f %+.2f | Vg: %+.2f %+.2f %+.2f | Ww: %+.2f %+.2f %+.2f | Vm: %+.2f %+.2f %+.2f | ChkStop: %+d |\n",Elapsed/1000,Pdx,Pdy,Ad*(180.0/M_PI),Pgx,Pgy,Ag*(180.0/M_PI),Pex,Pey,Ae*(180.0/M_PI),Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,ChkStop);																					

	return 0;
}
//=================================================================