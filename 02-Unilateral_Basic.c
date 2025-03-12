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

int    VmMatrix[3];
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
int i,j,nP,nF;
//==========================================================

//==========Declaration the controller variable=========
//Ziegler-Nichols Kp =  8.625, Ki = 16.587, Kd =  1.121
//Fine tune Kp =  6.500,  Ki = 0.015, Kd =  2.500 (Tracking Mode)
//Fine tune Kp =  25.500, Ki = 0.080, Kd =  18.500 (Touching Mode)
double Kp = 17.500;   
double Ki = 0.030;
double Kd = 13.500;
double Pd[16] = {0,0,0};				//Variable position desired
double Pg[16] = {0,0,0};				//Variable position global
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
//CRA1=192.168.11.101 (40081)
//CRA2=192.168.11.102 (40082)
//CRB1=192.168.11.106 (40083)
//CRB2=192.168.11.108 (40084)
//CRC1=192.168.11.112 (40085)
//CRC2=192.168.11.113 (40086)
int ChkPosdat;
int Possock;
struct sockaddr_in addr1;
socklen_t addr1_size;
char PosBufdat[2048];
double *Posdat;
//==========================================================

//==============Declaration the file management==============
FILE *fpt = fopen("CRA1.csv","w+");
int SamplingTime = 10000;
double Elapsed=0.000;
struct timespec Start_time, End_time;

//========Declaration the force sensor communication=========
double FEActCal();
double FEActSwapCal();
int Forsock;
struct sockaddr_in addr2,addr3;
socklen_t addr3_size;
double FEa[2048];
double FaS1[2048];
//===========================================================

//===========Declaration the force filter variable===========
double a=0.9391, b=0.0609; // Fc 10Hz, Ts 0.01 Sec
//===========================================================

//=====Declaration the actual force controller variable======
double Kspring = 16.000/0.020; // N/m (800 N/m)
int* Analog_Val;
int ADC_AI0,ADC_AI1,ADC_AI2,ADC_AI3;
double Volt0,Volt1,Volt2,Volt3;
double FAx,FAy,FBx,FBy;
double fFAx,fFAy,fFBx,fFBy;
double FAx_R,FAy_R,FA_R,FAa_R,FAx_G,FAy_G,FA_G,FAa_G;
double FBx_R,FBy_R,FB_R,FBa_R,FBx_G,FBy_G,FB_G,FBa_G;
double F_R,Fa_R,F_G,Fa_G;
double FEactM,FEactMa,FEactx,FEacty,FEactNull;
double AngleA_off = -(M_PI/6),AngleB_off = (M_PI*5/6);
double FEactMSat = 12.0*sqrt(2);                            //12N is Maximum of measureable range in X-axis, Then *sqrt(2) because is to convert in yhr polar form
double FEactMThreshold = 1.0;                               //Force Env. Actual Threshold
double FId[16] = {0.0,0.0,0.0,0.0,0.0};						//Variable ideal force
//==========================================================

//=====Declaration the virtual force controller variable====
double FIdCal();
double FEVirCal();
double DIde,A_DIde,FIdeal,A_FIdeal;
double DVir,A_DVir,FEVir,A_FEVir;
int MRTouchChk,CRTouchChk;
double DVirMin=0.000, DVirMax=0.025;
double FEv[16] = {0.0,0.0,0.0,0.0,0.0};						//Variable ideal force
//==========================================================

//========Declaration the position compensation variable=======
double ForceCompensateCal();
double Pdf[16] = {0,0,0};
double Pc[16]  = {0,0,0};
double Pcd = +0.000;                                        //Displacement compensataion desired in m. unit (- sign is decrese the gap (Dvir), + sign is increase the gap (Dvir))
int CompenChk;
//==========================================================

//========Declaration the LabVIEW communication=============
double LabVIEWTransmit();
int LabVIEWsock;
struct sockaddr_in addr4;
socklen_t addr4_size;
char Dataset[1024];
//==========================================================

//============Declaration the display monitoring============
double DisplayInformation();
double FWw1,FWw2,FWw3,FVrx,FVry,FWr,FVgx,FVgy,FWg,Vm1,Vm2,Vm3;
double IWw1,IWw2,IWw3,IVrx,IVry,IWr,IVgx,IVgy,IWg;
double Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg;
int Vm1I,Vm2I,Vm3I;
double Pax,Pay;
double FIdx,FIdy,FIdM,FIdMa,FIdMNull,FEvirx,FEviry,FEvirM,FEvirMa,FEvirNull;
double Pdfx,Pdfy,Adf,Pcx,Pcy,Ac;
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

    // Set up a signal handler to stop the loop gracefully on Ctrl+C
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

    //Force create socket
    Forsock = socket(AF_INET, SOCK_DGRAM, 0);
    if(Forsock<0){
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    //LabVIEW create socket
    LabVIEWsock = socket(AF_INET, SOCK_DGRAM, 0);
    if(LabVIEWsock<0){
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

	//Position initialize server address structure
    memset(PosBufdat, 0, sizeof(PosBufdat));
	addr1.sin_family = AF_INET;
    addr1.sin_port = htons(40011);					//Set port 400xx to receive the MRC's position from the camera on location C
    addr1.sin_addr.s_addr = INADDR_ANY;				//IP address (any address)
    //Position bind the socket to the server address
    bind(Possock, (struct sockaddr *)&addr1, sizeof(addr1));
	usleep(50000);									//Settle time for worker thread start
/*
	//Force initialize server address structure for transmit to destination 
    memset(&addr2, 0, sizeof(addr2));
    addr2.sin_family = AF_INET;     
    addr2.sin_port = htons(50021);                      //Destination Port for force
    inet_pton(AF_INET,"100.77.38.13", &addr2.sin_addr); //Destination IP Address for force
 	//Force initialize server address structure for receive from destination
    memset(&addr3, 0, sizeof(addr3));
    addr3.sin_family = AF_INET;                         
    addr3.sin_port = htons(50011);
    addr3.sin_addr.s_addr = INADDR_ANY;
    //Force bind the socket to the server address
    bind(Forsock, (struct sockaddr*)&addr3, sizeof(addr3));
	usleep(50000);

	//LabVIEW initialize server address structure for transmit to destination 
    memset(&addr4, 0, sizeof(addr4));
    addr4.sin_family = AF_INET;
    addr4.sin_port = htons(60011);
    inet_pton(AF_INET,"100.77.38.102", &addr4.sin_addr);
	usleep(50000);
*/
	printf("\nPlease wait! the robot is moving soon................... in 5 seconds\n\n");
	sleep(5);

	std::chrono::system_clock::time_point  Start, End;	//Time initializes
	Start = std::chrono::system_clock::now();			//Start time	

	while(1)
	{
		clock_gettime(CLOCK_REALTIME, &Start_time);		//Getting time

		BRPositionReceiving();							//Both Robot's position receiving
		FEActCal();								        //Get MasterRobot's actual force
        FEActSwapCal();							        //Replace a angle of actual force from the robot frame to the global frame
        //LabVIEWTransmit();						    //Trasmit all information to LabVIEW

		//Position check the communication
        if(nP!=0){
            Posdat=(double*)PosBufdat;
            ChkPosdat=(int)Posdat[0];

			Elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(End-Start).count();	//Elapse time in second unit

			//Input/Output part
			//---------------------------------------------------------------------------------------------
			//Position Part Operation
            MRobot_Manual_PositionCal();		//Get MasterRobot's Position for manual path trajectory
			//MRobot_Linear_PositionCal();		//Get MasterRobot's Position for linear path trajectory
			//MRobot_Rotate_PositionCal();		//Get MasterRobot's Position for rotate path trajectory
			//MRobot_Square_PositionCal();		//Get MasterRobot's Position for square path trajectory
			//MRobot_Circle_PositionCal();		//Get MasterRobot's Position for circle path trajectory
			//MRobot_Eight_PositionCal();		//Get MasterRobot's Position for Eight path trajectory
			//MRobot_Heart_PositionCal();		//Get MasterRobot's Position for Heart path trajectory
			CRobotPositionCal();				//Get CopyRobot's Position
            //Force Part Operation
			FIdCal();					        //Get Ideal Force
			FEVirCal();					        //Get Virtual Environment Force
            ForceCompensateCal();               //Force Compensatation Calculation

			ERobotPositionCal();				//Get ErrorRobot's Position

			//Controller part
			//---------------------------------------------------------------------------------------------			
			//OLPositionControllerCal();		//Send Global Velocity command [Vc] by [IVg] for Open loop controller
			CLPositionControllerPIDCal();		//Send Global Velocity command [Vc] by [IVg] for PID Closed loop controller
			
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
            //[PMR1 PCR1 PE VMR1 VC VCR1 Ww Vm VmI FId FEact FEvir MRTouchCheck CRTouchCheck PMR2 DIde DVir DVirMin DVirMax FEactSat Pc Pdf]
			fprintf(fpt,"%.2f, %+.1f, %+.1f, %+.1f, %+.1f, %+.1f, %+.1f, %+.0f, %+.0f, %+.1f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.4f, %+.2f, %+.2f, %+.2f, %+d, %+d, %+d, %+.2f, %+.1f, %+.2f, %+.1f, %+.2f, %+.1f, %+d, %+d, %+.1f, %+.1f, %+.2f, %+.1f, %+.2f, %+.1f, %+.0f, %+.0f, %+.2f, %+.1f, %+.1f, %+.1f, %+.1f, %+.1f, %+.1f\n",Elapsed/1000.00,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,Vm1I,Vm2I,Vm3I,FIdM,FIdMa,FEactM,FEactMa,FEvirM,FEvirMa,MRTouchChk,CRTouchChk,Pax,Pay,DIde*1000,A_DIde*(180.0/M_PI),DVir*1000,A_DVir*(180.0/M_PI),DVirMin*1000,DVirMax*1000,FEactMSat,Pcx,Pcy,Ac,Pdfx,Pdfy,Adf);
			
			End = std::chrono::system_clock::now();			//End time 1 cycle
			usleep(SamplingTime);				            //Microsecond
		}
	}
	int ret_axis = 0;
	usleep(50000);						                    //Settle time for worker thread start
    close(Possock);
	close(Forsock);
	close(LabVIEWsock);
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
    	Pd[0]=Posdat[1]/1000;		    //Pgx of MR1 in m
    	Pd[1]=Posdat[2]/1000;			//Pdy of MR1 in m
		Pd[2]=fmod(Posdat[3]*M_PI/180.0+M_PI,2*M_PI)-M_PI;	//Wrap to the range from [0,360] to [-pi, +pi] //Ad of MR1 in rad
		Pd[4]=Posdat[4]/1000;			//Vdx of MR1 in m/s
    	Pd[5]=Posdat[5]/1000;			//Vdy of MR1 in m/s
    	Pd[6]=Posdat[6]*M_PI/180.0;		//Wd  of MR1 in rad/s
	} return 0;
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
	else										{Pd[0]=0.50,Pd[1]=0.63,Pd[2]=M_PI/2.0;}
	return 0;
}
//[Pg]
double CRobotPositionCal(){
	//If flag = 2 collect the copy robot's data
    if(ChkPosdat==2){
    	Pg[0]=Posdat[1]/1000;		//Pgx of CR1 in m
    	Pg[1]=Posdat[2]/1000;		//Pgy of CR1 in m
		Pg[2]=fmod(Posdat[3]*M_PI/180.0+M_PI,2*M_PI)-M_PI;	//Wrap to the range from [0,360] to [-pi, +pi] //Ad of CR1 in rad
		Pg[4]=Posdat[4]/1000;		//Vgx of CR1 in m/s
    	Pg[5]=Posdat[5]/1000;		//Vgy of CR1 in m/s
    	Pg[6]=Posdat[6]*M_PI/180.0;	//Wg  of CR1 in rad/s
        Pg[7]=Posdat[7]/1000.0;		//Pgx of MR2 in m
    	Pg[8]=Posdat[8]/1000.0;		//Pgy of MR2 in m
	} return 0;
}
//[Pe]=[Pdf]-[Pg]
double ERobotPositionCal(){
    for(i=0;i<3;i++){
        Pe[i]=Pdf[i]-Pg[i];
    } return 0;
}
//Inverse Gobal Velocity_Matrix [IVg]
double InvGlobalVelocityCal(){
	//printf("\nInverse Gobal Velocity_Matrix [IVg]);
	for(i=0;i<3;i++){
		InvVgMatrix[i]=	Vc[i];
    //    printf("%+.4f\n",InvVgMatrix[i]);
    } return 0;
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
	printf("\n"); */
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
    } return 0;
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
    } return 0;
}
//Forwrard Wheel Velocity_Matrix = Inverse Wheel Velocity_Matrix [FWw]=[IWw]
double LinkIWwtoFWwCal(){
	for(i=0;i<3;i++){ 
		Ww[i]=InvWwMatrix[i];
	} return 0;
}
//Forwrard Wheel Velocity_Matrix [FWw]
double ForWheelVelocityCal(){
	//printf("\nForwrard Wheel Velocity_Matrix [FWw]");
	for(i=0;i<3;i++){
		ForWwMatrix[i]=	Ww[i];
    //	printf("%+.4f\n",ForWwMatrix[i]);
    } return 0;
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
    } return 0;
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
	printf("\n"); */
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
    } return 0;
}
//=================================================================

//==========Sub function for closed-loop control variable==========
//[Vd] or [Vc] Open Loop Desired/Control Velocity Command
double OLPositionControllerCal(){
	//printf("Global Velocity Matrix Command");
	for(i=0;i<3;i++){
		Vc[i]=Vd[i];
	//	printf("%+.4f\n",Vcg[i]);
	} return 0;
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
	} return 0;
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
	}
	else{
		Vc[0] = 0;
		Vc[1] = 0;
		Vc[2] = 0;
	} return 0;
}
//[Vm] 
double ConvertWWtoVmCal(){
	//printf("Motor Voltage Matrix");
	for(i=0;i<3;i++){
	//	printf("\n");
	//	[Vm]=[WwtoVoltGain]*[Ww]
    	VmMatrix[i] = ForWwMatrix[i]*WwtoVoltGain;
	//	printf("%+d",VmMatrix[i]);
	} return 0;
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

//==========Sub function for force variable==========
//Read raw signal from the sensor
int* ReadAnalogInputs() {
    FILE *Analog_files[4];                                      //Number Analog Input Pin to Read [AI0-AI3]
    const char *Analog_Files_Paths[4] = {                       //ADC file paths [AI0-AI3]
        "/sys/bus/iio/devices/iio:device0/in_voltage0_raw",     //Path AI0
        "/sys/bus/iio/devices/iio:device0/in_voltage1_raw",     //Path AI1
        "/sys/bus/iio/devices/iio:device0/in_voltage2_raw",     //Path AI2
        "/sys/bus/iio/devices/iio:device0/in_voltage3_raw",     //Path AI3
    };
    //Memory allocation error
    int* Analog_Val = (int*)malloc(4 * sizeof(int));
    if (Analog_Val == NULL) {
        perror("Memory allocation error");
        exit(EXIT_FAILURE);
    }
    //Open files for reading
    for (int i=0;i<4;i++) {
        Analog_files[i] = fopen(Analog_Files_Paths[i],"r");
        if (Analog_files[i] == NULL) {
            perror("Error opening analog input file");
            exit(EXIT_FAILURE);
        }
    }
    //Read analog input values
    for (i=0;i<4;i++) {
        int Value;
        rewind(Analog_files[i]); // Move the file pointer to the beginning
        fscanf(Analog_files[i],"%d",&Value);
        Analog_Val[i]=Value;
    }
    //Close files
    for (i=0;i<4;i++) {
        fclose(Analog_files[i]);
    }
    return Analog_Val;
}
//Actual Force Environment calculate
double FEActCal(){
    Analog_Val = ReadAnalogInputs();
    ADC_AI0=Analog_Val[0];  //FAx
    ADC_AI2=Analog_Val[2];  //FAy
    ADC_AI1=Analog_Val[1];  //FBx
    ADC_AI3=Analog_Val[3];  //FBy
    //Analog Read in Voltage Values
    Volt0=Analog_Val[0]*(1.8/4096);     //FAx
    Volt2=Analog_Val[2]*(1.8/4096);     //FAy
    Volt1=Analog_Val[1]*(1.8/4096);     //FBx
    Volt3=Analog_Val[3]*(1.8/4096);     //FBy
    //Analog Read in Newton Values
    FAx=(Analog_Val[0]*(1.8/4096))*15.5038-16.7597;  //FAx
    FAy=(Analog_Val[2]*(1.8/4096))*15.5038-16.7597;  //FAy
    FBx=(Analog_Val[1]*(1.8/4096))*15.5038-16.8527;  //FBx
    FBy=(Analog_Val[3]*(1.8/4096))*15.5038-16.7597;  //FBy 
    //Digital Lowpass Filter
    fFAx = a*fFAx+b*FAx;    //fc=10Hz Ts=0.01s
    fFAy = a*fFAy+b*FAy;    //fc=10Hz Ts=0.01s
    fFBx = a*fFBx+b*FBx;    //fc=10Hz Ts=0.01s
    fFBy = a*fFBy+b*FBy;    //fc=10Hz Ts=0.01s 

	//With Filter Process
    //1st Order Digital Lowpass Filter
    fFAx = a*fFAx+b*FAx;    //fc=10Hz Ts=0.01s
	fFAy = a*fFAy+b*FAy;    //fc=10Hz Ts=0.01s
    fFBx = a*fFBx+b*FBx;    //fc=10Hz Ts=0.01s
    fFBy = a*fFBy+b*FBy;    //fc=10Hz Ts=0.01s 
    //Result Force Robot Frame of Sensor A
    FA_R  = sqrt(pow(fFAx,2)+pow(fFAy,2));
    FAa_R = (atan2(fFAy,fFAx));
    FAx_R = FA_R*cos(FAa_R);
    FAy_R = FA_R*sin(FAa_R);
    //Result Force Global Frame of Sensor A
    FAx_G = FAx_R*cos(AngleA_off)+FAy_R*cos(AngleA_off+(M_PI/2));
    FAy_G = FAx_R*sin(AngleA_off)+FAy_R*sin(AngleA_off+(M_PI/2));    
    FA_G  = sqrt(pow(FAx_G,2)+pow(FAy_G,2));
    FAa_G = (atan2(FAy_G,FAx_G));
    //Result Force Robot Frame of Sensor B
    FB_R =sqrt(pow(fFBx,2)+pow(fFBy,2));
    FBa_R=(atan2(fFBy,fFBx));
    FBx_R = FB_R*cos(FBa_R);
    FBy_R = FB_R*sin(FBa_R);
    //Result Force Global Frame of Sensor B
    FBx_G = FBx_R*cos(AngleB_off)+FBy_R*cos(AngleB_off+(M_PI/2));
    FBy_G = FBx_R*sin(AngleB_off)+FBy_R*sin(AngleB_off+(M_PI/2));    
    FB_G  = sqrt(pow(FBx_G,2)+pow(FBy_G,2));
    FBa_G = (atan2(FBy_G,FBx_G));
    //Result Force Robot Frame of MR
    F_R =sqrt(pow(FAx_R+FBx_R,2)+pow(FAy_R+FBy_R,2));
    Fa_R=(atan2(FAy_R+FBy_R,FAx_R+FBx_R));
    //if(Fa_R<0) Fa_R=Fa_R+(2*M_PI);
    //Result Force Global Frame of MR
    F_G =sqrt(pow(FAx_G+FBx_G,2)+pow(FAy_G+FBy_G,2));
    Fa_G=(atan2(FAy_G+FBy_G,FAx_G+FBx_G));
    //if(Fa_G<0) Fa_G=Fa_G+(2*M_PI);
	if(F_G<FEactMThreshold){                                // Force actual threshold determining
    FEactM  =0;
    FEactMa =0;
    FEactx =0;
    FEacty =0;
    FEactNull =0;}
	else{
	FEactM  =F_G;
    FEactMa =Fa_G;
    FEactx =F_G*cos(Fa_G);
    FEacty =F_G*sin(Fa_G);}
	FEactNull =0;
    return 0;
}
//Replace a angle of actual force from the robot frame to the global frame [FEa]=FEa(A_MRDisplacement)
double FEActSwapCal(){
    if(FEactM<=FEactMThreshold){A_FEVir=0;}     // Force Env. Actual less than 0, Angle of Force Env. Virtual is 0
    else {A_FEVir;}

 	FEa[3]= FEactM;								// Switch data storage in array
	FEactM = FEa[3];						    // Switch data storage in array
	FEa[4]= A_FEVir;				            // Switch data storage in array (Angle of Force Env. Actual will be Angle of Force Env. Virtual)
	FEactMa= FEa[4]*(180.0/M_PI);				// Switch data storage in array (Degree to Rad)
    FEa[0]= FEactM*cos(A_FEVir);		        // Actual Force in X direction on a global coordinate frame
	FEactx= FEa[0];								// Switch data storage in array
    FEa[1]= FEactM*sin(A_FEVir);		        // Actual Force in Y direction on a global coordinate frame
    FEacty= FEa[1];								// Switch data storage in array
	FEa[2]= FEactM*0.0;							// A moment is defined as zero
	FEactNull= FEa[2];							// Switch data storage in array
	return 0;
}
//Virtual Force Environment calculate [FEVir]=[Ks]*[0.365-([PM1]-[PC1])]
double FEVirCal(){
	DVir   = sqrt(pow(Pg[7]-Pg[0],2.0)+pow(Pg[8]-Pg[1],2.0));                       //Displcement Virtual
	A_DVir = atan2((Pg[8]-Pg[1]),(Pg[7]-Pg[0]));                                    //Angle of Displcement Virtual
	if		(DVir>=0.365) 				{FEVir = 0;                     DVir;       CRTouchChk=0;}
	else if	(DVir> 0.340&&DVir<0.365)	{FEVir = Kspring*(0.365-DVir);  DVir;       CRTouchChk=1;}
	else								{FEVir = Kspring*(0.365-DVir);  DVir=0.025;              }
	FEv[0]=FEVir*cos(A_DVir);                                                       //Force Virtual Environment in X direction
	FEv[1]=FEVir*sin(A_DVir);                                                       //Force Virtual Environment in Y direction
	FEv[2]=FEVir*0;                                                                 //A Moment Environment is defined as zero
	FEv[3]=FEVir;                                                                   //Magnitude of Force Virtual Environment
    A_FEVir=A_DVir;
	FEv[4]=A_FEVir;                                                                 //Angle of Force Virtual Environment in rad
	return 0;
}
//Force Ideal calculate [FId]=[Ks]*[0.365-([PM1]-[PM2])]
double FIdCal(){
	DIde   = sqrt(pow(Pg[7]-Pd[0],2.0)+pow(Pg[8]-Pd[1],2.0));                       //Displcement Ideal
	A_DIde = atan2((Pg[8]-Pd[1]),(Pg[7]-Pd[0]));                                    //Angle Virtual
	if		(DIde>=0.365)               {FIdeal = 0;                    DIde;       MRTouchChk=0;}
	else if	(DIde> 0.340&&DIde<0.365)	{FIdeal = Kspring*(0.365-DIde); DIde;       MRTouchChk=1;}
	else								{FIdeal = Kspring*(0.365-DIde); DIde=0.025;              }
	FId[0]=FIdeal*cos(A_DIde);                                                      //Force Ideal in X direction
	FId[1]=FIdeal*sin(A_DIde);                                                      //Force Ideal in Y direction
	FId[2]=FIdeal*0;                                                                //A Moment Ideal is defined as zero
	FId[3]=FIdeal;                                                                  //Magnitude of Force Ideal
    A_FIdeal=A_DIde;
	FId[4]=A_FIdeal;                                                                //Angle of Force Ideal
	return 0;
}
//Force compensation by constant [Pdf]=[Pd]+[Pc]
double ForceCompensateCal(){
    Pc[0]=(Pcd*cos(A_DVir));       //Pcx in m.
    Pc[1]=(Pcd*sin(A_DVir));       //Pcy in m.
	Pc[2]=A_DVir;                  //Ac in rad

    Pdf[0]=Pd[0]+Pc[0];            //Pfdx in m.
    Pdf[1]=Pd[1]+Pc[1];            //Pfdx in m.
    Pdf[2]=Pd[2]+0;                //Afd in rad.   
	return 0;
}
//=================================================================

//==========Sub function for LabVIEW variable==========
//All information transmition
double LabVIEWTransmit(){
    Dataset[0]=Elapsed/1000.00;
    Dataset[1]=Pdx;    
    Dataset[2]=Pdy;   
    Dataset[3]=Ad;
    Dataset[4]=Pgx;    
    Dataset[5]=Pgy;   
    Dataset[6]=Ag;
	Dataset[7]=Vdx;    
    Dataset[8]=Vdy;   
    Dataset[9]=Wd;
	Dataset[10]=Vgx;    
    Dataset[11]=Vgy;   
    Dataset[12]=Wg;
	Dataset[13]=Vcx;    
    Dataset[14]=Vcy;   
    Dataset[15]=Wc;
    Dataset[16]=FEactx;
    Dataset[17]=FEacty;
    //Make the information storage
    sprintf(Dataset, "%06.2f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %+0.2f %+0.2f %+0.2f %+0.2f %+0.2f %+0.2f %+0.2f %+0.2f %+0.2f %0.3f %0.3f", Elapsed/1000.00,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Vdx,Vdy,Wd,Vgx,Vgy,Wg,Vcx,Vcy,Wc,FEactx,FEacty);
   //Send the data to the PC
    sendto(LabVIEWsock, Dataset, 256, 0, (const struct sockaddr*)&addr4, sizeof(addr4));
    return(0);
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
	Vm1=ForWwMatrix[0]*WwtoVoltGain;		// Voltage of motor 1	float number									;volt
	Vm2=ForWwMatrix[1]*WwtoVoltGain;		// Voltage of motor 2	float number									;volt
	Vm3=ForWwMatrix[2]*WwtoVoltGain;		// Voltage of motor 3	float number									;volt
	Vm1I=VmMatrix[0];						// Voltage of motor 1	integer number									;volt
	Vm2I=VmMatrix[1];						// Voltage of motor 2	integer number									;volt
	Vm3I=VmMatrix[2];						// Voltage of motor 3	integer number									;volt
	//===Position of a Robot===
	Pdx=Pd[0]*1000; Pdy=Pd[1]*1000; Ad =Pd[2]*(180.0/M_PI);
	Pgx=Pg[0]*1000; Pgy=Pg[1]*1000; Ag =Pg[2]*(180.0/M_PI);
    Pex=Pe[0]*1000; Pey=Pe[1]*1000; Ae =Pe[2]*(180.0/M_PI);
    Pax=Pg[7]*1000; Pay=Pg[8]*1000;
    Pcx=Pc[0]*1000; Pcy=Pc[1]*1000; Ac =Pc[2]*(180.0/M_PI);
    Pdfx=Pdf[0]*1000; Pdfy=Pdf[1]*1000; Adf =Pdf[2]*(180.0/M_PI);
	//===Velocity of a Robot===
	Vdx=Pd[4]; Vdy=Pd[5]; Wd =Pd[6];
	Vcx=Vc[0]; Vcy=Vc[1]; Wc =Vc[2];
	Vgx=Pg[4]; Vgy=Pg[5]; Wg =Pg[6];
	//===Force of a Robot===
    FIdx=FId[0];     FIdy=FId[1];     FIdM=FId[3];     FIdMa=FId[4]*(180.0/M_PI);
    FEvirx=FEv[0];   FEviry=FEv[1];   FEvirM=FEv[3];   FEvirMa=FEv[4]*(180.0/M_PI);

	//(Reverse Kinematic Parameters) Inverse Velocity Gloval, Inverse Velocity Robot, Inverse Velocity Wheel, Forward Voltage, Angle Global 
	//printf("Inverse Parameters | IVd: %+.4f %+.4f %+.4f | IVr: %+.4f %+.4f %+.4f | IWw: %+.4f %+.4f %+.4f | IVm: %+.1f %+.1f %+.1f | Ag: %+.4f\n",Vd[0],Vd[1],Vd[2],IVrx,IVry,IWr,IWw1,IWw2,IWw3,Vm1,Vm2,Vm3,Ag);
	//(Forward Kinematic Parameters) Foward Velocity Wheel, Inverse Voltage, Forward Velocity Robot, Forward Velocity Global, Angle Global 
	//printf("Forward Parameters | FWw: %+.4f %+.4f %+.4f | FVm: %+.1f %+.1f %+.1f | FVr: %+.4f %+.4f %+.4f | FVg: %+.4f %+.4f %+.4f | Ag: %+.4f\n\n",FWw1,FWw2,FWw3,Vm1,Vm2,Vm3,FVrx,FVry,FWr,FVgx,FVgy,FWg,Ag);
    //(Position and velocities display) Position MR1, Position CR, Position Error, Velocity MR1, Velocity Controlled, Velocity CR1, Velocity Wheel, Voltage (Integer)
	//printf("T: %06.2f | Pd: %+.2f %+.2f %+.2f | Pg: %+.2f %+.2f %+.2f | Pe: %+.2f %+.2f %+.2f | Vd: %+.2f %+.2f %+.2f | Vc: %+.2f %+.2f %+.2f | Vg: %+.2f %+.2f %+.2f | Ww: %+.2f %+.2f %+.2f | Vm: %+.1f %+.1f %+.1f| \n",Elapsed/1000,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vdx,Vdy,Wd,Vcx,Vcy,Wc,Vgx,Vgy,Wg,FWw1,FWw2,FWw3,Vm1,Vm2,Vm3);				
    //(Position) Position MR1, Position MR2, Position CR1
    //printf("T: %06.2f | Pd: %+0.0f %+0.0f | Pg: %+0.0f %+0.0f | Pa: %+0.0f %+0.0f\n",Elapsed/1000.00,Pd[0],Pd[1],Pg[7],Pg[8],Pg[0],Pg[1]);
    //(Displacement) Displacement MR1, Displacement MR2, Displacement CR1, Touch Check
    //printf("T: %06.2f | DIde: %+0.3f (%+0.3f) | Dvir: %+0.3f (%+0.3f) | MRChk: %d\n",Elapsed/1000.00,DIde,A_DIde,DVir,A_DVir,MRTouchChk);
    //(Force in Rectangular Form) Force Ideal, Force Env. Actual, Force Env. Virtual, Touch Check
    //printf("T: %06.2f | FId: %+0.3f %+0.3f | FEact: %+0.3f %+0.3f | FEvir: %+0.3f %+0.3f | MRChk: %d\n",Elapsed/1000.00,FIdx,FIdy,FEactx,FEacty,FEvirx,FEviry,MRTouchChk);
   	//(Force in Polar Form) Force Ideal, Force Env. Actual, Force Env. Virtual, Touch Check
    //printf("T: %06.2f | FId: %+0.3f (%+0.3f) | FEact: %+0.3f (%+0.3f) | FEvir: %+0.3f (%+0.3f) | MRChk: %d\n",Elapsed/1000.00,FIdM,FIdMa,FEactM,FEactMa,FEvirM,FEvirMa,MRTouchChk);
   	//(Position and Force diaplay) Position MR1, Position CR1, Position Error, Voltage, Force Ideal, Force Env. Actual, Force Env. Virtual, Touch Check, Position MR2, Displacement Ideal, Displacement Virtual
    //printf("T: %06.2f | Pd: %+3.1f %+3.1f %+.1f | Pg: %+3.1f %+3.1f %+.1f | Pe: %+3.0f %+3.0f %+.1f | Vm: %+.1f %+.1f %+.1f | FId: %+0.2f (%+0.1f) | FEact: %+0.2f (%+0.1f) | FEvir: %+0.2f (%+0.1f) | TChk: %d %d | Pa: %+3.1f %+3.1f | DIde: %+3.2f (%+3.1f) | Dvir: %+3.2f (%+3.1f)\n",Elapsed/1000.00,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vm1,Vm2,Vm3,FIdM,FIdMa,FEactM,FEactMa,FEvirM,FEvirMa,MRTouchChk,CRTouchChk,Pax,Pay,DIde*1000,A_DIde*(180.0/M_PI),DVir*1000,A_DVir*(180.0/M_PI));
    //(Position and Force diaplay) Position MR1, Position CR1, Position Error, Voltage, Force Ideal, Force Env. Actual, Force Env. Virtual, Touch Check, Position MR2, Displacement Ideal, Displacement Virtual
    //printf("T: %06.2f | Pd: %+3.1f %+3.1f %+.1f | Pg: %+3.1f %+3.1f %+.1f | Pe: %+3.0f %+3.0f %+.1f | Vm: %+.1f %+.1f %+.1f | FId: %+0.2f (%+0.1f) | FEact: %+0.2f (%+0.1f) | FEvir: %+0.2f (%+0.1f) | TChk: %d %d | Pa: %+3.1f %+3.1f | DIde: %+3.2f (%+3.1f) | Dvir: %+3.2f (%+3.1f)\n",Elapsed/1000.00,Pdx,Pdy,Ad,Pgx,Pgy,Ag,Pex,Pey,Ae,Vm1,Vm2,Vm3,FIdM,FIdMa,FEactM,FEactMa,FEvirM,FEvirMa,MRTouchChk,CRTouchChk,Pax,Pay,DIde*1000,A_DIde*(180.0/M_PI),DVir*1000,A_DVir*(180.0/M_PI));
    //(Compensation display) Position MR1, Position MR1 Compensate Desired, Position MR1 Final, Position CR1, Position Error, Position MR2, Voltage, Displacement Ideal, Displacement Virtual, Touch Check
    //printf("T: %06.2f | Pd: %+3.1f %+3.1f %+.1f | Pc: %+3.1f %+3.1f, %+3.1f (%+.1f) | Pdf: %+3.1f %+3.1f (%+.1f) | Pg: %+3.1f %+3.1f %+.1f | Pe: %+1.0f %+1.0f %+.1f | Pa: %+1.1f %+1.1f | Vm: %+.1f %+.1f %+.1f | DIde: %+3.2f (%+3.1f) | Dvir: %+3.2f (%+3.1f) | TChk: %d %d %d| \n",Elapsed/1000.00,Pdx,Pdy,Ad,Pcx,Pcy,Pcd*1000,Ac,Pdfx,Pdfy,Adf,Pgx,Pgy,Ag,Pex,Pey,Ae,Pax,Pay,Vm1,Vm2,Vm3,DIde*1000,A_DIde*(180.0/M_PI),DVir*1000,A_DVir*(180.0/M_PI),MRTouchChk,CRTouchChk,CRTouchChk,CompenChk);
   	//(Position Compensation and Force) Position MR1, Position MR1 Compensate Desired, Position MR1 Final, Position CR1, Position Error, Voltage, Force Ideal, Force Actual, Touch Check
    printf("T: %06.2f | Pd: %+3.1f %+3.1f %+.1f | Pc: %+3.1f %+3.1f, %+3.1f (%+.1f) | Pdf: %+3.1f %+3.1f (%+.1f) | Pg: %+3.1f %+3.1f %+.1f | Pe: %+1.1f %+1.1f %+.1f | Vm: %+.1f %+.1f %+.1f | FIde: %+3.2f (%+3.1f) | FEact: %+0.2f (%+0.1f) | TChk: %d %d %d| \n",Elapsed/1000.00,Pdx,Pdy,Ad,Pcx,Pcy,Pcd*1000,Ac,Pdfx,Pdfy,Adf,Pgx,Pgy,Ag,Pex,Pey,Ae,Vm1,Vm2,Vm3,FIdM,FIdMa,FEactM,FEactMa,MRTouchChk,CRTouchChk,CRTouchChk,CompenChk);
    return 0;
}
//=================================================================