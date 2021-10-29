#include <LRemote.h>
#include <Ultrasonic.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "math.h"

MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float pitch;
float roll;

#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;


Ultrasonic ultrasonic(3, 4);

LRemoteLabel dislabel;
LRemoteButton button_forward;
LRemoteButton button_backward;
LRemoteButton button_left;
LRemoteButton button_right;

LRemoteSwitch switchButton_trace;

float dis;

boolean IR_left;
boolean IR_right;

int mode = 0;

int runtime = 25;

void setup()
{
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
  Serial.println("     ");
  
  
  LRemote.setName("Future GO !!");
  LRemote.setOrientation(RC_PORTRAIT);
  LRemote.setGrid(6, 8);
  
  dislabel.setPos(0, 0);
  dislabel.setText("Distance");
  dislabel.setSize(6, 2);
  dislabel.setColor(RC_ORANGE);
  LRemote.addControl(dislabel);
  
  button_forward.setPos(2, 4);
  button_forward.setText("^");
  button_forward.setSize(2, 2);
  button_forward.setColor(RC_BLUE);
  LRemote.addControl(button_forward);

  button_backward.setPos(2, 6);
  button_backward.setText("v");
  button_backward.setSize(2, 2);
  button_backward.setColor(RC_BLUE);
  LRemote.addControl(button_backward);

  button_left.setPos(0, 6);
  button_left.setText("<");
  button_left.setSize(2, 2);
  button_left.setColor(RC_BLUE);
  LRemote.addControl(button_left);

  button_right.setPos(4, 6);
  button_right.setText(">");
  button_right.setSize(2, 2);
  button_right.setColor(RC_BLUE);
  LRemote.addControl(button_right);
  
  switchButton_trace.setPos(4, 2);
  switchButton_trace.setText("Trace mode");
  switchButton_trace.setSize(2, 2);
  switchButton_trace.setColor(RC_YELLOW);
  LRemote.addControl(switchButton_trace);

  LRemote.begin();
  
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(7, INPUT);
  pinMode(6, INPUT);
}


void loop()
{
  LRemote.process();
  if (button_forward.isValueChanged()) {
    if (button_forward.getValue() == 1) {
      Forward();
    } else {
      Stop();
    }
  }
  
  if (button_backward.isValueChanged()) {
    if (button_backward.getValue() == 1) {
      Backward();
    } else {
      Stop();
    }
  }
  
  if (button_left.isValueChanged()) {
    if (button_left.getValue() == 1) {
      Leftward();
    } else {
      Stop();
    }
  }
  
  if (button_right.isValueChanged()) {
    if (button_right.getValue() == 1) {
      Rightward();
    } else {
      Stop();
    }
  }
  
  
  dis = ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);
  if (dis > 25)  dislabel.updateText("Clear!!");
  else dislabel.updateText("Obs.: " + String(dis) + "cm");
  
  
  
  
  if(switchButton_trace.isValueChanged()){
    if(switchButton_trace.getValue() == 1) {
		mode = 1;
	}
	else {
		Stop();
		mode = 0;
	}
  }
  
	IR_left = digitalRead(6);
	IR_right = digitalRead(7);
  if (mode == 1) {
    if (dis < 10) {
		Rightward();
		delay(runtime);
		Stop();
    }
    else if ((IR_left == 1) && (IR_right == 1)) {
		Forward();
		delay(runtime);
		Stop();
    }
    else if ((IR_left == 0) && (IR_right == 1)){
		Rightward();
		delay(runtime);
		Stop();
    }
    else if ((IR_left == 1) && (IR_right == 0)){
		Leftward();
		delay(runtime);
		Stop();
    }
    else {
		Backward();
		delay(runtime);
		Stop();
    }
  }  
}

void Forward(){
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
}

void Backward(){
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
}

void Rightward(){
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
}

void Leftward(){
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
}

void Stop(){
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}



void Mxyz_init_calibrated ()
{
	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
	Serial.print("  ");
	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
	Serial.print("  ");
	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");

	get_calibration_Data ();

	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
for (int i=0; i<sample_num_mdate;i++)
	{
	get_one_sample_date_mxyz();
	/*
	Serial.print(mx_sample[2]);
	Serial.print(" ");
	Serial.print(my_sample[2]);                            //you can see the sample data here .
	Serial.print(" ");
	Serial.println(mz_sample[2]);
	*/



	if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
	if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
	if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		

	if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
	if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
	if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];

	}

	mx_max = mx_sample[1];
	my_max = my_sample[1];
	mz_max = mz_sample[1];			

	mx_min = mx_sample[0];
	my_min = my_sample[0];
	mz_min = mz_sample[0];



	mx_centre = (mx_max + mx_min)/2;
	my_centre = (my_max + my_min)/2;
	mz_centre = (mz_max + mz_min)/2;	

}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
}	


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
	
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;	
	
	//Mxyz[0] = (double) mx * 1200 / 4096;
	//Mxyz[1] = (double) my * 1200 / 4096;
	//Mxyz[2] = (double) mz * 1200 / 4096;
	Mxyz[0] = (double) mx * 4800 / 8192;
	Mxyz[1] = (double) my * 4800 / 8192;
	Mxyz[2] = (double) mz * 4800 / 8192;
}

void getCompassDate_calibrated ()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;	
}