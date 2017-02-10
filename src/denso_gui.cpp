// Always goes first
#define _CUSTOM_UINT64
// ROS
#include <ros/ros.h>
#include "bcap/stdint.h"

#include "../include/denso_gui/denso_gui.hpp"
//#include <atlbase.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>  

// bCAP (Always last)
#include "bcap/bcap_client.h"

#define E_BUF_FULL  (0x83201483)

#define SERVER_IP_ADDRESS "tcp:192.168.0.1"
#define SERVER_PORT_NUM 5007
#define PERIOD 100
#define AMPLITUDE 15
namespace denso_gui{
namespace d_gui{

HRESULT initArm(void);


QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


bool QNode::init() {
ROS_INFO("begin init");
	int fargc = 0;
	char** fargv = (char**)malloc(sizeof(char*)*(fargc+1));
	ros::init(fargc,fargv,"DensoNode");
	free(fargv);
	if ( ! ros::master::check() ) {
		return false;

	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	ros::NodeHandle nh;
	// Add your ros communications here.
  //Initialize Denso Arm
   ROS_INFO("Starting denso stuff");
  /* Init socket */
  hr = bCap_Open_Client(SERVER_IP_ADDRESS, SERVER_PORT_NUM, 0, &fd);
  if FAILED(hr) ROS_ERROR("command failed %x",hr);
  else ROS_INFO("Socket Created");
  /* Start b-CAP service */
  hr = bCap_ServiceStart(fd, NULL);
  if FAILED(hr)ROS_ERROR("command failed %x",hr);
  else ROS_INFO("b-CAP started");
  /* Get controller handle */
  BSTR bstrName, bstrProv, bstrMachine, bstrOpt;
  bstrName = SysAllocString(L"");
  bstrProv = SysAllocString(L"CaoProv.DENSO.VRC");
  bstrMachine = SysAllocString(L"localhost");
  bstrOpt = SysAllocString(L"");
  hr = bCap_ControllerConnect(fd, bstrName, bstrProv, bstrMachine, bstrOpt, &hCtrl);
  SysFreeString(bstrName);
  SysFreeString(bstrProv);
  SysFreeString(bstrMachine);
  SysFreeString(bstrOpt);
  if FAILED(hr) ROS_ERROR("Move command failed %x",hr);
  else ROS_INFO("Received controller handle");

  /* Get robot handle */
  BSTR bstrRobotName, bstrRobotOpt;
  bstrRobotName = SysAllocString(L"Arm");
  bstrRobotOpt = SysAllocString(L"");
  hr = bCap_ControllerGetRobot(fd, hCtrl, bstrRobotName, bstrRobotOpt, &hRobot);
  SysFreeString(bstrRobotName);
  SysFreeString(bstrRobotOpt);
  if FAILED(hr) ROS_ERROR("Move command failed %x",hr);
  else ROS_INFO("Received robot handle");
  
 
  //Get error flag handle
  BSTR nullstr;
  bstrCommand = SysAllocString(L"@ERROR_CODE");
  nullstr = SysAllocString(L"");
  hr = bCap_ControllerGetVariable(fd, hRobot, bstrCommand, nullstr, &hErr);
  SysFreeString(bstrCommand);
  SysFreeString(nullstr);
  if FAILED(hr){
  printf("Could not get handle for errors %x\n", hr);
   ROS_INFO("FAIL");
   }
  else {
  //Read error value
  vntResult.vt = VT_I4;
  hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  ROS_INFO("Errors %d", vntErr.lVal);
  eStatus = vntErr.lVal;
  VariantClear(&vntErr);
  }
  //Sometimes it takes multiple calls to clear all the errors so its in a while loop
  while(ros::ok() && eStatus !=  0){
  ROS_INFO("Clearing errors");
  // Clear previous errors
  bstrCommand = SysAllocString(L"ClearError");
  vntParam.lVal = 0;
  vntParam.vt = VT_I4;
  hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
  if (FAILED(hr))
    ROS_ERROR("[ClearError] command failed %x",hr);
  else ROS_INFO("Errors Cleared");
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  VariantClear(&vntErr);
  //Check for error codes
  hr = bCap_VariableGetValue(fd, hErr, &vntErr);
  ROS_INFO("Errors %d", vntErr.lVal);
  eStatus = vntErr.lVal;
  VariantClear(&vntErr);
  }
  
  //Get ROBSLAVE task handle
  bstrCommand = SysAllocString(L"ROBSLAVE");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"1");
  hr = bCap_ControllerGetTask(fd, hCtrl, bstrCommand, NULL, &hTask);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr) {
  ROS_INFO("Could not find task handle %x", hr);
  return (hr);
  }else ROS_INFO("Got Handle: %d",hTask);
  
  //Wait a moment then begin ROBSLAVE task
  ros::Duration(3).sleep();
  hr = bCap_TaskStart(fd, hTask, 1, NULL);
  if FAILED(hr) {
  ROS_INFO("Could not start task: %x",hr);
  return (hr);
  }else ROS_INFO("Started Task");
  VARIANT vntPose;
  
  /* Get handle for current angle */
  bstrCommand = SysAllocString(L"@CURRENT_ANGLE");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &jHandle);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("Could not get handle %x\n", hr);
   return (hr);
   }
   //Read joint values
  hr = bCap_VariableGetValue(fd, jHandle, &vJnt);
  SafeArrayAccessData(vJnt.parray, (void**)&pdArray);
  memcpy(dJnt, pdArray, sizeof(dJnt));
  SafeArrayUnaccessData(vJnt.parray);
  VariantClear(&vJnt);
  ROS_INFO("RET %x, Val %f -- %f -- %f -- %f -- %f -- %f Fig: %f",hr, dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);
  
  //Get speed handle
  bstrCommand = SysAllocString(L"@SPEED");
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"");
  nullstr = SysAllocString(L"");
  hr = bCap_RobotGetVariable(fd, hRobot, bstrCommand, nullstr, &hSpd);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("Could not get handle for speed %x\n", hr);
   return (hr);
   }
  else {
  //Read speed value
  vSpd.vt = VT_R4;
  hr = bCap_VariableGetValue(fd, hSpd, &vSpd);
  ROS_INFO("Speed %f", vSpd.fltVal);
  VariantClear(&vSpd);
  }
  //Set speed vlaue
  vSpd.fltVal = 100;
  bstrCommand = SysAllocString(L"EXTSPEED");
  dnt[0] = 50.0; //Velocity
  dnt[1] = 100.0; //Acceleration
  dnt[2] = 100.0; //Decceleration
  vntS.vt = (VT_R4 | VT_ARRAY);
  vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
  SafeArrayAccessData(vntS.parray, (void**)&pdArray);
  memcpy(pdArray, dnt, sizeof(dnt));
  SafeArrayUnaccessData(vntS.parray);
  hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vSpd);
  SysFreeString(bstrCommand);
  VariantClear(&vntS);
  VariantClear(&vSpd);
  if FAILED(hr){
  ROS_INFO("Ext Speed not set %x\n", hr);
   return (hr);
   }
  else ROS_INFO("Ext Speed Set");

  //Turn Motor ON
  bstrCommand = SysAllocString(L"Motor");
  vntParam.vt = VT_I2;
  vntParam.iVal = 1;
  hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("FAIL %x\n", hr);
   return (hr);
   }
  else ROS_INFO("Motor On");

  //Home the arm
  vntParam.vt = VT_BSTR;
  vntParam.bstrVal = SysAllocString(L"@P J(0,0,90,0,0,0,0)");
  bstrCommand = SysAllocString(L"");
  hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
  SysFreeString(bstrCommand);
  VariantClear(&vntParam);
  if FAILED(hr){
  printf("FAIL %x\n", hr);
   return (hr);
   }
  else ROS_INFO("Homed");
return(hr);

	start();
	return true;
}

void QNode::run() {
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;


ROS_INFO("Shutting Down");
//Turn off motor
bstrCommand = SysAllocString(L"Motor");
vntParam.iVal = 0;
vntParam.vt = VT_I2;
hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
SysFreeString(bstrCommand);
VariantClear(&vntParam);
if FAILED(hr) {
ROS_INFO("Motors Not off: %x",hr);
}else ROS_INFO("Motor Off");
/* Release robot handle */
bCap_RobotRelease(fd, &hRobot);
/* Release controller handle */
bCap_ControllerDisconnect(fd, &hCtrl);
/* Stop b-CAP service (Very important in UDP/IP connection) */
bCap_ServiceStop(fd);
/* Close socket */
bCap_Close_Client(&fd);

	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



void QNode::send_ptp_command(float x, float y, float z, float rx, float ry, float rz, float fig){

	int n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,%f)",x,y,z,rx,ry,rz,fig);
	ROS_INFO("Moving ptp to: %s",buffer);
	n=swprintf(wstr,80, L"%s",buffer);
	vntParam.vt = VT_BSTR;
	vntParam.bstrVal = SysAllocString(wstr);
	bstrCommand = SysAllocString(L"");
	hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if (FAILED(hr))
	ROS_ERROR("Move command failed %x",hr);
	else ROS_INFO("Moved");

}
void QNode::send_cp_command(float x, float y, float z, float rx, float ry, float rz, float fig){

	int n=sprintf(buffer,"@P P(%f,%f,%f,%f,%f,%f,%f)",x,y,z,rx,ry,rz,fig);
	ROS_INFO("Moving cp to: %s",buffer);
	n=swprintf(wstr,80, L"%s",buffer);
	vntParam.vt = VT_BSTR;
	vntParam.bstrVal = SysAllocString(wstr);
	bstrCommand = SysAllocString(L"");
	hr = bCap_RobotMove(fd, hRobot, 2L, vntParam, bstrCommand);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if (FAILED(hr))
	ROS_ERROR("Move command failed %x",hr);
	else ROS_INFO("Moved");
	 

}
void QNode::send_joint_command(float j1, float j2, float j3, float j4, float j5, float j6){

	int n=sprintf(buffer,"@P J(%f,%f,%f,%f,%f,%f,0)",j1,j2,j3,j4,j5,j6);
	ROS_INFO("Moving joints to: %s",buffer);
	n=swprintf(wstr,80, L"%s",buffer);
	vntParam.vt = VT_BSTR;
	vntParam.bstrVal = SysAllocString(wstr);
	bstrCommand = SysAllocString(L"");
	hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if (FAILED(hr))
	ROS_ERROR("Move command failed %x",hr);
	else ROS_INFO("Moved");
	 

}
void QNode::send_shutdown(){

	//Turn off motor
	bstrCommand = SysAllocString(L"Motor");
	vntParam.iVal = 0;
	vntParam.vt = VT_I2;
	hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntParam, &vntResult);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if FAILED(hr) {
	ROS_INFO("Motors Not off: %x",hr);
	}else ROS_INFO("Motor Off");
	/* Release robot handle */
	bCap_RobotRelease(fd, &hRobot);
	/* Release controller handle */
	bCap_ControllerDisconnect(fd, &hCtrl);
	/* Stop b-CAP service (Very important in UDP/IP connection) */
	bCap_ServiceStop(fd);
	/* Close socket */
	bCap_Close_Client(&fd);

}
void QNode::clear_errors(){
	vntResult.vt = VT_I4;
	hr = bCap_VariableGetValue(fd, hErr, &vntErr);
	ROS_INFO("Errors %d", vntErr.lVal);
	eStatus = vntErr.lVal;
	VariantClear(&vntErr);

	while(ros::ok() && eStatus !=  0){
		ROS_INFO("Clearing errors");
		// Clear previous errors
		bstrCommand = SysAllocString(L"ClearError");
		vntParam.lVal = 0;
		vntParam.vt = VT_I4;
		hr = bCap_ControllerExecute(fd, hCtrl, bstrCommand, vntParam, &vntErr);
		if (FAILED(hr))
		ROS_ERROR("[ClearError] command failed %x",hr);
		else ROS_INFO("Errors Cleared");
		SysFreeString(bstrCommand);
		VariantClear(&vntParam);
		VariantClear(&vntErr);
		//Check for error codes
		hr = bCap_VariableGetValue(fd, hErr, &vntErr);
		ROS_INFO("Errors %d", vntErr.lVal);
		eStatus = vntErr.lVal;
		VariantClear(&vntErr);
	}

}
void QNode::send_string(std::string user_string){

	std::cout << "Moving to" << user_string;
	int n=swprintf(wstr,80, L"%s",user_string.c_str());
	vntParam.vt = VT_BSTR;
	vntParam.bstrVal = SysAllocString(wstr);
	bstrCommand = SysAllocString(L"");
	hr = bCap_RobotMove(fd, hRobot, 1L, vntParam, bstrCommand);
	SysFreeString(bstrCommand);
	VariantClear(&vntParam);
	if (FAILED(hr))
	ROS_ERROR("Move command failed %x",hr);
	else ROS_INFO("Moved");
	 

}
void QNode::send_speed(float speed){

	//Set speed vlaue
	bstrCommand = SysAllocString(L"EXTSPEED");
	dnt[0] = speed; //Velocity
	dnt[1] = 100.0; //Acceleration
	dnt[2] = 100.0; //Decceleration
	vntS.vt = (VT_R4 | VT_ARRAY);
	vntS.parray = SafeArrayCreateVector(VT_R4, 0, 1);
	SafeArrayAccessData(vntS.parray, (void**)&pdArray);
	memcpy(pdArray, dnt, sizeof(dnt));
	SafeArrayUnaccessData(vntS.parray);
	hr = bCap_RobotExecute(fd, hRobot, bstrCommand, vntS, &vSpd);
	SysFreeString(bstrCommand);
	VariantClear(&vntS);
	VariantClear(&vSpd);
	if FAILED(hr){
	ROS_INFO("Ext Speed not set %x\n", hr);
	}
	else ROS_INFO("Ext Speed Set");

}

}
};


