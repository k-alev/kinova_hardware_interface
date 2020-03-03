#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
//#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <dlfcn.h>

#define OPTIMAL_Z_PARAM_SIZE 16
#define COMMAND_SIZE 6
#define GRAVITY_PAYLOAD_SIZE 4
#define GRAVITY_PARAM_SIZE 42

int main(int argc, char **argv)
{

  //////Kinova API///////
  int result, resultComm;
  AngularPosition data;
  CartesianPosition dataC;
  TrajectoryPoint trajectoryPoint;
  //General
  int (*MyInitAPI)();
  int (*MyCloseAPI)();
  int (*MyGetAngularPosition)(AngularPosition &);
  int (*MyGetAngularVelocity)(AngularPosition &);
  int (*MyGetAngularForce)(AngularPosition &);
  int (*MyGetCartesianForce)(CartesianPosition &);
  int (*MyGetAngularForceGravityFree)(AngularPosition &);
  int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &);
  int (*MySetActiveDevice)(KinovaDevice);
  int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
  int (*MyMoveHome)();
  //Torque Control
  int (*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
  int (*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
  int (*MySetTorqueSafetyFactor)(float factor);
  int (*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
  int (*MySetGravityVector)(float Command[3]);
  int (*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
  int (*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
  int (*MySetGravityType)(GRAVITY_TYPE Type);
  int (*MySetTorqueVibrationController)(float value);
  int (*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
  int (*MyGetAngularTorqueCommand)(float Command[COMMAND_SIZE]);
  int (*MyGetAngularCurrentMotor)(AngularPosition &);
  int (*MyGetAngularCurrent)(AngularPosition &);
  int (*MyGetAngularTorqueGravityEstimation)(float Command[COMMAND_SIZE]);

  //Initializing Kinova API
  result = 0;
  //Handle for the library's command layer.
  void *commandLayer_handle;

  //We load the library (Under Windows, use the function LoadLibrary)
  commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
  //A vector that holds all devices found on the USB bus.
  KinovaDevice devicesList[MAX_KINOVA_DEVICE];
  //We load the functions from the library (Under Windows, use GetProcAddress)
  MyInitAPI = (int (*)())dlsym(commandLayer_handle, "InitAPI");
  MyCloseAPI = (int (*)())dlsym(commandLayer_handle, "CloseAPI");
  MyGetAngularPosition = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularPosition");
  MyGetAngularVelocity = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularVelocity");
  MyGetCartesianForce = (int (*)(CartesianPosition &))dlsym(commandLayer_handle, "GetCartesianForce");
  MyGetAngularForce = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularForce");
  MyGetAngularForceGravityFree = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularForceGravityFree");
  MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &))dlsym(commandLayer_handle, "GetDevices");
  MySetActiveDevice = (int (*)(KinovaDevice))dlsym(commandLayer_handle, "SetActiveDevice");
  MySendAdvanceTrajectory = (int (*)(TrajectoryPoint))dlsym(commandLayer_handle, "SendAdvanceTrajectory");

  MyRunGravityZEstimationSequence = (int (*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]))dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
  MySwitchTrajectoryTorque = (int (*)(GENERALCONTROL_TYPE))dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
  MySetTorqueSafetyFactor = (int (*)(float))dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
  MySendAngularTorqueCommand = (int (*)(float Command[COMMAND_SIZE]))dlsym(commandLayer_handle, "SendAngularTorqueCommand");
  MySetGravityVector = (int (*)(float Command[3]))dlsym(commandLayer_handle, "SetGravityVector");
  MySetGravityPayload = (int (*)(float Command[GRAVITY_PAYLOAD_SIZE]))dlsym(commandLayer_handle, "SetGravityPayload");
  MySetGravityOptimalZParam = (int (*)(float Command[GRAVITY_PARAM_SIZE]))dlsym(commandLayer_handle, "SetGravityOptimalZParam");
  MySetGravityType = (int (*)(GRAVITY_TYPE Type))dlsym(commandLayer_handle, "SetGravityType");
  MyGetAngularForceGravityFree = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularForceGravityFree");
  MySetTorqueVibrationController = (int (*)(float))dlsym(commandLayer_handle, "SetTorqueVibrationController");
  MySetTorqueControlType = (int (*)(TORQUECONTROL_TYPE))dlsym(commandLayer_handle, "SetTorqueControlType");
  MyMoveHome = (int (*)())dlsym(commandLayer_handle, "MoveHome");
  MyGetAngularTorqueCommand = (int (*)(float Command[COMMAND_SIZE]))dlsym(commandLayer_handle, "GetAngularTorqueCommand");
  MyGetAngularCurrentMotor = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularCurrentMotor");
  MyGetAngularCurrent = (int (*)(AngularPosition &))dlsym(commandLayer_handle, "GetAngularCurrent");
  MyGetAngularTorqueGravityEstimation = (int (*)(float Command[COMMAND_SIZE]))dlsym(commandLayer_handle,
                                                                                    "GetAngularTorqueGravityEstimation");

  //Check if API was loaded correctly
  if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularPosition == NULL) || (MyGetAngularForce == NULL) || (MyGetAngularForceGravityFree == NULL) || (MyGetAngularVelocity == NULL) || (MySendAdvanceTrajectory == NULL) || (MySwitchTrajectoryTorque == NULL) || (MySetTorqueControlType == NULL) || (MySetTorqueSafetyFactor == NULL) || (MySetTorqueVibrationController == NULL) || (MySendAngularTorqueCommand == NULL) || (MyGetCartesianForce == NULL))
  {
    //ROS_WARN("Unable to initialize the command layer." );
    ROS_ERROR("Unable to initialize the command layer.");
    return false;
  }
  result = (*MyInitAPI)();
  // Get the angular command to test the communication with the robot
  resultComm = MyGetAngularPosition(data);

  if (result != NO_ERROR_KINOVA && resultComm != NO_ERROR_KINOVA)
  {
    ROS_ERROR("Unable to initialize API.");
    std::cout << std::endl
              << "Calling the method CloseAPI()" << std::endl;
    result = (*MyCloseAPI)();
    std::cout << "result of CloseAPI() = " << result << std::endl;
    return false;
  }

  ros::init(argc, argv, "talker_frc");
  ros::NodeHandle n;
  ros::Publisher talker_frc_pub = n.advertise<std_msgs::String>("talker_frc", 1000);
  ros::Rate loop_rate(4000);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    //result = (MyGetAngularForce)(data);
    //result = (MyGetAngularForce)(data);
    //result = (*MyGetAngularPosition)(data);
    talker_frc_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
