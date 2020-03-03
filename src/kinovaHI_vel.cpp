#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
//#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <dlfcn.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include <kinova_hardware_interface/array_torque_sensors.h>
#include <signal.h>

#define OPTIMAL_Z_PARAM_SIZE 16
#define COMMAND_SIZE 6
#define GRAVITY_PAYLOAD_SIZE 4
#define GRAVITY_PARAM_SIZE 42

sig_atomic_t volatile g_sigint_request = 0;

class KinovaMico : public hardware_interface::RobotHW
{
public:
    KinovaMico()
    {

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("m1n6s200_joint_1", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointStateHandle state_handle_b("m1n6s200_joint_2", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        hardware_interface::JointStateHandle state_handle_c("m1n6s200_joint_3", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_c);

        hardware_interface::JointStateHandle state_handle_d("m1n6s200_joint_4", &pos[3], &vel[3], &eff[3]);
        jnt_state_interface.registerHandle(state_handle_d);

        hardware_interface::JointStateHandle state_handle_e("m1n6s200_joint_5", &pos[4], &vel[4], &eff[4]);
        jnt_state_interface.registerHandle(state_handle_e);

        hardware_interface::JointStateHandle state_handle_f("m1n6s200_joint_6", &pos[5], &vel[5], &eff[5]);
        jnt_state_interface.registerHandle(state_handle_f);

        registerInterface(&jnt_state_interface);

        // connect and register the joint torque interface
        hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("m1n6s200_joint_1"), &cmd[0]);
        jnt_vel_interface.registerHandle(vel_handle_a);

        hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("m1n6s200_joint_2"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_b);

        hardware_interface::JointHandle vel_handle_c(jnt_state_interface.getHandle("m1n6s200_joint_3"), &cmd[2]);
        jnt_vel_interface.registerHandle(vel_handle_c);

        hardware_interface::JointHandle vel_handle_d(jnt_state_interface.getHandle("m1n6s200_joint_4"), &cmd[3]);
        jnt_vel_interface.registerHandle(vel_handle_d);

        hardware_interface::JointHandle vel_handle_e(jnt_state_interface.getHandle("m1n6s200_joint_5"), &cmd[4]);
        jnt_vel_interface.registerHandle(vel_handle_e);

        hardware_interface::JointHandle vel_handle_f(jnt_state_interface.getHandle("m1n6s200_joint_6"), &cmd[5]);
        jnt_vel_interface.registerHandle(vel_handle_f);

        registerInterface(&jnt_vel_interface);

        // // connect and register force/torque sensor interface
        // hardware_interface::ForceTorqueSensorHandle ft_handle("ft_sensor", "id", frc, trq);
        // ft_sensor_interface.registerHandle(ft_handle);

        // registerInterface(&ft_sensor_interface);

        // // connect and register torque sensors array interface
        // hardware_interface::ArrayTorqueSensorsHandle trq_arr_handle("arr_trq_sensors", "id_arr", trq_arr);
        // trq_arr_interface.registerHandle(trq_arr_handle);

        // registerInterface(&trq_arr_interface);

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

        // MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) dlsym(commandLayer_handle, "RunGravityZEstimationSequence");
        // MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
        // MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
        // MySendAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SendAngularTorqueCommand");
        // MySetGravityVector = (int(*)(float Command[3])) dlsym(commandLayer_handle, "SetGravityVector");
        // MySetGravityPayload = (int(*)(float Command[GRAVITY_PAYLOAD_SIZE])) dlsym(commandLayer_handle, "SetGravityPayload");
        // MySetGravityOptimalZParam = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
        // MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
        // MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
        // MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
        // MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
        // MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
        // MyGetAngularTorqueCommand = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "GetAngularTorqueCommand");
        // MyGetAngularCurrentMotor = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCurrentMotor");
        // MyGetAngularCurrent = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCurrent");
        // MyGetAngularTorqueGravityEstimation = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "GetAngularTorqueGravityEstimation");
    }

    bool checkInit()
    {
        //Check if API was loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularPosition == NULL) || (MyGetAngularForce == NULL) || (MyGetAngularForceGravityFree == NULL) || (MyGetAngularVelocity == NULL) || (MySendAdvanceTrajectory == NULL) || (MyGetCartesianForce == NULL))
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
        else
        {
            std::cout << "API initialization worked" << std::endl;
            std::cout << "The robot will swich to torque control mode and move. Be cautious." << std::endl;

            //Initializing the point.
            trajectoryPoint.Limitations.accelerationParameter1 = 0.0f; //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.accelerationParameter2 = 0.0f; //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.accelerationParameter3 = 0.0f; //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.forceParameter1 = 0.0f;        //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.forceParameter2 = 0.0f;        //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.forceParameter3 = 0.0f;        //Not implemented yet but will be in a future release.
            trajectoryPoint.Limitations.speedParameter1 = 15.0f;       //We limit the translation velocity to 8 cm per second.
            trajectoryPoint.Limitations.speedParameter2 = 15.0f;       //We limit the orientation velocity to 0.6 RAD per second
            trajectoryPoint.Limitations.speedParameter3 = 0.08f;
            trajectoryPoint.LimitationsActive = 1;
            trajectoryPoint.Position.Type = ANGULAR_VELOCITY;
            //Since it is a cartesian position trajectory point those values will not be used but we initialize them anyway. :)
            trajectoryPoint.Position.Actuators.Actuator1 = 0.0f; //0.0f;
            trajectoryPoint.Position.Actuators.Actuator2 = 0.0f; //180.0f;
            trajectoryPoint.Position.Actuators.Actuator3 = 0.0f; //180.0f;
            trajectoryPoint.Position.Actuators.Actuator4 = 0.0f; //0.0f;
            trajectoryPoint.Position.Actuators.Actuator5 = 0.0f; //0.0f;
            trajectoryPoint.Position.Actuators.Actuator6 = 8.0f; //0.0f;

            //    // Set to position mode
            //    MySwitchTrajectoryTorque(POSITION);
            //    // Set the torque control type to Direct Torque Control
            //    MySetTorqueControlType(DIRECTTORQUE);
            //    // Set the safety factor to 0.5
            //    MySetTorqueSafetyFactor(0.95);//0.95
            //    // Set the vibration controller to 0.5
            //    MySetTorqueVibrationController(0.8);//0.8
            //    // Make sure we use the gravity manual input type
            //    MySetGravityType(MANUAL_INPUT);
            //    // Gravity vector in -X
            //    float GravityVector[3];
            //    GravityVector[0] = 0.0;
            //    GravityVector[1] = 0.0;
            //    GravityVector[2] = -9.81;//-9.81;
            //    // Set the gravity vector
            //    MySetGravityVector(GravityVector);
            //    // Switch to torque control
            //    // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)

            //    MySwitchTrajectoryTorque(TORQUE);
            //   //  for(int i=0;i<1000;i++)
            //   //  {
            //   //    ROS_WARN("Trying to switch to torque mode...");
            //   //    MySwitchTrajectoryTorque(TORQUE);
            //   //    usleep(5000);
            //   //  }
            //    ROS_INFO("Torque mode established");

            //    cmd[0]=0.0;
            //    cmd[1]=0.0;
            //    cmd[2]=0.0;
            //    cmd[3]=0.0;
            //    cmd[4]=0.0;
            //    cmd[5]=0.0;
            return true;
        }
    }

    void read()
    {
        //std::cout<<"I'm reading"<<std::endl;

        //read in joint position
        result = (*MyGetAngularPosition)(data);
        if (result == NO_ERROR_KINOVA)
        {
            pos[0] = (3.1415 / 180) * data.Actuators.Actuator1;
            pos[1] = (3.1415 / 180) * data.Actuators.Actuator2;
            pos[2] = (3.1415 / 180) * data.Actuators.Actuator3;
            pos[3] = (3.1415 / 180) * data.Actuators.Actuator4;
            pos[4] = (3.1415 / 180) * data.Actuators.Actuator5;
            pos[5] = (3.1415 / 180) * data.Actuators.Actuator6;
        }
        else
        {
            std::cout << "Error code = " << result << std::endl;
        }
        //read in joint velocity
        result = (*MyGetAngularVelocity)(data);
        if (result == NO_ERROR_KINOVA)
        {
            vel[0] = (3.1415 / 180) * data.Actuators.Actuator1;
            vel[1] = (3.1415 / 180) * data.Actuators.Actuator2;
            vel[2] = (3.1415 / 180) * data.Actuators.Actuator3;
            vel[3] = (3.1415 / 180) * data.Actuators.Actuator4;
            vel[4] = (3.1415 / 180) * data.Actuators.Actuator5;
            vel[5] = (3.1415 / 180) * data.Actuators.Actuator6;
        }
        else
        {
            std::cout << "Error code = " << result << std::endl;
        }
        //read in joint torque sensors
        result = (MyGetAngularForce)(data);

        //read in joint torque sensors (gravity free)
        //result = (MyGetAngularForceGravityFree)(data);

        //read in joint motor current
        //result = (*MyGetAngularCurrentMotor)(data);

        //read in joint actuator current
        //result = (*MyGetAngularCurrent)(data);

        /*//Get commanded torque in effort data
        float Result[COMMAND_SIZE];
        // Get the angular torque command
        MyGetAngularTorqueCommand(Result);
	    */
        /*
        //Get gravity estimation in effort data
        float Result[COMMAND_SIZE];
        MyGetAngularTorqueGravityEstimation(Result);
        */
        if (result == NO_ERROR_KINOVA)
        {
            eff[0] = data.Actuators.Actuator1;
            eff[1] = data.Actuators.Actuator2;
            eff[2] = data.Actuators.Actuator3;
            eff[3] = data.Actuators.Actuator4;
            eff[4] = data.Actuators.Actuator5;
            eff[5] = data.Actuators.Actuator6;

            /*
    	  //Get commanded torque/grv estimation in effort data
    	  eff[0]=Result[0];
          eff[1]=Result[1];
          eff[2]=Result[2];
          eff[3]=Result[3];
          eff[4]=Result[4];
          eff[5]=Result[5];

          //for(int i=0;i<6;i++) std::cout << "RESULT = " << Result[i] << std::endl;
		  */
        }
        else
        {
            std::cout << "Error code = " << result << std::endl;
        }

        //read in force torque sensor measurements
        // result = (*MyGetCartesianForce)(dataC);
        // if (result == NO_ERROR_KINOVA)
        // {
        //     frc[0] = dataC.Coordinates.X;
        //     frc[1] = dataC.Coordinates.Y;
        //     frc[2] = dataC.Coordinates.Z;
        //     trq[0] = dataC.Coordinates.ThetaX;
        //     trq[1] = dataC.Coordinates.ThetaY;
        //     trq[2] = dataC.Coordinates.ThetaZ;
        // }
        // else
        // {
        //     std::cout << "Error code = " << result << std::endl;
        // }

        // //read in joint torque sensors (gravity free)
        // result = (MyGetAngularForceGravityFree)(data);
        // if (result == NO_ERROR_KINOVA)
        // {
        //     trq_arr[0] = data.Actuators.Actuator1;
        //     trq_arr[1] = data.Actuators.Actuator2;
        //     trq_arr[2] = data.Actuators.Actuator3;
        //     trq_arr[3] = data.Actuators.Actuator4;
        //     trq_arr[4] = data.Actuators.Actuator5;
        //     trq_arr[5] = data.Actuators.Actuator6;
        // }
        // else
        // {
        //     std::cout << "Error code = " << result << std::endl;
        // }
    }

    void write()
    {
        //std::cout<<"I'm writing"<<std::endl;
        //std::cout<<"Printing result of torque command : "<<result<<std::endl;

        trajectoryPoint.Position.Actuators.Actuator1 = 0.0f;  //0.0f;
        trajectoryPoint.Position.Actuators.Actuator2 = 0.0f;  //180.0f;
        trajectoryPoint.Position.Actuators.Actuator3 = 0.0f;  //180.0f;
        trajectoryPoint.Position.Actuators.Actuator4 = 0.0f;  //0.0f;
        trajectoryPoint.Position.Actuators.Actuator5 = 0.0f;  //0.0f;
        trajectoryPoint.Position.Actuators.Actuator6 = 10.0f; //0.0f;

        result = MySendAdvanceTrajectory(trajectoryPoint);
    }

    void close()
    {
        std::cout << std::endl
                  << "Switching to Position control mode" << std::endl;
        // MySwitchTrajectoryTorque(POSITION);

        std::cout << std::endl
                  << "Closing Robot" << std::endl;
        std::cout << std::endl
                  << "Calling the method CloseAPI()" << std::endl;
        result = (*MyCloseAPI)();
        std::cout << "result of CloseAPI() = " << result << std::endl;
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    // hardware_interface::ForceTorqueSensorInterface ft_sensor_interface;
    // hardware_interface::ArrayTorqueSensorsInterface trq_arr_interface;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];
    float VelCommand[COMMAND_SIZE];
    double frc[3];
    double trq[3];
    double trq_arr[6];

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
    //   int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
    //   int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    //   int(*MySetTorqueSafetyFactor)(float factor);
    //   int(*MySendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
    //   int(*MySetGravityVector)(float Command[3]);
    //   int(*MySetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
    //   int(*MySetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
    //   int(*MySetGravityType)(GRAVITY_TYPE Type);
    //   int(*MySetTorqueVibrationController)(float value);
    //   int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    //   int(*MyGetAngularTorqueCommand)(float Command[COMMAND_SIZE]);
    //   int (*MyGetAngularCurrentMotor)(AngularPosition &);
    //   int (*MyGetAngularCurrent)(AngularPosition &);
    //   int(*MyGetAngularTorqueGravityEstimation)(float Command[COMMAND_SIZE]);
};

void mySigintHandler(int sig)
{
    ROS_WARN("SIGINT caught!!!");
    g_sigint_request = 1;
}

#define RATE 100.0 //ka++

int main(int argc, char **argv)
{
    ros::init(argc, argv, "KinovaMico", ros::init_options::NoSigintHandler);
    //ros::init(argc, argv, "MyRobot");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);

    ros::CallbackQueue my_callback_queue;    //ka++
    nh.setCallbackQueue(&my_callback_queue); //ka++

    KinovaMico robot;
    //initialize control manager
    controller_manager::ControllerManager cm(&robot, nh);

    if (robot.checkInit() == false)
    {
        ROS_ERROR("Robot Initialization Error.");
        return 0;
    }

    ros::AsyncSpinner spinner(0, &my_callback_queue);
    spinner.start();
    //ros::waitForShutdown();//ka++

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(RATE);

    while (!g_sigint_request)
    {
        //time loop starts
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        prev_time = time; //ka++

        // read robot data
        robot.read();

        // update data
        cm.update(time, period);

        //send data to the robot
        robot.write();

        //ros::spinOnce();

        rate.sleep();
    }
    robot.close();
    spinner.stop(); //ka++
                    //kapoio try catch pou na gurnaei se position control
    return 0;
}
