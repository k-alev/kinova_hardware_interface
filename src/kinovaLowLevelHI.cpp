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
#include <kinova_hardware_interface/jaco2_rs485.h>
#include <signal.h>
#include <thread>
#include <mutex>

#define OPTIMAL_Z_PARAM_SIZE 16
#define COMMAND_SIZE 6
#define GRAVITY_PAYLOAD_SIZE 4
#define GRAVITY_PARAM_SIZE 42

sig_atomic_t volatile g_sigint_request = 0;
//should be removed
int clb_flag;
int traj_flag;
std::mutex m, m2;

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
    hardware_interface::JointHandle tor_handle_a(jnt_state_interface.getHandle("m1n6s200_joint_1"), &cmd[0]);
    jnt_tor_interface.registerHandle(tor_handle_a);

    hardware_interface::JointHandle tor_handle_b(jnt_state_interface.getHandle("m1n6s200_joint_2"), &cmd[1]);
    jnt_tor_interface.registerHandle(tor_handle_b);

    hardware_interface::JointHandle tor_handle_c(jnt_state_interface.getHandle("m1n6s200_joint_3"), &cmd[2]);
    jnt_tor_interface.registerHandle(tor_handle_c);

    hardware_interface::JointHandle tor_handle_d(jnt_state_interface.getHandle("m1n6s200_joint_4"), &cmd[3]);
    jnt_tor_interface.registerHandle(tor_handle_d);

    hardware_interface::JointHandle tor_handle_e(jnt_state_interface.getHandle("m1n6s200_joint_5"), &cmd[4]);
    jnt_tor_interface.registerHandle(tor_handle_e);

    hardware_interface::JointHandle tor_handle_f(jnt_state_interface.getHandle("m1n6s200_joint_6"), &cmd[5]);
    jnt_tor_interface.registerHandle(tor_handle_f);

    registerInterface(&jnt_tor_interface);

    // connect and register force/torque sensor interface
    hardware_interface::ForceTorqueSensorHandle ft_handle("ft_sensor", "id", frc, trq);
    ft_sensor_interface.registerHandle(ft_handle);

    registerInterface(&ft_sensor_interface);

    // connect and register torque sensors array interface
    hardware_interface::ArrayTorqueSensorsHandle trq_arr_handle("arr_trq_sensors", "id_arr", trq_arr);
    trq_arr_interface.registerHandle(trq_arr_handle);

    registerInterface(&trq_arr_interface);
  }

  bool checkInit()
  {
    std::cout << "Connecting.." << std::endl;
    arm.Connect();

    std::cout << "Initializing Position Mode" << std::endl;
    arm.InitPositionMode();

    std::cout << "Initializing Force Mode" << std::endl;
    arm.InitForceMode();

    return true;
  }

  void read()
  {
    //std::cout<<"I'm reading"<<std::endl;
    arm.ReadForces();
    for (int i = 0; i < COMMAND_SIZE; i++)
    {
      pos[i] = (3.1415 / 180) * arm.pos[i];
      vel[i] = (3.1415 / 180) * arm.vel[i];
      eff[i] = arm.torque_load[i];
    }
    //should be removed
    std::lock_guard<std::mutex> lock(m2);
    trq_arr[0] = (double)traj_flag;
  }

  void write()
  {
    //std::cout<<"I'm writing"<<std::endl;

    for (int i = 0; i < COMMAND_SIZE; i++)
    {
      TorqueCommand[i] = cmd[i];
    }
    arm.WriteForces(TorqueCommand);
    //std::cout<<"Printing result of torque command : "<<result<<std::endl;
  }

  void close()
  {

    cout << "Initializing Position Mode" << endl;
    arm.InitPositionMode();
    cout << "Disconnecting..." << endl;
    arm.Disconnect();
    std::cout << std::endl
              << "Robot Disconnected" << std::endl;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_tor_interface;
  hardware_interface::ForceTorqueSensorInterface ft_sensor_interface;
  hardware_interface::ArrayTorqueSensorsInterface trq_arr_interface;
  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
  float TorqueCommand[COMMAND_SIZE];
  double frc[3];
  double trq[3];
  double trq_arr[6];

  //Instatiate Jaco2_rs485
  Jaco2 arm = Jaco2(3);
};

void _SigintHandler(int sig)
{
  ROS_WARN("SIGINT caught!!!");
  g_sigint_request = 1;
}

//should be removed
void kbrd_callback()
{
  //signal(SIGINT, _SigintHandler_callback);
  char my_char;

  m.lock();
  clb_flag = 0;
  m.unlock();

  while (!clb_flag)
  {
    std::cin >> my_char;
    if (my_char == 'b')
    {
      std::cout << "Gotcha!\n";
      std::lock_guard<std::mutex> lock(m2);
      traj_flag = 1;
    }
    else if (my_char == 'm')
    {
      std::cout << "Gotcha!\n";
      std::lock_guard<std::mutex> lock(m2);
      traj_flag = 0;
    }
    else
    {
      std::cout << "Wrong letter!\n";
    }
    usleep(2000);
  }
  ROS_INFO("Thread returned");
}

#define RATE 300.0

int main(int argc, char **argv)
{
  //should be removed
  std::thread t(&kbrd_callback);

  ros::init(argc, argv, "KinovaMico", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  signal(SIGINT, _SigintHandler);

  ros::CallbackQueue my_callback_queue;
  nh.setCallbackQueue(&my_callback_queue);

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

  //should be removed
  m.lock();
  clb_flag = 1;
  m.unlock();
  //should be removed
  t.join();
  ROS_INFO("Thread joint");

  spinner.stop(); //ka++

  //kapoio try catch pou na gurnaei se position control
  return 0;
}
