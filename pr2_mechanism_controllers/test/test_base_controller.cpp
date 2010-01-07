
#include <pr2_mechanism_controllers/base_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <urdf/parser.h>
#include <ros/node.h>


int main( int argc, char** argv )
{
  if(argc != 3)
  {
    printf("Usage: %s <robot_xml> <controller_xml>\n",argv[0]);
    exit(-1);
  }
  printf("robot file:: %s, controller file:: %s\n",argv[1],argv[2]);


  /*********** Create the robot model ****************/
  pr2_mechanism::Robot *robot_model = new pr2_mechanism::Robot;
  controller::BaseControllerNode bc;
  HardwareInterface hw(0);
  robot_model->hw_ = &hw;


  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);

  /*********** Load the robot model and state file ************/
  char *xml_robot_file = argv[1];
  TiXmlDocument xml(xml_robot_file);   // Load robot description
  xml.LoadFile();
  TiXmlElement *root = xml.FirstChildElement("robot");
  urdf::normalizeXml(root);
  robot_model->initXml(root);
  pr2_mechanism_model::RobotState *robot_state = new pr2_mechanism_model::RobotState(robot_model, &hw);


  /*********** Load the controller file ************/
  #error Broken because the base controller does not have initXml anymore
  char *xml_control_file = argv[2];
  TiXmlDocument xml_control(xml_control_file);   // Load robot description
  xml_control.LoadFile();
  TiXmlElement *root_control = xml_control.FirstChildElement("controllers");
  TiXmlElement *root_controller = root_control->FirstChildElement("controller");
  //bc.initXml(robot_state,root_controller);


  /************ Testing the odometry calculations themselves ******************/
/*  NEWMAT::Matrix A(16,3);

  A.Row(1) << 10 << 8.95 << 0.05;
  A.Row(2) << 0 <<  -2 << 0;
  A.Row(3) << 1 << -0.1 << 0.01;
  A.Row(4) << 2 << 1.1 << 0;

  A.Row(5) << 3 << 2 << -0.05;
  A.Row(6) << 4 << 3 << 0.01;
  A.Row(7) << 5 << 4.1 << 0.05;
  A.Row(8) << -1 << -2 << 0.025;

  A.Row(9) << 6.15 << 5.05 << 0.01;
  A.Row(10) << 6.985 << 6.02 << 0.01;
  A.Row(11) << 8.01 << 8.05 << -0.05;
  A.Row(12) << 9.03 << 8.1 << -0.01;

  A.Row(13) << -8.03 << -9.1 << 0.01;
  A.Row(14) << -10.03 << -13.1 << 0.05;
  A.Row(15) << -15.03 << -16.1 << -0.015;
  A.Row(16) << -16.03 << -17.1 << -0.01;

  NEWMAT::Matrix B(16,1);
  B << 1.1 << 1 << 1.1 << 1.15 << 0.95 << 0.99 << 0.98 << 0.95 << 1.05 << 1.1 << 1.05 << 1 << 1.13 << 0.995 << 1.035 << 1.08;
  NEWMAT::Matrix xfit(3,1);

  xfit = bc.c_->iterativeLeastSquares(A,B,"Gaussian",10);
  cout << "done" << xfit << endl;
*/

  delete robot_model;
  delete robot_state;
}






/*class BaseControllerTest : public testing::Test {
 protected:
  virtual void SetUp() {
  }

  // virtual void TearDown() {}

  Queue<int> q0_;
  Queue<int> q1_;
  Queue<int> q2_;
};


TEST (BaseControllerTests, constructionDestruction)
{
  controller::BaseController *bc = new controller::BaseController();
  delete bc;
}

TEST (BaseControllerTests, loadXML)
{
  pr2_mechanism::Robot *robot = new pr2_mechanism::Robot("r2d2");
  controller::BaseController bc;

  const string xml_controller_file = "controller_base.xml";
  const string xml_robot_file = "pr2_base.xml"

  TiXmlDocument xml(xml_robot_file);   // Load robot description
  xml.LoadFile();
  TiXmlElement *root = xml.FirstChildElement("robot");
  robot->initXml(root);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
*/
