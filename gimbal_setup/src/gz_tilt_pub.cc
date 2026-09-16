// gz_tilt_pub - PERMANENT gimbal tilt command broadcaster (gimbal branch, Phase O) WHY IT EXISTS: `gz
// topic -p` establishes a new transport connection and pays ~1 s on each broadcast. It was sufficient
// for steady/slow tilt of Phase A/B; In Phase O, the tilt follows the target elevation and changes to
// eps = asin(down/r) in the terminal ~40-70 deg/s in the last seconds -- one broadcast per second is
// not enough. USE: gz_tilt_pub <model_name> (e.g. gz_tilt_pub iris-1) reads angle (radians, decimal)
// line by line from stdin and IMMEDIATELY publishes it as GzString to topic ~/<model>/gimbal_tilt_cmd.
// When the connection is established, it writes "READY" to stderr (python side expects this). It exits
// clean when stdin is closed. /
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: gz_tilt_pub <model_name>\n";
    return 2;
  }
  gazebo::client::setup(argc, argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  const std::string topic = std::string("~/") + argv[1] + "/gimbal_tilt_cmd";
  gazebo::transport::PublisherPtr pub =
      node->Advertise<gazebo::msgs::GzString>(topic);
  pub->WaitForConnection();
  std::cerr << "READY" << std::endl;   // handshake: Python waits for this line

  std::string line;
  while (std::getline(std::cin, line))
  {
    if (line.empty())
      continue;
    gazebo::msgs::GzString m;
    m.set_data(line);
    pub->Publish(m);
  }

  gazebo::client::shutdown();
  return 0;
}
