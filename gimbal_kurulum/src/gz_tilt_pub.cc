/*
 * gz_tilt_pub - KALICI gimbal tilt komut yayincisi (gimbal dali, Faz C)
 *
 * NEDEN VAR: `gz topic -p` her yayinda yeni transport baglantisi kurup
 * ~1 s oduyor. Faz A/B'nin sabit/yavas tilt'i icin yeterliydi; Faz C'de
 * tilt hedefin yukselisini izler ve terminalde eps = asin(down/r) son
 * saniyelerde ~40-70 deg/s degisir -- saniyede bir yayin yetmez.
 *
 * KULLANIM:  gz_tilt_pub <model_adi>        (or. gz_tilt_pub iris-1)
 *   stdin'den satir satir aci okur (radyan, ondalik sayi) ve
 *   ~/<model>/gimbal_tilt_cmd konusuna GzString olarak ANINDA yayinlar.
 *   Baglanti kurulunca stderr'e "HAZIR" yazar (python tarafi bunu bekler).
 *   stdin kapaninca temiz cikar.
 */
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "kullanim: gz_tilt_pub <model_adi>\n";
    return 2;
  }
  gazebo::client::setup(argc, argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  const std::string topic = std::string("~/") + argv[1] + "/gimbal_tilt_cmd";
  gazebo::transport::PublisherPtr pub =
      node->Advertise<gazebo::msgs::GzString>(topic);
  pub->WaitForConnection();
  std::cerr << "HAZIR" << std::endl;   // el sikisma: python bu satiri bekler

  std::string satir;
  while (std::getline(std::cin, satir))
  {
    if (satir.empty())
      continue;
    gazebo::msgs::GzString m;
    m.set_data(satir);
    pub->Publish(m);
  }

  gazebo::client::shutdown();
  return 0;
}
