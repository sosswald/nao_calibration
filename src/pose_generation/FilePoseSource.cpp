#include <pose_generation/FilePoseSource.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

namespace kinematic_calibration
{


FilePoseSource::FilePoseSource(const std::vector<string> &filepaths, std::string projectdirectory) : MeasurementMsgPoseSource(false)
{
    ROS_INFO("New FilePoseSource generated");
    this->addPosesFromYaml(filepaths, projectdirectory);
    //this->addPosesFromBagFile(filepaths);
}

void FilePoseSource::addPosesFromYaml(const std::vector<std::string> & filenames,std::string projectdirectory)
{
  for(unsigned int i =0; i < filenames.size(); ++i)
     addPosesFromYaml(filenames[i],projectdirectory);
}

void FilePoseSource::addPosesFromYaml(const std::string & filename,std::string projectdirectory)
{
   cout << "Reading from " << filename << endl;
   YAML::Node doc = YAML::LoadFile(projectdirectory+filename);
   cout << "Parsing " << doc.size() << " elements"<<endl;
   for(YAML::const_iterator it=doc.begin();it != doc.end();++it) {
      std::string key = it->first.as<std::string>();       // <- key
      sensor_msgs::JointState js;
      cout << "key is " << key <<" ";
      if (it->second["joint_names"].size() != it->second["positions"].size())
      {
         cerr << "joint_names and positions have different size for key " << key << endl;
         throw std::runtime_error("error parsing yaml file");
      }
      YAML::Node child = it->second["joint_names"];
      for (std::size_t i=0;i<child.size();i++) {
	
         js.name.push_back(child[i].as<string>());
      }
      child = it->second["positions"];
      for (std::size_t i=0;i<child.size();i++) {
		//std::cout<<child[i].as<double>()<< " ";
         js.position.push_back(child[i].as<double>());
      }
      string chainName = key.substr(0,4); // here is for leg
      cout << chainName <<" ";
      string stampStr="";
      string search="leg";
      if (key.find(search)!=std::string::npos)
      	 stampStr = key.substr(4); // here is for leg
      else  
	stampStr = key.substr(8);
     // cout << stampStr<<"\n";

      js.header.stamp.fromSec( boost::lexical_cast<int>(stampStr) );
     cout << js.header.stamp<<std::endl;
      this->poses[chainName].addPose(key, js);
      
      this->ids[js.header.stamp] = key;
   }

}

void FilePoseSource::addPosesFromBagFile(const std::vector<std::string> & filenames)
{
  string msg_topic = "/kinematic_calibration/measurement_data";
  //string cam_info_topic = "/nao_camera/camera_info";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(msg_topic);
  //topics.push_back(cam_info_topic);

  for(unsigned int i =0; i < filenames.size(); ++i)
  {
    cout << "Reading from " << filenames[i] << endl;
    // Load bag
    rosbag::Bag bag;
    bag.open(filenames[i], rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == msg_topic || ("/" + m.getTopic() == msg_topic))
      {
        kinematic_calibration::measurementData::ConstPtr meas_msg = m.instantiate<kinematic_calibration::measurementData>();
        if (meas_msg != NULL)
        {
            this->poses[meas_msg->chain_name].addPose(meas_msg->id, meas_msg->jointState);
            this->ids[meas_msg->jointState.header.stamp] = meas_msg->id;
        }
      }
      /*
      if (m.getTopic() == cam_info_topic || ("/" + m.getTopic() == cam_info_topic))
      {
        sensor_msgs::CameraInfo::ConstPtr info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (info_msg != NULL)
          node.camerainfoCallback(info_msg);
      }
      */
    }
    bag.close();
  }
  //this->splitPoseSets(this->numOfPartitionsPerChain);
}

FilePoseSource::~FilePoseSource()
{

}

void FilePoseSource::getPoses(const KinematicChain &kinematicChain, vector<MeasurementPose> &poses)
{
    string chainName = kinematicChain.getName();
    if(this->poses.count(chainName) < 1)
        return;

    PoseSourcePool pool = this->poses[chainName];
    for (PoseSourcePool::const_iterator idPoseIt = pool.begin(); idPoseIt != pool.end(); idPoseIt++) {
        poses.push_back(MeasurementPose(kinematicChain, idPoseIt->second, idPoseIt->first));
    }
}

}
