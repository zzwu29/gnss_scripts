#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>

using namespace gnss_comm;

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath, const std::string &raw_gnss_topic)
{
    std::vector<std::vector<ObsPtr>> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(raw_gnss_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m : view)
    {
        GnssMeasMsgConstPtr obs_msg = m.instantiate<GnssMeasMsg>();
        if (obs_msg == NULL)
            continue;
        std::vector<ObsPtr> obs = msg2meas(obs_msg);
        result.push_back(obs);
    }
    return result;
}

int main(int argc, char **argv)
{
    if(argc!=3 && argc!=4)
    {
        std::cerr<<"----- Error: Paramter must be 2 (bag_input and rinexo_output) or 3 (bag_input, rinexo_output and gnss_comm/GnssMeasMsg) ! -----"<<std::endl;
        return 1;
    }
    std::string bag_file(argv[1]), output_file(argv[2]);
    std::string raw_gnss_topic("/ublox_driver/range_meas");

    if (argc == 4)
    {
        raw_gnss_topic=std::string(argv[3]);
    }

    std::cerr<<"Input bag: "<<argv[1]<<", gnss_comm/GnssMeasMsg topic: "<<raw_gnss_topic<<std::endl;
    std::cerr<<"Start read from bag!"<<std::endl;
    std::vector<std::vector<ObsPtr>> all_gnss_meas = parse_gnss_meas(bag_file,raw_gnss_topic);

    std::cerr<<"Output rinex obs file: "<<output_file<<std::endl;
    std::cerr<<"Start output to rinex obs file !"<<std::endl;
    obs2rinex(output_file, all_gnss_meas);
    std::cout << "Done\n";
    return 0;
}