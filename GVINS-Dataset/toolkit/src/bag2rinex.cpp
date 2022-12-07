#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>

using namespace gnss_comm;

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath)
{
    std::vector<std::vector<ObsPtr>> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/range_meas");
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
    if(argc!=3)
    {
        std::cerr<<"----- Error: Paramter must be 2 (bag_input and rinexo_output) ! -----"<<std::endl;
        return 1;
    }

    std::cerr<<"Input bag: "<<argv[1]<<std::endl;
    std::cerr<<"Start read from bag!"<<std::endl;
    std::vector<std::vector<ObsPtr>> all_gnss_meas = parse_gnss_meas(argv[1]);

    std::cerr<<"Output rinex obs file: "<<argv[2]<<std::endl;
    std::cerr<<"Start output to rinex obs file !"<<std::endl;
    obs2rinex(argv[2], all_gnss_meas);
    std::cout << "Done\n";
    return 0;
}