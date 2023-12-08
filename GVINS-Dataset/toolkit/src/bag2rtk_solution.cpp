#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>

using namespace gnss_comm;

struct RTKState
{
    unsigned long gnss_ts_ns;
    Eigen::Vector3d t_ecef;
    Eigen::Vector3d v_enu;
    uint8_t fix_type;
    bool valid_fix;
    bool diff_soln;
    uint8_t carr_soln;
};
typedef std::shared_ptr<RTKState> RTKStatePtr;

void extract_rtk_states(const std::string &bag_filepath, std::vector<RTKStatePtr> &all_states, const std::string &rtk_sol_topic)
{
    all_states.clear();

    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(rtk_sol_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for(rosbag::MessageInstance const m : view)
    {
        GnssPVTSolnMsgConstPtr pvt_msg = m.instantiate<GnssPVTSolnMsg>();
        if (pvt_msg != NULL)
        {
            PVTSolutionPtr pvt = msg2pvt(pvt_msg);
            RTKStatePtr rtk_state(new RTKState());
            rtk_state->gnss_ts_ns = static_cast<unsigned long>(time2sec(pvt->time)*1e9);
            Eigen::Vector3d pvt_lla(pvt->lat, pvt->lon, pvt->hgt);
            rtk_state->t_ecef = geo2ecef(pvt_lla);
            rtk_state->v_enu.x() =  pvt->vel_e;
            rtk_state->v_enu.y() =  pvt->vel_n;
            rtk_state->v_enu.z() = -pvt->vel_d;
            rtk_state->fix_type = pvt->fix_type;
            rtk_state->valid_fix = pvt->valid_fix;
            rtk_state->diff_soln = pvt->diff_soln;
            rtk_state->carr_soln = pvt->carr_soln;
            all_states.push_back(rtk_state);
        }
    }
}

void save_states(const std::vector<RTKStatePtr> &seq, const std::string &filepath)
{
    FILE *fp = fopen(filepath.c_str(), "w");
    for (uint32_t i = 0; i < seq.size(); ++i)
    {
        const RTKStatePtr &state = seq[i];
        /* global_timestamp, ecef_px, ecef_py, ecef_pz, enu_vx, enu_vy, enu_vz,
         * solution_fix_type, is_fix_valid, is_differential_correction_applied, 
         * carrier_phase_range_solution_status
         * Check gnss_comm/msg/GnssPVTSolnMsg.msg for details.
         */ 
        fprintf(fp, "%lu", state->gnss_ts_ns);
        for (uint32_t j = 0; j < 3; ++j)
            fprintf(fp, ", %.5f", state->t_ecef(j));
        for (uint32_t j = 0; j < 3; ++j)
            fprintf(fp, ", %.5f", state->v_enu(j));
        fprintf(fp, ", %d, %d, %d, %d\n", static_cast<int>(state->fix_type), 
            static_cast<int>(state->valid_fix), static_cast<int>(state->diff_soln), 
            static_cast<int>(state->carr_soln));
    }
    fclose(fp);
}

int main(int argc, char **argv)
{
    if(argc!=3 && argc!=4)
    {
        std::cerr<<"----- Error: Paramter must be 2 (bag_input and rtk_output) or 3 (bag_input, rtk_output and gnss_comm/GnssPVTSolnMsg)! -----"<<std::endl;
        return 1;
    }
    std::string bag_file(argv[1]), output_file(argv[2]);
    std::string rtk_sol_topic("/ublox_driver/receiver_pvt");

    if (argc == 4)
    {
        rtk_sol_topic=std::string(argv[3]);
    }

    std::cerr<<"Input bag: "<<bag_file<<", gnss_comm/GnssPVTSolnMsg topic: "<<rtk_sol_topic<<std::endl;
    std::cerr<<"Start read from bag!"<<std::endl;
    // load RTK states
    std::vector<RTKStatePtr> all_states;
    extract_rtk_states(bag_file, all_states, rtk_sol_topic);

    std::cerr<<"Output rinex obs file: "<<output_file<<std::endl;
    std::cerr<<"Start output to rinex obs file !"<<std::endl;
    // write to file
    save_states(all_states, output_file);

    std::cout << "Done.\n";
    return 0;
}