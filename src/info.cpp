#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

int samples = 10;

void laser_callback(sensor_msgs::LaserScan  ls){
    static int cnt = 0;
    static uint64_t begin = ros::Time::now().toNSec();

    if (cnt < samples){
        cnt = cnt + 1;
        fprintf(stderr, "Listening %d/%d\r",cnt, samples);
        return;
    }
    uint64_t end = ros::Time::now().toNSec();

    double period_ms = double(end-begin)/double(cnt)/1e6;

    fprintf(stdout, "\nFrame:                 %s\n", ls.header.frame_id.c_str());
    fprintf(stdout, "Points:                %d\n", (int) ls.ranges.size());
    fprintf(stdout, "Measurements (deg):    %f -> %f\n", ls.angle_min*180.0f/M_PI, ls.angle_max*180.0f/M_PI);
    fprintf(stdout, "Angle increment (deg): %f\n", ls.angle_increment*180.0f/M_PI);
    fprintf(stdout, "Range (m):             %.2f -> %.2f\n", ls.range_min, ls.range_max);
    fprintf(stdout, "Intensities:           %s\n", ls.intensities.size()>0?"Yes" : "No");
    fprintf(stdout, "Period (ms):           %.2f\n", period_ms);
    fprintf(stdout, "Rate (Hz):             %.2f\n", 1000/period_ms);

    ros::shutdown();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "laser_shaper_info");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    if (argc > 1){
        samples = atoi(argv[1]);
    }

    ros::Subscriber sub = n.subscribe("/scan", 1, laser_callback);
    ros::spin();
    return 0;
}