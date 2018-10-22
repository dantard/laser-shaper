#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

bool remove_intensity = false, angle_max_set = false, angle_min_set = false;
float angle_min = 0.0;
float angle_max = 0.0;
float range_max = 0.0;
float range_max_value_diff = 0.0;
float noise = 0.0;
int spurious = 0;
int decimation = 1;
int period_ms = 0;
bool dotrunc = false, output_frame_set;
std::string output_frame;

void laser_callback(sensor_msgs::LaserScan ls){

    static uint64_t before_ms = ros::Time::now().toNSec()/(uint64_t)1e6;
    uint64_t now_ms = ros::Time::now().toNSec()/(uint64_t)1e6;
    if (now_ms - before_ms < period_ms){
        return;
    }
    before_ms = now_ms;

    angle_min = angle_min_set ? angle_min : ls.angle_min;
    angle_max = angle_max_set ? angle_max : ls.angle_max;

    ls.range_max = range_max != 0.0 ? range_max : ls.range_max;

    for (long count = ls.ranges.size() - 1; count >=0 ; count --){

        if (ls.ranges.at(count) >= ls.range_max || isnan(ls.ranges.at(count))){
            ls.ranges.at(count) = range_max_value_diff != 0.0 ? ls.range_max - range_max_value_diff : ls.range_max;
        }else if (dotrunc){
            ls.ranges.at(count) = float(round(ls.ranges.at(count)));
        }

        double err_percent = noise/100.0;
        ls.ranges.at(count) = ls.ranges.at(count) * (1 - (err_percent*0.5 + double(rand())/double(RAND_MAX)*err_percent));

        float ang = ls.angle_min + ls.angle_increment * float(count);

        if (ang < angle_min - 1e-6 || ang > angle_max  + 1e-6){
            ls.ranges.erase(ls.ranges.begin() + count);
            if (ls.intensities.size() > count){
                ls.intensities.erase(ls.intensities.begin() + count);
            }
        }else{
            if (count % decimation != 0){
                ls.ranges.erase(ls.ranges.begin() + count);
                if (ls.intensities.size() > count){
                    ls.intensities.erase(ls.intensities.begin() + count);
                }
            }
        }
    }
    ls.angle_min = angle_min;
    ls.angle_max = angle_min + ls.angle_increment*(ls.ranges.size() - 1);
    ls.angle_increment *= decimation;

    if (spurious != 0.0){
        for (unsigned int count = 1; count < ls.ranges.size() - 1; count ++){
            double d1 = ls.ranges.at(count+1) - ls.ranges.at(count);
            double d2 = ls.ranges.at(count) - ls.ranges.at(count-1);
            double isspurious = d1 > spurious && d2 < -spurious || d1 < -spurious && d2 > spurious;
            if (isspurious){
                ls.ranges.at(count) = (ls.ranges.at(count+1) + ls.ranges.at(count-1))/2.0f;
            }
        }
    }

    if (remove_intensity){
        ls.intensities.clear();
    }

    if (output_frame_set){
        ls.header.frame_id = output_frame;
    }

    pub.publish(ls);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_shaper");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  nh.getParam("remove_intensity", remove_intensity);

  angle_min_set = nh.getParam("angle_min", angle_min);
  angle_max_set = nh.getParam("angle_max", angle_max);

  nh.getParam("decimation", decimation);
  nh.getParam("range_max", range_max);

  /* Subtract this value to the range_max */
  nh.getParam("range_max_value_diff", range_max_value_diff);
  nh.getParam("period", period_ms);
  nh.getParam("trunc", dotrunc);
  nh.getParam("spurious", spurious);
  nh.getParam("noise", noise);

    output_frame_set = nh.getParam("output_frame", output_frame);

  angle_max = angle_max*float(M_PI)/180.0f;
  angle_min = angle_min*float(M_PI)/180.0f;

  ros::Subscriber sub = n.subscribe("/scan", 1000, laser_callback);
  pub = n.advertise<sensor_msgs::LaserScan>("/scan/shaped", 1);
  ros::spin();

  return 0;
}
