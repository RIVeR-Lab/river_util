#include "stereo_image_sync/stereo_image_sync.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_image_sync");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  new stereo_image_sync::StereoImageSync(nh, pnh);
  ros::spin();

  return 0;
}
