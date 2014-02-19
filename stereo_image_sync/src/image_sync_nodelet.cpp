#include "stereo_image_sync/stereo_image_sync.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace stereo_image_sync{

class StereoImageSyncNodelet : public nodelet::Nodelet
{

public:
  StereoImageSyncNodelet(){}

private:
  void onInit(){
    sync_.reset(new stereo_image_sync::StereoImageSync(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }
  boost::shared_ptr<stereo_image_sync::StereoImageSync> sync_;

};

}

PLUGINLIB_DECLARE_CLASS(stereo_image_sync, StereoImageSyncNodelet, stereo_image_sync::StereoImageSyncNodelet, nodelet::Nodelet);
