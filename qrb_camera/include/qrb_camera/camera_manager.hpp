#ifndef QRB_CAMERA__CAMERA_MANAGER_HPP_
#define QRB_CAMERA__CAMERA_MANAGER_HPP_

#include <mutex>

#include "qrb_camera/camera_interface.hpp"
#include "qrb_camera/qmmf_camera.hpp"
#include "qrb_camera/qrb_camera_common.hpp"

namespace qrb_camera
{
using ImageCallback = std::function<void(std::unique_ptr<CameraFrame> frame)>;

class CameraManager
{
public:
  ~CameraManager();

  int create_camera(CameraType type, uint32_t camera_id);
  bool set_camera_parameter(int index, CameraConfigure & param);

  bool start_camera(int index);
  void stop_camera(int index);

  bool register_callback(int index, ImageCallback image_cb);

private:
  void convert_detail_to_param(int index, std::vector<CameraParam> & param);

  std::mutex mtx_;
  std::vector<ICameraDevice *> devices_;

  std::vector<std::vector<CameraConfigure>> params_;
  std::vector<std::vector<ICameraListener *>> listeners_;

  std::vector<CameraType> types_;

  std::string logger_ = "CameraManager";
};
}  // namespace qrb_camera
#endif