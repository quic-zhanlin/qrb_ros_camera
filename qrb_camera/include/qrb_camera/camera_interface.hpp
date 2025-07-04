/*
 * Copyright (c) 2015, 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *​​​​ ​Changes from Qualcomm Innovation Center, Inc. are provided
 * under the following license: Copyright (c) 2023 Qualcomm Innovation Center,
 * Inc. All rights reserved. SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_CAMERA__CAMERA_INTERFACE_HPP_
#define QRB_CAMERA__CAMERA_INTERFACE_HPP_

#include <string>
#include <vector>

namespace qrb_camera
{
/**
 * Interface to an object that represents single image/metadata
 * frame
 */
class ICameraFrame
{
  ICameraFrame(const ICameraFrame &) = delete;
  void operator=(const ICameraFrame &) = delete;

protected:
  uint32_t refs_;
  ICameraFrame() : refs_(0), timestamp(0), data(nullptr), fd(0) {}
  virtual ~ICameraFrame() {}

public:
  /**
   * aquire a reference to the frame, this is required if client
   * wants to hold the frame for further processing after camera
   * callback returns
   *
   * @return uint32_t : number of refs to the frame.
   */
  virtual uint32_t acquire_ref() = 0;
  /**
   * release reference to the frame object. This will release
   * memory associated with image data as well.
   *
   * @return uint32_t : number of refs to the frame
   */
  virtual uint32_t release_ref() = 0;
  /**
   * frame timestamp
   */
  uint64_t timestamp;
  /**
   * frame data size in bytes
   */
  uint32_t size;
  /**
   * pointer to start of image data
   */
  uint8_t * data;
  /**
   * file descriptor for the frame memory
   * If value is -1, it indicates the memory is not shared
   */
  int fd;
  /**
   * opaque metadata about the frame memory. This is used for
   * sharing memory between camera and video encoder
   */
  void * metadata;
  uint32_t stride;
  uint32_t slice;

  std::string frame_source;
  std::string format;

  /**
   * it work when the format is raw10
   * 0 means left camera and 1 means right camera
   */
  int orientation;

  uint32_t camera_id;
  uint32_t index;

  uint32_t width;
  uint32_t height;
};
/**
 * Interface for camera listener object. Client needs to
 * implement this interface to get access to camera data and
 * control events. The methods in listener can be invoked from
 * multiple different threads. Client needs to make sure that
 * implementation is thread-safe.
 */
class ICameraListener
{
public:
  ICameraListener() {}

  virtual ~ICameraListener() {}
  /**
   * This function is called when an error is generated by camera
   * driver
   */
  virtual void on_error() {}
  /**
   * This function is called when a preview frame is generated by
   * camera.
   *
   * @param frame [in]: pointer to an existing ICameraFrame
   *              generated by camera
   */
  virtual void on_preview_frame(ICameraFrame * frame) {}
  /**
   * This function is called when a preview frame is generated by
   * camera.
   *
   * @param frame [in]: pointer to an existing ICameraFrame
   *              generated by camera
   */
  virtual void on_video_frame(ICameraFrame * frame) {}
  /**
   * This function is called when a snapshot frame is generated by
   * camera.
   *
   * @param frame [in]: pointer to an existing ICameraFrame
   *              generated by camera
   */
  virtual void on_picture_frame(ICameraFrame * frame) {}
  /**
   * This function is called when a metadata frame is generated by
   * camera.
   *
   * @param frame [in]: pointer to an existing ICameraFrame
   *              generated by camera
   */
  virtual void on_metadata_frame(ICameraFrame * frame) {}
};

struct StreamParam
{
  uint32_t width;
  uint32_t height;
  uint32_t frame_rate;
  std::string format;
  int32_t orientation;
  std::string name;
};

struct CameraParam
{
  std::vector<StreamParam> track_params;
  std::vector<StreamParam> snapshot_params;
  std::vector<StreamParam> video_params;
};

enum class CameraType
{
  QMMF
};

/**
 * Interface to a camera device. Client uses the API provided by
 * this interface to interact with camera device.
 */
class ICameraDevice
{
  ICameraDevice(const ICameraDevice &) = delete;
  void operator=(const ICameraDevice &) = delete;

public:
  ICameraDevice(uint32_t id) : camera_id_(id) {}

  virtual ~ICameraDevice() {}
  /**
   * Add listener to handle various notifications and data frames from the
   * camera device.
   * @param listener [in]
   **/
  virtual void add_listener(ICameraListener * listener) = 0;
  /**
   * Removes a previously added listener from camera device
   *
   * @param listener [in]
   */
  virtual void remove_listener(ICameraListener * listener) = 0;

  virtual int start_camera() = 0;

  virtual int stop_camera() = 0;

  virtual void set_param(std::vector<CameraParam> & param) = 0;

  virtual bool init() = 0;

  bool working() { return is_working_; };

  CameraType get_type() { return type_; }

  uint32_t get_id() { return camera_id_; }

protected:
  bool is_working_;
  CameraType type_;
  uint32_t camera_id_;
};
}  // namespace qrb_camera
#endif
