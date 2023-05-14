#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include <cnoid/Camera>
#include <cnoid/SimpleController>

class VisionController : public cnoid::SimpleController
{
public:
  VisionController() : sampling_time_(0.1) {}
  ~VisionController() = default;

  virtual bool initialize(cnoid::SimpleControllerIO * io) override
  {
    io_ = io;
    camera_ptr_ = io->body()->findDevice<cnoid::Camera>("camera");
    io->enableInput(camera_ptr_);
    return true;
  }

  virtual bool control() override
  {
    double time = io_->currentTime();

    if (sampling_time_ < std::fabs(time - previous_time_)) {
      const cnoid::Image & image_raw = camera_ptr_->constImage();

      const int height = image_raw.height();
      const int width = image_raw.width();

      cv::Mat sensor_image(height, width, CV_8UC3, const_cast<uchar *>(&image_raw.pixels()[0]));

      cv::Mat gray_image, canny_image;
      cvtColor(sensor_image, gray_image, cv::COLOR_RGB2GRAY);
      Canny(gray_image, canny_image, 50, 200);

      cv::imshow("vision", canny_image);

      previous_time_ = time;
    }

    return true;
  }

private:
  cnoid::SimpleControllerIO * io_;
  cnoid::CameraPtr camera_ptr_;
  double previous_time_{0.0};
  double sampling_time_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VisionController)