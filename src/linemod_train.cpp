#include <ecto/ecto.hpp>

#include <object_recognition_core/common/json.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using ecto::tendrils;
using ecto::spore;

namespace ecto_linemod
{
  struct Trainer
  {
    static void
    declare_params(tendrils& params)
    {
      /// @todo Parameters for various LINE-MOD settings?
      params.declare(&Trainer::json_submethod_, "json_submethod", "The submethod to use, as a JSON string.").required(
          true);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Trainer::color_, "image", "An rgb full frame image.").required(true);
      inputs.declare(&Trainer::depth_, "depth", "The 16bit depth image.").required(true);
      inputs.declare(&Trainer::mask_, "mask", "The mask.").required(true);

      outputs.declare(&Trainer::detector_, "detector", "The detector.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      or_json::mValue submethod = object_recognition_core::to_json(*json_submethod_);
      if (submethod.get_str() == "DefaultLINEMOD")
        *detector_ = cv::linemod::getDefaultLINEMOD();
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Resize color and mask to 640x480
      /// @todo Move resizing to separate cell, and try LINE-MOD w/ SXGA images
      cv::Mat color;
      cv::pyrDown(color_->rowRange(0, 960), color);
      cv::Mat mask;
      cv::resize(mask_->rowRange(0, 960), mask, cv::Size(640, 480), 0.0, 0.0, CV_INTER_NN);

      std::vector<cv::Mat> sources;
      sources.push_back(color);
      sources.push_back(*depth_);

      int template_id = (**detector_).addTemplate(sources, "bogus_id", mask);

      return ecto::OK;
    }

    spore<cv::Ptr<cv::linemod::Detector> > detector_;
    spore<cv::Mat> color_, depth_, mask_;
    spore<std::string> json_submethod_;
  };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.");
