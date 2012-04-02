#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>

#include "db_linemod.h"

using object_recognition_core::db::ObjectId;
using object_recognition_core::db::Document;

namespace linemod_ecto
{
  struct ModelFiller
  {
  public:
    static void
    declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      typedef ModelFiller C;
      inputs.declare(&C::detector_, "detector", "The Linemod detector.").required(true);

      outputs.declare(&C::db_document_, "db_document", "The filled document.");
    }

    int
    process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      db_document_->set_attachment<cv::linemod::Detector>("detector", **detector_);
      return ecto::OK;
    }

  private:
    ecto::spore<cv::Ptr<cv::linemod::Detector> > detector_;
    ecto::spore<Document> db_document_;
  };
}

ECTO_CELL(ecto_linemod, linemod_ecto::ModelFiller, "ModelFiller",
          "Populates a db document with a TOD model for persisting a later date.")
