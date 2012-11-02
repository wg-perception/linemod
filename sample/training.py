#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Willow Garage, Inc. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.


import ecto

from ecto_opencv.highgui import imshow
from ecto_opencv.imgproc import cvtColor, Conversion
from object_recognition_linemod.ecto_cells.ecto_linemod import Trainer, ModelFiller
from ecto.opts import run_plasm, scheduler_options
from object_recognition_core.ecto_cells.db import ModelWriter
from object_recognition_linemod.detector import LinemodDetectionPipeline
from object_recognition_core.utils.json_helper import dict_to_cpp_json_str, list_to_cpp_json_str
from object_recognition_core.db import ObjectDbParameters

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Compute a LINE-MOD model and saves it to the DB.')
    scheduler_options(parser.add_argument_group('Scheduler'))
    options = parser.parse_args()

    return options


if __name__ == '__main__':
    options = parse_args()

    plasm = ecto.Plasm()

    #setup the input source, grayscale conversion
    trainer = Trainer(path='/wg/stor2a/vrabaud/workspace/recognition_kitchen_groovy/build/buildspace/lib/object_recognition_reconstruction')
    model_filler = ModelFiller()

    #connect up the pose_est
    plasm.connect(trainer['detector'] >>  model_filler['detector'])

    object_id = 'whoolite'
    writer = ModelWriter(session_ids=list_to_cpp_json_str([]),
                        object_id=object_id, db_params=ObjectDbParameters({'type': 'CouchDB',
                                                                              'root': 'http://bwl.willowgarage.com:5984',
                                                                              'collection': 'object_recognition'}),
                        method=LinemodDetectionPipeline.type_name(),
                        json_submethod=dict_to_cpp_json_str({}),
                        json_params=dict_to_cpp_json_str({}))

    plasm.connect(model_filler["db_document"] >> writer["db_document"])

    options.niter = 1
    run_plasm(options, plasm, locals=vars())
