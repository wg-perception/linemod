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
from object_recognition_linemod.ecto_cells.ecto_linemod import Detector
from ecto.opts import run_plasm, scheduler_options
from object_recognition_linemod.detector import LinemodDetector
from object_recognition_core.utils.json_helper import obj_to_cpp_json_str
from object_recognition_core.db import ObjectDb, ObjectDbParameters, Models
from ecto_image_pipeline.io.source import create_source
from object_recognition_core.utils import json_helper

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Compute a LINE-MOD model and saves it to the DB.')
    scheduler_options(parser.add_argument_group('Scheduler'))
    options = parser.parse_args()

    return options


if __name__ == '__main__':
    options = parse_args()

    plasm = ecto.Plasm()

    json_db_params = obj_to_cpp_json_str({'type': 'CouchDB', 'root': 'http://bwl.willowgarage.com:5984',
                                        'collection': 'object_recognition'})

    #setup the input source, grayscale conversion
    from ecto_openni import VGA_RES, FPS_30
    source = create_source('image_pipeline','OpenNISource',image_mode=VGA_RES,image_fps=FPS_30)

    object_ids = obj_to_cpp_json_str(['whoolite', 'tilex'])
    detector = Detector(json_object_ids=object_ids, json_db=json_db_params, threshold=90, visualize=True)

    #connect up the pose_est
    plasm.connect(source['image'] >> detector['image'],
                  source['depth'] >> detector['depth']
                  )
    plasm.connect(source['image'] >> imshow(name='source')[:])

    if 0:
        import ecto_ros
        import ecto_ros.ecto_sensor_msgs
        import sys
        ecto_ros.init(sys.argv, "image_pub")
        mat2image = ecto_ros.Mat2Image(frame_id='base', encoding='bgr8')
        pub_rgb = ecto_ros.ecto_sensor_msgs.Publisher_Image("image pub", topic_name='linemod_image')
        plasm.connect(detector['image'] >> mat2image[:],
                      mat2image[:] >> pub_rgb[:])

    run_plasm(options, plasm, locals=vars())
