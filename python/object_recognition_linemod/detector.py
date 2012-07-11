#!/usr/bin/env python
"""
Module defining the LINE-MOD detector to find objects in a scene
"""

from object_recognition_core.db import ObjectDb, Models
from object_recognition_core.utils import json_helper
from object_recognition_core.pipelines.detection import DetectionPipeline
import ecto_cells.ecto_linemod as ecto_linemod

########################################################################################################################

class LinemodDetectionPipeline(DetectionPipeline):
    @classmethod
    def type_name(cls):
        return 'LINEMOD'
    @classmethod
    def detector(self, *args, **kwargs):
        #visualize = args.get('visualize', False)
        submethod = kwargs.get('submethod')
        parameters = kwargs.get('parameters')
        object_ids = parameters['object_ids']
        object_db = ObjectDb(parameters['db'])
        model_documents = Models(object_db, object_ids, self.type_name(), json_helper.dict_to_cpp_json_str(submethod))
        threshold = parameters.get('threshold', 90)
        return ecto_linemod.Detector(model_documents=model_documents, db=object_db, threshold=threshold)
