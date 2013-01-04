#!/usr/bin/env python
"""
Module defining the LINE-MOD trainer to train the LINE-MOD models
"""

from object_recognition_core.pipelines.training import TrainingPipeline
from object_recognition_core.utils.json_helper import dict_to_cpp_json_str
import ecto
import ecto_cells.ecto_linemod as ecto_linemod

########################################################################################################################

class LinemodTrainingPipeline(TrainingPipeline):
    '''Implements the training pipeline functions'''

    @classmethod
    def type_name(cls):
        return "LINEMOD"

    @classmethod
    def incremental_model_builder(cls, *args, **kwargs):
        submethod = kwargs.get('submethod')
        mesh_path = kwargs.get('pipeline_params').get('path')
        return ecto_linemod.Trainer(path=mesh_path, json_submethod=dict_to_cpp_json_str(submethod))

    @classmethod
    def post_processor(cls, *args, **kwargs):
        return ecto_linemod.ModelFiller()
