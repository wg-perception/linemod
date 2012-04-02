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
        return ecto_linemod.Trainer(json_submethod=dict_to_cpp_json_str(submethod))

    @classmethod
    def post_processor(cls, *args, **kwargs):
        #if not search_params:
        #    raise RuntimeError("You must supply search parameters for TOD.")
        return ecto_linemod.ModelFiller()
