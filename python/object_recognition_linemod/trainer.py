#!/usr/bin/env python
"""
Module defining the LINE-MOD trainer to train the LINE-MOD models
"""

from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from object_recognition_core.ecto_cells.db import ModelWriter
from object_recognition_core.pipelines.training import TrainerBase
import ecto
import ecto_cells.ecto_linemod as ecto_linemod

########################################################################################################################

class LinemodTrainer(ecto.BlackBox, TrainerBase):
    '''Implements the training pipeline functions'''

    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        TrainerBase.__init__(self, *args, **kwargs)

    @classmethod
    def declare_cells(cls, _p):
        return {'model_filler': CellInfo(ecto_linemod.ModelFiller),
                'model_writer': CellInfo(ModelWriter),
                'trainer': CellInfo(ecto_linemod.Trainer)}

    @classmethod
    def declare_forwards(cls, _p):
        p = {'model_writer': 'all', 'trainer': 'all'}
        i = {}
        o = {}

        return (p, i, o)

    def connections(self, _p):
        connections = []

        # connect the output to the post-processor
        for key in set(self.trainer.outputs.keys()).intersection(self.model_filler.inputs.keys()):
            connections += [self.trainer[key] >> self.model_filler[key]]

        # and write everything to the DB
        connections += [self.model_filler["db_document"] >> self.model_writer["db_document"]]

        return connections
