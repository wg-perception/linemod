#!/usr/bin/env python
"""
Module defining the LINE-MOD detector to find objects in a scene
"""

from object_recognition_core.pipelines.detection import DetectorBase
import ecto_cells.ecto_linemod as ecto_linemod

########################################################################################################################

class LinemodDetector(ecto_linemod.Detector, DetectorBase):

    def __init__(self, *args, **kwargs):
        ecto_linemod.Detector.__init__(self, *args, **kwargs)
        DetectorBase.__init__(self)
