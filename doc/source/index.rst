.. _line_mod:

object_recognition_linemod: Object Recognition Using LINE-MOD
#############################################################

This pipeline implements LINE-MOD. For more information, see http://ar.in.tum.de/Main/StefanHinterstoisser. It is one of
the best methods out there for generic rigid object recognition and it proceeds using very fast template matching. The
version used in that package is the same as the original paper (and it is in `OpenCV <http://opencv.org/>`_) but the
tricks to get it work properly are in the pre/post processing steps.

Pre-processing step:
********************

To generate all the templates, it uses an :ref:`automatic view generator <orkrenderer:renderer>`. Those vary the
in-plane rotations of the cameras and deals with the different scales and view points. Thousands of images with
depth+mask are generated and fed to the OpenCV trainer.


Processing step:
****************

The OpenCV detector is simply called

Post-processing step:
*********************

TODO: implement an ICP step like the ACCV paper, probably:
``Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration`` from Kok-Lim Low.
