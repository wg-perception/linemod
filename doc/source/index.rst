.. _line_mod:

object_recognition_linemod: Object Recognition Using LINE-MOD
#############################################################

This pipeline implements LINE-MOD. For more information, see http://ar.in.tum.de/Main/StefanHinterstoisser. It is one of
the best methods out there for generic rigid object recognition and it proceeds using very fast template matching. The
version used in that package is the same as the original paper (and it is in `OpenCV <http://opencv.org/>`_) but the
tricks to get it work properly are in the pre/post processing steps.

First steps
***********

LINE-MOD requires a mesh in order to train a model. You can either use the :ref:`ORK 3d capture <orkcapture:ork_capture>` or,
if you already have a mesh around, just create your object:

.. code-block:: sh

   ./ork_core/apps/dbscripts/object_add.py -n coke -d "A universal can of coke"

and then upload the mesh (use the .obj one, the .blend seems to not work with Assimp right now):

.. code-block:: sh

   ./ork_core/apps/dbscripts/mesh_add.py YOUR_OBJECT_ID YOUR_COKE_BLEND_PATH --commit

Training step
*************

To generate all the templates, it uses an :ref:`automatic view generator <orkrenderer:renderer>`. Those also vary the
in-plane rotations of the cameras and deals with the different scales and view points. Thousands of images with
depth+mask are generated and fed to the OpenCV trainer.


Processing step:
****************

The OpenCV detector is simply called

Post-processing step:
*********************

TODO: implement an ICP step like the ACCV paper, probably:
``Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration`` from Kok-Lim Low.
