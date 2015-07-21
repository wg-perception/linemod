0.3.7 (2015-07-21)
------------------
* small changes in training: object distance and mesh path
* Contributors: Vincent Rabaud, nlyubova

0.3.6 (2015-04-20)
------------------
* fixing training and detection for meshes generated with the Reconstration pipeline
* Contributors: nlyubova

0.3.5 (2015-04-19)
------------------
* fixed the depth conversion to 32F, updated possible mesh names for both training and detection
* Contributors: nlyubova

0.3.4 (2015-04-14)
------------------
* remove the sample folder: a lot of old non-working code
  This also fixes `#11 <https://github.com/wg-perception/linemod/issues/11>`_
* fix `#13 <https://github.com/wg-perception/linemod/issues/13>`_ fully
* Merge pull request `#13 <https://github.com/wg-perception/linemod/issues/13>`_ from nlyubova/master
  Fixed issue with openNI2 while handing both 32F and 8U depth formats
* Fixed issue with openNI2 while handing both 32F and 8U depth formats
* Contributors: Vincent Rabaud, nlyubova

0.3.3 (2015-01-20)
------------------
* Fixed input params for training
* use the new renderer API
* Contributors: Vincent Rabaud, nlyubova

0.3.2 (2015-01-18)
------------------
* fix compilation on 32 bits
* update OpenCV dependency
* Contributors: Vincent Rabaud

0.3.1 (2015-01-18)
------------------
* fixed object orientation,
  fixed icp,
  added detection of multiple objects,
  added visualization of point clouds
* clean extensions
* update docs
* Updated render function to match latest ork_renderer commit bfcdbc2c74b21ac07b20c953f994269a427eb25c
* fix some runtime behavior
* Contributors: Isura, Vincent Rabaud, nlyubova

0.3.0 (2014-01-29  01:59:06 +0100)
----------------------------------
- better mesh handling
- drop Fuerte support
