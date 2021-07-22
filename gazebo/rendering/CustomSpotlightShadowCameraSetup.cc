/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Code in this file has been adapted from Ogre's OgreShadowCameraSetup.
// The original Ogre's licence and copyright headers are copied below:

/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "gazebo/rendering/CustomSpotlightShadowCameraSetup.hh"
#include "gazebo/rendering/ogre_gazebo.h"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
CustomSpotlightShadowCameraSetup::CustomSpotlightShadowCameraSetup()
{
}

//////////////////////////////////////////////////
CustomSpotlightShadowCameraSetup::~CustomSpotlightShadowCameraSetup()
{
}

//////////////////////////////////////////////////
void CustomSpotlightShadowCameraSetup::getShadowCamera(const Ogre::SceneManager *_sm,
    const Ogre::Camera *_cam, const Ogre::Viewport *_vp,
    const Ogre::Light *_light, Ogre::Camera *_texCam, size_t _iteration) const
{
  Ogre::Vector3 pos, dir;

  // reset custom view / projection matrix in case already set
  _texCam->setCustomViewMatrix(false);
  _texCam->setCustomProjectionMatrix(false);
  _texCam->setNearClipDistance(_light->_deriveShadowNearClipDistance(_cam) - 0.001);
  _texCam->setFarClipDistance(_light->_deriveShadowFarClipDistance(_cam));

  // get the shadow frustum's far distance
  Ogre::Real shadowDist = _light->getShadowFarDistance();
  if (!shadowDist)
  {
    // need a shadow distance, make one up
    shadowDist = _cam->getNearClipDistance() * 300;
  }
  Ogre::Real shadowOffset = shadowDist * (_sm->getShadowDirLightTextureOffset());

  // Set perspective projection
  _texCam->setProjectionType(Ogre::PT_PERSPECTIVE);
  // set FOV slightly larger than the spotlight range to ensure coverage
  Ogre::Radian fovy = _light->getSpotlightOuterAngle()*1.2;
  // limit angle
  if (fovy.valueDegrees() > 175)
    fovy = Ogre::Degree(175);
  _texCam->setFOVy(fovy);

  // Calculate position, which same as spotlight position
  pos = _light->getDerivedPosition();

  // Calculate direction, which same as spotlight direction
  dir = - _light->getDerivedDirection(); // backwards since point down -z
  dir.normalise();

  // Finally set position
  _texCam->setPosition(pos);

  // Calculate orientation based on direction calculated above
  /*
  // Next section (camera oriented shadow map) abandoned
  // Always point in the same direction, if we don't do this then
  // we get 'shadow swimming' as camera rotates
  // As it is, we get swimming on moving but this is less noticeable

  // calculate up vector, we want it aligned with _cam direction
  Vector3 up = _cam->getDerivedDirection();
  // Check it's not coincident with dir
  if (up.dotProduct(dir) >= 1.0f)
  {
  // Use camera up
  up = _cam->getUp();
  }
  */
  Ogre::Vector3 up = Ogre::Vector3::UNIT_Y;
  // Check it's not coincident with dir
  if (Ogre::Math::Abs(up.dotProduct(dir)) >= 1.0f)
  {
    // Use camera up
    up = Ogre::Vector3::UNIT_Z;
  }
  // cross twice to rederive, only direction is unaltered
  Ogre::Vector3 left = dir.crossProduct(up);
  left.normalise();
  up = dir.crossProduct(left);
  up.normalise();
  // Derive Ogre::quaternion from axes
  Ogre::Quaternion q;
  q.FromAxes(left, up, dir);
  _texCam->setOrientation(q);
}