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

// Code in this file has been adapted from Ogre's RTShader::IntegratedPSSM3,
// and different ShadowCameraSetup classes. The original Ogre's licence and
// copyright headers are copied below:

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

#include "gazebo/rendering/CustomDefaultShadowCameraSetup.hh"
#include "gazebo/rendering/ogre_gazebo.h"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
CustomDefaultShadowCameraSetup::CustomDefaultShadowCameraSetup()
{
}

//////////////////////////////////////////////////
CustomDefaultShadowCameraSetup::~CustomDefaultShadowCameraSetup()
{
}

//////////////////////////////////////////////////
void CustomDefaultShadowCameraSetup::getShadowCamera(const Ogre::SceneManager *_sm,
    const Ogre::Camera *_cam, const Ogre::Viewport *_vp, const Ogre::Light *_light,
    Ogre::Camera *_texCam, size_t _iteration) const
{
  Ogre::Vector3 pos, dir;

  // reset custom view / projection matrix in case already set
  _texCam->setCustomViewMatrix(false);
  _texCam->setCustomProjectionMatrix(false);
  _texCam->setNearClipDistance(_light->_deriveShadowNearClipDistance(_cam));
  _texCam->setFarClipDistance(_light->_deriveShadowFarClipDistance(_cam));

  // get the shadow frustum's far distance
  Ogre::Real shadowDist = _light->getShadowFarDistance();
  if (!shadowDist)
  {
    // need a shadow distance, make one up
    shadowDist = _cam->getNearClipDistance() * 300;
  }
  Ogre::Real shadowOffset = shadowDist * (_sm->getShadowDirLightTextureOffset());

  // Directional lights 
  if (_light->getType() == Ogre::Light::LT_DIRECTIONAL)
  {
    // set up the shadow texture
    // Set ortho projection
    _texCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    // set ortho window so that texture covers far dist
    _texCam->setOrthoWindow(shadowDist * 2, shadowDist * 2);

    // Calculate look at position
    // We want to look at a spot shadowOffset away from near plane
    // 0.5 is a little too close for angles
    Ogre::Vector3 target = _cam->getDerivedPosition() + (_cam->getDerivedDirection() * shadowOffset);

    // Calculate direction, which same as directional light direction
    dir = - _light->getDerivedDirection(); // backwards since point down -z
    dir.normalise();

    // Calculate position
    // We want to be in the -ve direction of the light direction
    // far enough to project for the dir light extrusion distance
    pos = target + dir * _sm->getShadowDirectionalLightExtrusionDistance();

    // Round local x/y position based on a world-space texel; this helps to reduce
    // jittering caused by the projection moving with the camera
    // Viewport is 2 * near clip distance across (90 degree fov)
    //~ Real worldTexelSize = (texCam->getNearClipDistance() * 20) / vp->getActualWidth();
    //~ pos.x -= fmod(pos.x, worldTexelSize);
    //~ pos.y -= fmod(pos.y, worldTexelSize);
    //~ pos.z -= fmod(pos.z, worldTexelSize);
    Ogre::Real worldTexelSize = (shadowDist * 2) / _texCam->getViewport()->getActualWidth();

    //get texCam orientation

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
    // Derive quaternion from axes
    Ogre::Quaternion q;
    q.FromAxes(left, up, dir);

    //convert world space camera position into light space
    Ogre::Vector3 lightSpacePos = q.Inverse() * pos;

    //snap to nearest texel
    lightSpacePos.x -= fmod(lightSpacePos.x, worldTexelSize);
    lightSpacePos.y -= fmod(lightSpacePos.y, worldTexelSize);

    //convert back to world space
    pos = q * lightSpacePos;

  }
  // Spotlight
  else if (_light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    // Set perspective projection
    _texCam->setProjectionType(Ogre::PT_PERSPECTIVE);
    // set FOV slightly larger than the spotlight range to ensure coverage
    Ogre::Radian fovy = _light->getSpotlightOuterAngle()*1.2;
    // limit angle
    if (fovy.valueDegrees() > 175)
    {
      fovy = Ogre::Degree(175);
    }
    _texCam->setFOVy(fovy);

    // Calculate position, which same as spotlight position
    pos = _light->getDerivedPosition();

    // Calculate direction, which same as spotlight direction
    dir = - _light->getDerivedDirection(); // backwards since point down -z
    dir.normalise();
  }
  // Point light
  else
  {
    // Set perspective projection
    _texCam->setProjectionType(Ogre::PT_PERSPECTIVE);
    // Use 120 degree FOV for point light to ensure coverage more area
    _texCam->setFOVy(Ogre::Degree(120));

    // Calculate look at position
    // We want to look at a spot shadowOffset away from near plane
    // 0.5 is a little too close for angles
    Ogre::Vector3 target = _cam->getDerivedPosition() + (_cam->getDerivedDirection() * shadowOffset);

    // Calculate position, which same as point light position
    pos = _light->getDerivedPosition();

    dir = (pos - target); // backwards since point down -z
    dir.normalise();
  }

  // Finally set position
  _texCam->setPosition(pos);

  // Calculate orientation based on direction calculated above
  /*
  // Next section (camera oriented shadow map) abandoned
  // Always point in the same direction, if we don't do this then
  // we get 'shadow swimming' as camera rotates
  // As it is, we get swimming on moving but this is less noticeable

  // calculate up vector, we want it aligned with cam direction
  Vector3 up = cam->getDerivedDirection();
  // Check it's not coincident with dir
  if (up.dotProduct(dir) >= 1.0f)
  {
  // Use camera up
  up = cam->getUp();
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
  // Derive quaternion from axes
  Ogre::Quaternion q;
  q.FromAxes(left, up, dir);
  _texCam->setOrientation(q);
}