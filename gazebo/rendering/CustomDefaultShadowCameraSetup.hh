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

#ifndef GAZEBO_RENDERING_CUSTOMDEFAULTSHADOWCAMERASETUP_HH_
#define GAZEBO_RENDERING_CUSTOMDEFAULTSHADOWCAMERASETUP_HH_

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class GAZEBO_VISIBLE CustomDefaultShadowCameraSetup
          : public Ogre::DefaultShadowCameraSetup
    {
      /// \brief Constructor, defaults to 3 splits
      public: CustomDefaultShadowCameraSetup();

      /// \brief Destructor
      public: ~CustomDefaultShadowCameraSetup();

      /// \brief Returns a shadow camera with PSSM splits based on iteration.
      /// \sa FocusedShadowCameraSetup::getShadowCamera()
      public: virtual void getShadowCamera(const Ogre::SceneManager *_sm,
          const Ogre::Camera *_cam, const Ogre::Viewport *_vp,
          const Ogre::Light *_light, Ogre::Camera *_texCam, size_t _iteration)
          const override;
    };
  }
}


#endif
