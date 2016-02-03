/** @file
@brief Header

@date 2016

@author
Sensics, Inc.
<http://sensics.com/osvr>
*/

// Copyright 2015 Vuzix Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
// - none

// Library/third-party includes
/*
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/freeglut.h>
#endif*/
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#define snprintf _snprintf
#endif

#include <ARWrapper/ARController.h>

// Standard includes
#include <iostream>

// ============================================================================
//	Constants
// ============================================================================

#define VIEW_SCALEFACTOR                                                       \
  1.0 // Units received from ARToolKit tracking will be multiplied by this
      // factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN                                                      \
  40.0 // Objects closer to the camera than this will not be displayed. OpenGL
       // units.
#define VIEW_DISTANCE_MAX                                                      \
  10000.0 // Objects further away from the camera than this will not be
          // displayed. OpenGL units.
#define MATRIX_COLS 4
#define MATRIX_ROWS 3

namespace {

// Preferences.
static int imageWidth =
    640; // Initial window width, also updated during program execution.
static int imageHeight =
    480; // Initial window height, also updated during program execution.

// Image acquisition.
static ARUint8 *gARTImage = NULL;
static int gARTImageSavePlease = FALSE;

// Marker detection.
static ARHandle *gARHandle = NULL;
static ARPattHandle *gARPattHandle = NULL;
static long gCallCountMarkerDetect = 0;

// Transformation matrix retrieval.
static AR3DHandle *gAR3DHandle = NULL;
static ARdouble gPatt_width =
    80.0; // Per-marker, but we are using only 1 marker.
static ARdouble
    gPatt_trans[MATRIX_ROWS]
               [MATRIX_COLS];   // Per-marker, but we are using only 1 marker.
static int gPatt_found = FALSE; // Per-marker, but we are using only 1 marker.
static int gPatt_id;            // Per-marker, but we are using only 1 marker.

// Drawing.
static ARParamLT *gCparamLT;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0; // For use in drawing.

static bool initialized = false;

AR_PIXEL_FORMAT pixFormat = AR_PIXEL_FORMAT_BGR;

class OSVRARToolKitController {
public:
  OSVRARToolKitController() {}

  ~OSVRARToolKitController() {}

  bool isInitialized() { return initialized; }

  void DetectMarkers(OSVR_TimeValue const &timestamp,
                     OSVR_ImagingReport const &report) {
    // static int imageNumber = 0; //used for saving images
    ARUint8 *image;
    ARdouble err;
    int j, k;

    // Grab a video frame from the imaging report
    if ((image = report.state.data) != NULL) {
      gARTImage = image;

      // Save the fetched image.
      /*if (gARTImageSavePlease) {
              char imageNumberText[15];
              sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
              if (arVideoSaveImageJPEG(gARHandle->xsize, gARHandle->ysize,
      gARHandle->arPixelFormat, gARTImage, imageNumberText, 75, 0) < 0) {
                      ARLOGe("Error saving video image.\n");
              }
              gARTImageSavePlease = FALSE;
      }*/

      gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.
      // Detect the markers in the video frame.
      if (arDetectMarker(gARHandle, gARTImage) < 0) {
        return;
      }

      // Check through the marker_info array for highest confidence
      // visible marker matching our preferred pattern.
      k = -1;
      for (j = 0; j < gARHandle->marker_num; j++) {
        if (gARHandle->markerInfo[j].id == gPatt_id) {
          if (k == -1)
            k = j; // First marker detected.
          else if (gARHandle->markerInfo[j].cf > gARHandle->markerInfo[k].cf)
            k = j; // Higher confidence marker detected.
        }
      }

      if (k != -1) {
        // Get the transformation between the marker and the real camera into
        // gPatt_trans.
        err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[k]),
                                  gPatt_width, gPatt_trans);
        gPatt_found = TRUE;
      } else {
        gPatt_found = FALSE;
      }
    }
  }

  void initARToolKit(const char *window_name) {
    // see ARToolKit's simpleLite.c example
    std::cout << "initARToolKit" << std::endl;
    const char *cparam_name = "Data/camera_para.dat";
    char vconf[] = "";
    const char *patt_name = "Data/patt.hiro";

    if (!createARHandle(cparam_name, vconf, &gCparamLT, &gARHandle,
                        &gAR3DHandle)) {
      ARLOGe("main(): Unable to set up AR camera.\n");
    }
    std::cout << "Successfully set up AR camera." << std::endl;

    // arglSetupDebugMode(gArglSettings, gARHandle);
    arUtilTimerReset();

    // Load marker
    //@todo support multiple markers.
    if (!setupARMarker(patt_name, &gPatt_id, gARHandle, &gARPattHandle)) {
      ARLOGe("initARToolKit(): Unable to set up AR marker.\n");
      cleanup();
    }
    std::cout << "Successfully set up AR marker." << std::endl;

    initialized = true;
  }

  /* cleanup function called when program exits */
  void cleanup(void) {
    // arVideoCapStop();
    // argCleanup(); //part of gsub library
    arPattDetach(gARHandle);
    arPattDeleteHandle(gARPattHandle);
    ar3DDeleteHandle(&gAR3DHandle);
    arDeleteHandle(gARHandle);
    arParamLTFree(&gCparamLT);
    // arVideoClose();
  }

  bool isMarkerDetected() { return gPatt_found; }

  ARdouble (&OSVRARToolKitController::getMarkerTransform())[MATRIX_ROWS]
                                                           [MATRIX_COLS] {
    return gPatt_trans;
  }

private:
  /*Create a handle to hold settings for an ARToolKit tracker instance.

  ARHandle is the primary structure holding the settings for a single ARToolKit
  square marker tracking instance. Settings include expected video stream image
  size and pixel format, tracking modes, loaded markers and more.

  Expected video stream image size is taken directly from the supplied ARParamLT
  structure's xsize and ysize fields. Video stream image pixel format defaults
  to
  AR_DEFAULT_PIXEL_FORMAT, which is platform and video-module dependent. Usually
  a call to arSetPixelFormat() is advisable to set the correct format.*/
  int createARHandle(const char *cparam_name, char *vconf,
                     ARParamLT **cparamLT_p, ARHandle **arhandle,
                     AR3DHandle **ar3dhandle) {
    // Structure holding camera parameters, including image size, projection
    // matrix and lens distortion parameters.
    ARParam cameraParameters;

    // Load the camera parameters from data file, resize for the window and
    // init.
    if (arParamLoad(cparam_name, 1, &cameraParameters) < 0) {
      ARLOGe("createARHandle(): Error loading parameter file %s for camera.\n",
             cparam_name);
      arVideoClose();
      return (FALSE);
    }

    // overwrite the image width and height from the ARToolKit data file
    if (cameraParameters.xsize != imageWidth ||
        cameraParameters.ysize != imageHeight) {
      ARLOGw("*** Camera Parameter resized from %d, %d. ***\n",
             cameraParameters.xsize, cameraParameters.ysize);
      arParamChangeSize(&cameraParameters, imageWidth, imageHeight,
                        &cameraParameters);
    }

    /*Allocate and calculate a lookup - table camera parameter from a standard
      camera parameter.
      A lookup - table based camera parameter offers significant performance
      savings in certain ARToolKit operations(including unwarping of pattern
      spaces)
      compared to use of the standard camera parameter.*/
    if ((*cparamLT_p = arParamLTCreate(&cameraParameters,
                                       AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
      ARLOGe("createARHandle(): Error: arParamLTCreate.\n");
      return (FALSE);
    }

    // Create the ARHandle
    if ((*arhandle = arCreateHandle(*cparamLT_p)) == NULL) {
      ARLOGe("createARHandle(): Error: arCreateHandle.\n");
      return (FALSE);
    }

    // Set the pixel format
    //@todo find the actual pixel format of the image, instead of hardcoding BGR
    // for OpenCV
    if (arSetPixelFormat(*arhandle, pixFormat) < 0) {
      ARLOGe("createARHandle(): Error: arSetPixelFormat.\n");
      return (FALSE);
    }

    /*Create handle used for 3D calculation from calibrated camera parameters.

    An AR3DHandle holds data structures used in calculating the 3D pose of a
    marker from the 2D location of its corners(i.e.pose estimation).*/
    if ((*ar3dhandle = ar3DCreateHandle(&cameraParameters)) == NULL) {
      ARLOGe("createARHandle(): Error: ar3DCreateHandle.\n");
      return (FALSE);
    }
    return (TRUE);
  }

  // Setup a tracked marker by associating a pattern with an ARHandle
  int setupARMarker(const char *patt_name, int *patt_id, ARHandle *arhandle,
                    ARPattHandle **pattHandle_p) {
    /*Allocate a pattern handle.
Allocates an empty pattern handle, into which patterns can
  be loaded by calling arPattLoad().*/
    if ((*pattHandle_p = arPattCreateHandle()) == NULL) {
      ARLOGe("setupARMarker(): Error: arPattCreateHandle.\n");
      return (FALSE);
    }

    // Loading only 1 pattern in this example.
    if ((*patt_id = arPattLoad(*pattHandle_p, patt_name)) < 0) {
      ARLOGe("setupARMarker(): Error loading pattern file %s.\n", patt_name);
      arPattDeleteHandle(*pattHandle_p);
      return (FALSE);
    }

    /*Associate a set of patterns with an ARHandle.
      Associating a set of patterns with an ARHandle makes
      the patterns the set which will be searched when marker
      identification is performed on an image associated with the
      same ARHandle.*/
    arPattAttach(arhandle, *pattHandle_p);

    return (TRUE);
  }
};
} // namespace
