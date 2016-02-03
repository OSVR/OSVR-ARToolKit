/** @file
    @brief Comprehensive example: Implementation of a dummy Hardware Detect
   Callback that creates a dummy device when it is "detected"

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
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
// Internal Includes
#include <osvr/AnalysisPluginKit/AnalysisPluginKitC.h>
#include <osvr/ClientKit/Imaging.h>
#include <osvr/ClientKit/InterfaceC.h>
#include <osvr/ClientKit/InterfaceCallbackC.h>
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/Util/StringLiteralFileToString.h>

#include <osvr/Util/EigenFilters.h>
#include <osvr/Util/EigenInterop.h>

// Generated JSON header file
#include "org_osvr_artoolkit_json.h"

// Library/third-party includes
#include <json/reader.h>
#include <json/value.h>

// Standard includes
#include <iostream>
#include <memory>

#include "OSVRARToolKitController.h"

// Anonymous namespace to avoid symbol collision
namespace {

static const auto DRIVER_NAME = "OSVRARToolKit";
typedef std::shared_ptr<OSVRARToolKitController> OSVRARToolKitInstancePtr;

class OSVRARToolKitAnalysisPlugin {
public:
  // Constructor
  OSVRARToolKitAnalysisPlugin(OSVR_PluginRegContext ctx,
                              std::string const &name,
                              std::string const &input) {

    /// Create the initialization options
    OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

    // configure for TrackerInterface
    osvrDeviceTrackerConfigure(opts, &m_trackerOut);

    OSVR_DeviceToken dev;
    if (OSVR_RETURN_FAILURE ==
        osvrAnalysisSyncInit(ctx, name.c_str(), opts, &dev, &m_clientCtx)) {
      throw std::runtime_error("Could not initialize analysis plugin!");
    }
    m_dev = osvr::pluginkit::DeviceToken(dev);

    /// Send JSON descriptor
    m_dev.sendJsonDescriptor(osvr::util::makeString(org_osvr_artoolkit_json));

    /// Register update callback
    m_dev.registerUpdateCallback(this);

    /// Create our client interface and register a callback.
    if (OSVR_RETURN_FAILURE == osvrClientGetInterface(m_clientCtx,
                                                      input.c_str(),
                                                      &m_clientInterface)) {
      throw std::runtime_error(
          "Could not get client interface for analysis plugin!");
    }

    // Register imaging callback
    osvrRegisterImagingCallback(
        m_clientInterface, &OSVRARToolKitAnalysisPlugin::imagingCallback, this);

    // initialize ARToolKit for marker detection
    // images will be received from an imaging plugin
    m_arToolKitDevicePtr->initARToolKit(DRIVER_NAME);
  }

  ~OSVRARToolKitAnalysisPlugin() {
    /// Free the client interface so we don't end up getting called after
    /// destruction.
    osvrClientFreeInterface(m_clientCtx, m_clientInterface);

    // tell ARToolKit to cleanup
    m_arToolKitDevicePtr->cleanup();
  }

  OSVR_ReturnCode update() { return OSVR_RETURN_SUCCESS; }

  // Called when a new imaging report is available
  static void imagingCallback(void *userdata, const OSVR_TimeValue *timestamp,
                              const OSVR_ImagingReport *report) {
    auto &self = *static_cast<OSVRARToolKitAnalysisPlugin *>(userdata);
    self.handleData(*timestamp, *report);
  }

  /// Processes an imaging report.
  void handleData(OSVR_TimeValue const &timestamp,
                  OSVR_ImagingReport const &report) {

    // printImagingReport(timestamp, report);

    /* The first time, let's print some info. */
    if (reportNumber == 0) {
      //@todo set the ARToolKit paramters
      std::cout << "OSVRARToolKit got first report: image is "
                << report.state.metadata.width << " width and "
                << report.state.metadata.height << " height." << std::endl;
    }

    reportNumber++;
    // Marker detection
    m_arToolKitDevicePtr->DetectMarkers(timestamp, report);

    // if a marker was detected, sent a tracker pose report
    if (m_arToolKitDevicePtr->isMarkerDetected()) {
      OSVR_PoseState markerPose;
      Eigen::Matrix4d mat;

      // get the marker transform
      double **markerTransform[MATRIX_ROWS][MATRIX_COLS];
      for (int i = 0; i < MATRIX_ROWS; i++) {
        for (int j = 0; j < MATRIX_COLS; j++) {
          // copy the marker transform into an Eigen::Matrix4d, converting from
          // mm to m
          mat(i, j) = m_arToolKitDevicePtr->getMarkerTransform()[i][j] / 1000.0;
        }
      }

      // Turn an Eigen::Matrix4d (transform) into an OSVR_Pose3
      osvr::util::toPose(mat, markerPose);

      // send the tracker pose data
      osvrDeviceTrackerSendPose(m_dev, m_trackerOut, &markerPose, 0);
      printMarkerPoseTranslation(markerPose);
    }

    // free the image
    if (OSVR_RETURN_SUCCESS !=
        osvrClientFreeImage(m_clientCtx, report.state.data)) {
      std::cout << "Error, osvrClientFreeImage call failed." << std::endl;
    }
  }

private:
  osvr::pluginkit::DeviceToken m_dev;
  OSVR_TrackerDeviceInterface
      m_trackerOut; // tracker interface for detected AR marker
  OSVRARToolKitInstancePtr m_arToolKitDevicePtr;
  OSVR_ClientContext m_clientCtx;         // client context for analysis plugin
  OSVR_ClientInterface m_clientInterface; // client interface
  unsigned int reportNumber = 0;

  // prints an image report for debugging purposes
  void printImagingReport(OSVR_TimeValue const &timestamp,
                          OSVR_ImagingReport const &report) {
    std::cout << "timestamp = " << timestamp.seconds << std::endl;
    std::cout << "OSVR_ImageChannels = " << (int)report.state.metadata.channels
              << std::endl;
    std::cout << "OSVR_ImageDepth = " << (int)report.state.metadata.depth
              << std::endl;
    std::cout << "OSVR_ImageDimensions, (" << report.state.metadata.width
              << ", " << report.state.metadata.height << ")" << std::endl;
    std::cout << "OSVR_ImagingValueType = " << report.state.metadata.type
              << std::endl;
  }

  // print the marker pose translation
  void printMarkerPoseTranslation(OSVR_PoseState markerPose) {
    std::cout << "Marker Pose (" << markerPose.translation.data[0] << ", "
              << markerPose.translation.data[1] << ", "
              << markerPose.translation.data[2] << ")" << std::endl;
  }
};

class AnalysisPluginInstantiation {
public:
  AnalysisPluginInstantiation() : arToolkit(new OSVRARToolKitController()) {}
  OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
    Json::Value root;
    {
      Json::Reader reader;
      if (!reader.parse(params, root)) {
        std::cerr << "Couldn't parse JSON for ARToolKit analysis plugin!"
                  << std::endl;
        return OSVR_RETURN_FAILURE;
      }
    }

    // required input for the analysis plugin, specified in the server config
    // we'll want to use an imaging path so we can get image reports from
    // another plugin, such as OpenCV
    // ex. "input": "/camera"
    auto input = root["input"].asString();

    // optional
    auto deviceName = root.get("name", DRIVER_NAME).asString();

    std::cout << "OSVRARToolKit PLUGIN: Got a hardware detection request"
              << std::endl;
    if (m_found) {
      return OSVR_RETURN_SUCCESS;
    }
    m_found = true;
    std::cout << "OSVRARToolKit PLUGIN initializing" << std::endl;

    osvr::pluginkit::PluginContext context(ctx);
    /// Create our device object
    context.registerObjectForDeletion(
        new OSVRARToolKitAnalysisPlugin(ctx, DRIVER_NAME, input));

    return OSVR_RETURN_SUCCESS;
  }

private:
  OSVRARToolKitInstancePtr arToolkit;
  bool m_found = false;
};
} // namespace

OSVR_PLUGIN(org_osvr_artoolkit) {
  osvr::pluginkit::PluginContext context(ctx);

  /// Register a detection callback function object.
  context.registerDriverInstantiationCallback(
      DRIVER_NAME, new AnalysisPluginInstantiation());

  return OSVR_RETURN_SUCCESS;
}
