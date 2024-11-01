
"use strict";

let GetWaypoints = require('./GetWaypoints.js')
let ConvertPixelWaypointsToMap = require('./ConvertPixelWaypointsToMap.js')
let GetAmclPose = require('./GetAmclPose.js')
let InitiateCoveragePath = require('./InitiateCoveragePath.js')
let GetPixelPose = require('./GetPixelPose.js')
let ConvertPixelToMap = require('./ConvertPixelToMap.js')

module.exports = {
  GetWaypoints: GetWaypoints,
  ConvertPixelWaypointsToMap: ConvertPixelWaypointsToMap,
  GetAmclPose: GetAmclPose,
  InitiateCoveragePath: InitiateCoveragePath,
  GetPixelPose: GetPixelPose,
  ConvertPixelToMap: ConvertPixelToMap,
};
