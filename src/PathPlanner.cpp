#include "PathPlanner.hpp"
#include "GCommand.hpp"

void PathPlanner::update() {
  if (gReader.newCommandAvailable() && !gCommandBuff.isFull()) {
    GCommand comm = gReader.getNewCommand();
    // GCodeReader::commandToSerial(comm);
    // gCommandBuff.push(gReader.getNewCommand());
  }

  gReader.read();
}

// FPoint3 PathPlanner::computeFeedrate(FPoint3 s1, FPoint3 s2, FPoint3 accelProfile, float delta) {
// 	float s1size = s1.vSize();
// 	float s2size = s2.vSize();
//
// 	float feedrate_x = abs(accelProfile.x * delta * s1size * s2size / (s2size * (-s1.x) - s1size * s2.x));
// 	float feedrate_y = abs(accelProfile.y * delta * s1size * s2size / (s2size * (-s1.y) - s1size * s2.y));
// 	float feedrate_z = abs(accelProfile.z * delta * s1size * s2size / (s2size * (-s1.z) - s1size * s2.z));
//
// 	return FPoint3(2 * sqrt(feedrate_x), 2 * sqrt(feedrate_y), 2 * sqrt(feedrate_z));
// }
//
// float PathPlanner::findMinFeedrate(FPoint3* points, unsigned int start_idx, unsigned int end_idx, FPoint3 accelProfile) {
// 	// If the indices are the same, no distance
// 	if (start_idx == end_idx) return 0.0;
// 	if (start_idx - end_idx < 2) return MAX_FEEDRATE;
//
// 	float minCornerFeedrate = FLT_MAX;
// 	float maxOutputFeedrate = FLT_MAX;
//
// 	// Find the shortest segment between start_idx and end_idx
// 	for (unsigned int point_idx = start_idx; point_idx <= end_idx - 2; ++point_idx) {
// 		FPoint3 xi = points[point_idx];
// 		FPoint3 xm = points[point_idx + 1];
// 		FPoint3 xf = points[point_idx + 2];
//
// 		float minDelta;
// 		float delta_1 = 0.5 * (xm - xi).vSize();
// 		float delta_2 = 0.5 * (xf - xm).vSize();
// 		minDelta = (delta_1 < delta_2) ? delta_1 : delta_2;
//
// 		FPoint3 feedrate = computeFeedrate(xm - xi, xf - xm, accelProfile, minDelta);
// 		float smallestComponent = chooseSmallestNonZeroComponent(feedrate);
//
// 		if (smallestComponent < minCornerFeedrate) minCornerFeedrate = smallestComponent;
// 	}
//
// 	FPoint3 lastSegment = points[end_idx - 1] - points[end_idx - 2];
// 	float lastSegmentSize = lastSegment.vSize();
// 	FPoint3 maxFinalVel = FPoint3(
// 		sqrt(accelProfile.x * lastSegment.x) * lastSegmentSize / lastSegment.x,
// 		sqrt(accelProfile.y * lastSegment.y) * lastSegmentSize / lastSegment.y,
// 		sqrt(accelProfile.z * lastSegment.z) * lastSegmentSize / lastSegment.z);
// 	maxOutputFeedrate = chooseSmallestNonZeroComponent(maxFinalVel);
//
// 	if (minCornerFeedrate < maxOutputFeedrate) return minCornerFeedrate;
// 	else return maxOutputFeedrate;
// }
//
// float PathPlanner::chooseSmallestNonZeroComponent(FPoint3 fp) {
// 	float smallest = FLT_MAX;
//
// 	if (fp.x != 0 && fp.x < smallest) smallest = fp.x;
// 	else if (fp.y != 0 && fp.y < smallest) smallest = fp.y;
// 	else if (fp.z != 0 && fp.z < smallest) smallest = fp.z;
//
// 	return smallest;
// }

FPoint3 PathPlanner::bezier(FPoint3 p1, FPoint3 p2, FPoint3 p3, long int t) {
    FPoint3 newPt;
		newPt.x = p1.x * (1 - t) * (1 - t) + p2.x * 2 * (t - t * t) + p2.x * t * t;
		newPt.y = p1.y * (1 - t) * (1 - t) + p2.y * 2 * (t - t * t) + p2.y * t * t;
		newPt.z = p1.z * (1 - t) * (1 - t) + p2.z * 2 * (t - t * t) + p2.z * t * t;

    return newPt;
}

bool PathPlanner::loadFile(char const* gcodeFilePath) {
  // Open the GCode file
  if (!SD.begin(chipSelect)) {
    #ifdef DEBUG
    Serial.println("[ERROR] SD initialization failed");
    #endif
    return false;
  }
  #ifdef DEBUG
  Serial.println("[INFO] Initialized SD lib");
  #endif

  gFile = SD.open(gcodeFilePath);

  if (!gFile) {
    #ifdef DEBUG
    Serial.print("[ERROR] Could not open GCode file: ");
    Serial.println(gcodeFilePath);
    #endif
    return false;
  }
  #ifdef DEBUG
  Serial.print("[INFO] Successfully opened GCode file: ");
  Serial.println(gcodeFilePath);
  #endif

  gReader.loadFile(gFile);

  return true;
}
