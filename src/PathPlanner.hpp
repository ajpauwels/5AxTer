#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "gcommand.hpp"
#include "settings.h"
#include <SD.h>
#include "utility.hpp"
#include "GCodeReader.hpp"
#include "Buffer.hpp"

class PathPlanner {
private:
  // Buffers
  Buffer<GCommand, GCODE_COMMAND_BUFFER_SIZE> gCommandBuff;
  Buffer<Point3, PATH_POINTS_BUFFER_SIZE> pathPointsBuff;

  // SD card and file vars
  File gFile;
  const int chipSelect = BUILTIN_SDCARD;

  // Interface to the GCode file
  GCodeReader gReader;

  // Path smoothing parameters
  float maxFeedrate = 0.0;

  float chooseSmallestNonZeroComponent(FPoint3 fp);
  float findMinFeedrate(FPoint3* points, unsigned int start_idx, unsigned int end_idx, FPoint3 accelProfile);
  FPoint3 computeFeedrate(FPoint3 s1, FPoint3 s2, FPoint3 accelProfile, float delta);
  FPoint3 bezier(FPoint3 p1, FPoint3 p2, FPoint3 p3, long int idx);

public:
  bool loadFile(char const* gcodeFilePath);
  void update();
};

#endif
