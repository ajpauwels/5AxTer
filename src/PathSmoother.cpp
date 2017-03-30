#include "PathSmoother.hpp"
#include <cmath>
#include <float.h>
#include "ForwardKinematics.hpp"
#include "InverseKinematics.hpp"
#include "../lib/SD/SD.h"

File gFile;
const int chipSelect = BUILTIN_SDCARD;

PathSmoother::PathSmoother(char* gcodeFilePath) {
	Matrix3x1 S;
	float x_dot_buildplate = 0;
	float y_dot_buildplate = 0;
	float z_dot_buildplate = 0;
	float theta_dot = 0;
	float phi_dot = 0;
	float x_buildplate = 0;
	float y_buildplate = 0;
	float z_buildplate = 0;
	float x_frame = 0;
	float y_frame = 0;
	float z_frame = 8;
	float rho = 0;
	float theta = 0;
	float phi = 0;
	float psi = 0;
	float z_offset = 4;
	ForwardKinematics fk = ForwardKinematics();

	// Compute the forward position kinematics and create the S matrix for calibration
	Matrix3x1 forwardPosMatrix = fk.position(x_frame, y_frame, z_frame, x_buildplate, y_buildplate,
		z_buildplate, rho, theta, phi, psi, z_offset);
	S.m[0][0] = forwardPosMatrix.m[0][0];
	S.m[1][0] = forwardPosMatrix.m[1][0];
	S.m[2][0] = forwardPosMatrix.m[2][0];

	// Compute the forward velocity kinematics
	Matrix3x8 forwardVelMatrix = fk.velocity_jacobian(x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi,
		z_offset);

	// Compute the forward acceleration kinematics
	Matrix3x8 forwardAccMatrix = fk.acceleration_jacobian(x_dot_buildplate, y_dot_buildplate, z_dot_buildplate, theta_dot,
		phi_dot, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);

	// logAlways("Forward position kinematics:\n");
	// logAlways("[ %f\n  %f\n  %f ]\n\n", forwardPosMatrix[0][0], forwardPosMatrix[1][0], forwardPosMatrix[2][0]);
	//
	// logAlways("Forward velocity kinematics:\n");
	// logAlways("[ %f %f %f %f %f %f %f %f\n"
 // 			  "  %f %f %f %f %f %f %f %f\n"
	// 		  "  %f %f %f %f %f %f %f %f ]\n\n", forwardVelMatrix[0][0], forwardVelMatrix[0][1], forwardVelMatrix[0][2],
	// 	forwardVelMatrix[0][3], forwardVelMatrix[0][4], forwardVelMatrix[0][5], forwardVelMatrix[0][6],
	// 	forwardVelMatrix[0][7], forwardVelMatrix[1][0], forwardVelMatrix[1][1], forwardVelMatrix[1][2],
	// 	forwardVelMatrix[1][3], forwardVelMatrix[1][4], forwardVelMatrix[1][5], forwardVelMatrix[1][6],
	// 	forwardVelMatrix[1][7], forwardVelMatrix[2][0], forwardVelMatrix[2][1], forwardVelMatrix[2][2],
	// 	forwardVelMatrix[2][3], forwardVelMatrix[2][4], forwardVelMatrix[2][5], forwardVelMatrix[2][6],
	// 	forwardVelMatrix[2][7]);
	//
	// logAlways("Forward acceleration kinematics:\n");
	// logAlways("[ %f %f %f %f %f %f %f %f\n"
  // 			  "  %f %f %f %f %f %f %f %f\n"
	// 		  "  %f %f %f %f %f %f %f %f ]\n\n", forwardAccMatrix[0][0], forwardAccMatrix[0][1], forwardAccMatrix[0][2],
	// 	forwardAccMatrix[0][3], forwardAccMatrix[0][4], forwardAccMatrix[0][5], forwardAccMatrix[0][6],
	// 	forwardAccMatrix[0][7], forwardAccMatrix[1][0], forwardAccMatrix[1][1], forwardAccMatrix[1][2],
	// 	forwardAccMatrix[1][3], forwardAccMatrix[1][4], forwardAccMatrix[1][5], forwardAccMatrix[1][6],
	// 	forwardAccMatrix[1][7], forwardAccMatrix[2][0], forwardAccMatrix[2][1], forwardAccMatrix[2][2],
	// 	forwardAccMatrix[2][3], forwardAccMatrix[2][4], forwardAccMatrix[2][5], forwardAccMatrix[2][6],
	// 	forwardAccMatrix[2][7]);

	z_buildplate = 4;
	theta = M_PI / 2;
	InverseKinematics ik = InverseKinematics();
	Matrix3x1 inversePosMatrix = ik.position_frame(S, x_buildplate, y_buildplate, z_buildplate, rho, theta, phi, psi, z_offset);

	// logAlways("Inverse position kinematics:\n");
	// logAlways("[ %f\n  %f\n  %f ]\n", inversePosMatrix[0][0], inversePosMatrix[1][0], inversePosMatrix[2][0]);

	// Set the initial value of x/y/z point to be 0, 0, 0
	lastX = lastY = lastZ = 0;

	// Set the initial value of the current position to home values
	currX = X_HOME;
	currY = Y_HOME;
	currZ = Z_HOME;

	// Open the GCode file
	if (!SD.begin(chipSelect)) {
		Serial.println("[ERROR] SD initialization failed");
		return;
	}
	Serial.println("[INFO] Initialized SD lib");

	gFile = SD.open(gcodeFilePath);

	// Check we successfully opened
	if (gFile) {
		// Process each line individually
		while (gFile.available()) {
			// Get the next char
			char nextChar = (char)gFile.read();

			// Check to see what type of line it is
			switch (nextChar) {
				// Comment, used to separate GCode layers
				case ';': {
					// Store the "LAYER:" character sequence (if it is there)
					char line[6] = { 0, 0, 0, 0, 0, 0 };

					// Read "LAYER:" or up to \r or \n
					nextChar = (char)gFile.read();
					for (unsigned int idx = 0; idx < 6 && nextChar != '\n' && nextChar != '\r'; ++idx) {
						line[idx] = nextChar;
						nextChar = (char)gFile.read();
					}

					// Check if this is a "LAYER:" comment
					if (strncmp(line, "LAYER:", 6) == 0) {
						int layer_nr = 0;

						// Get the layer number
						nextChar = (char)gFile.read();
						for (unsigned int idx = 0; nextChar != '\n' && nextChar != '\r'; ++idx) {
							layer_nr = layer_nr * 10 + (int)(nextChar - '0');
							nextChar = (char)gFile.read();
						}

						// Process the latest completed layer
						if (layer_nr > 0) {
							processLayer();
							// layerCommands.clear();
							// getchar();
						}
					}
					break;
				}

				// M-command
				case 'M':
					break;

				// G-command
				case 'G':
					processGCommand(line);
					break;

				default:
					break;
			}
		}

		// Close the file handler
		file.close();
	} else {
		Serial.println("[ERROR] Could not open GCode file");
	}
}

/**
 * Loads the specified number of GCode commands from a file on
 * the SD card to the given array. Make sure the number given
 * is less than or equal to the size of the array.
 *
 * @param num The number of commands to place in the buffer
 * @param buf The buffer array
 */
void PathSmoother::bufferCommands(unsigned int num, GCommand* buf) {

}

void PathSmoother::getPoint(uint64_t buf_idx, uint64_t spline_idx) {
	FPoint3 maxAccel_vect = FPoint3(MAX_ACCEL/2, MAX_ACCEL/2, MAX_ACCEL/2);
}

void PathSmoother::processLayer() {
	// Get the shortest distance from the first 15 path segments
	FPoint3 maxAccel_vect = FPoint3(MAX_ACCEL/2, MAX_ACCEL/2, MAX_ACCEL/2);
	float maxFeedrate = findMinFeedrate(layerPoints, 0, 15, maxAccel_vect);
	FPoint3 initialVel = FPoint3(0, 0, 0);

	// Iterate through each point on the path
	for (unsigned int point_idx = 0; point_idx < layerPoints.size() - 2; ++point_idx) {
		FPoint3 pi = layerPoints[point_idx];
		FPoint3 pm = layerPoints[point_idx + 1];
		FPoint3 pf = layerPoints[point_idx + 2];

		float firstSegmentDistance = (pm - pi).vSize();
		float vf_x = maxFeedrate * (pm.x - pi.x) / firstSegmentDistance;
		float vf_y = maxFeedrate * (pm.y - pi.y) / firstSegmentDistance;
		float vf_z = maxFeedrate * (pm.z - pi.z) / firstSegmentDistance;
		float vm_x = sqrt((maxAccel_vect.x * (pm.x - pi.x) / 2) + (pow(initialVel.x, 2) / 2) + (pow(vf_x, 2) / 2));
		float vm_y = sqrt((maxAccel_vect.y * (pm.y - pi.y) / 2) + (pow(initialVel.y, 2) / 2) + (pow(vf_y, 2) / 2));
		float vm_z = sqrt((maxAccel_vect.z * (pm.z - pi.z) / 2) + (pow(initialVel.z, 2) / 2) + (pow(vf_z, 2) / 2));
		FPoint3 maxInputFeedrate = FPoint3(vm_x, vm_y, vm_z);

		// logAlways("maxFeedrate = %f\n", maxFeedrate);
		// logAlways("maxInputFeedrate = <%f, %f, %f>\n", maxInputFeedrate.x, maxInputFeedrate.y, maxInputFeedrate.z);

		connectPoints(pi, pm, pf, maxFeedrate);
	}
}

void PathSmoother::connectPoints(FPoint3 p1, FPoint3 p2, FPoint3 p3, float feedrate) {
	float line1_mag, line2_mag, delta, t_d, angle;
	unsigned int ratio;
	FPoint3 lineSegment1, lineSegment1Normalized, lineSegment2, lineSegment2Normalized;
	FPoint3 splinePoints[ratio];

	lineSegment1 = (p2 - p1);
	lineSegment2 = (p3 - p2);

	line1_mag = lineSegment1.vSize();
	line2_mag = lineSegment2.vSize();
	delta = (line1_mag < line2_mag) ? line1_mag / 2 : line2_mag / 2;

	lineSegment1Normalized = lineSegment1.normalized();
	lineSegment2Normalized = lineSegment2.normalized();

	t_d = 2 * delta / feedrate;
	ratio = ceil(t_d * CONTROL_LOOP_FREQ);
	angle = acos((lineSegment1Normalized.x * lineSegment2Normalized.x + lineSegment1Normalized.y * lineSegment2Normalized.y + lineSegment1Normalized.z * lineSegment2Normalized.z));

	if (delta > 0 && angle > MAX_ANGLE) {
		FPoint3 splineEndPoints[2];
		FPoint3 splineHalfwayPoint;

		splineEndPoints[0] = p2 - lineSegment1Normalized * delta;
		splineEndPoints[1] = p2 + lineSegment2Normalized * delta;

		for (unsigned int step_idx = 0; step_idx < ratio; ++step_idx) {
			float t = step_idx / (ratio - 1);
			FPoint3 nextPt;

			nextPt.x = splineEndPoints[0].x * (1 - t) * (1 - t) + p2.x * 2 * (t - t * t) + splineEndPoints[1].x * t * t;
			nextPt.y = splineEndPoints[0].y * (1 - t) * (1 - t) + p2.y * 2 * (t - t * t) + splineEndPoints[1].y * t * t;
			nextPt.z = splineEndPoints[0].z * (1 - t) * (1 - t) + p2.z * 2 * (t - t * t) + splineEndPoints[1].z * t * t;

			splinePoints[step_idx] = nextPt;

			if (step_idx == floor(ratio / 2) - 1) {
				splineHalfwayPoint = nextPt;
			}
		}

		float chord_error = (p2 - splineHalfwayPoint).vSize();
	} else {

	}
}

FPoint3 PathSmoother::computeFeedrate(FPoint3 s1, FPoint3 s2, FPoint3 accelProfile, float delta) {
	float s1size = s1.vSize();
	float s2size = s2.vSize();

	float feedrate_x = abs(accelProfile.x * delta * s1size * s2size / (s2size * (-s1.x) - s1size * s2.x));
	float feedrate_y = abs(accelProfile.y * delta * s1size * s2size / (s2size * (-s1.y) - s1size * s2.y));
	float feedrate_z = abs(accelProfile.z * delta * s1size * s2size / (s2size * (-s1.z) - s1size * s2.z));

	return FPoint3(2 * sqrt(feedrate_x), 2 * sqrt(feedrate_y), 2 * sqrt(feedrate_z));
}

float PathSmoother::findMinFeedrate(FPoint3* points, unsigned int start_idx, unsigned int end_idx, FPoint3 accelProfile) {
	// If the indices are the same, no distance
	if (start_idx == end_idx) return 0.0;
	if (start_idx - end_idx < 2) return MAX_FEEDRATE;

	float minCornerFeedrate = FLT_MAX;
	float maxOutputFeedrate = FLT_MAX;

	// Find the shortest segment between start_idx and end_idx
	for (unsigned int point_idx = start_idx; point_idx <= end_idx - 2; ++point_idx) {
		FPoint3 xi = points[point_idx];
		FPoint3 xm = points[point_idx + 1];
		FPoint3 xf = points[point_idx + 2];

		float minDelta;
		float delta_1 = 0.5 * (xm - xi).vSize();
		float delta_2 = 0.5 * (xf - xm).vSize();
		minDelta = (delta_1 < delta_2) ? delta_1 : delta_2;

		FPoint3 feedrate = computeFeedrate(xm - xi, xf - xm, accelProfile, minDelta);
		float smallestComponent = chooseSmallestNonZeroComponent(feedrate);

		if (smallestComponent < minCornerFeedrate) minCornerFeedrate = smallestComponent;
	}

	FPoint3 lastSegment = points[end_idx - 1] - points[end_idx - 2];
	float lastSegmentSize = lastSegment.vSize();
	FPoint3 maxFinalVel = FPoint3(
		sqrt(accelProfile.x * lastSegment.x) * lastSegmentSize / lastSegment.x,
		sqrt(accelProfile.y * lastSegment.y) * lastSegmentSize / lastSegment.y,
		sqrt(accelProfile.z * lastSegment.z) * lastSegmentSize / lastSegment.z);
	maxOutputFeedrate = chooseSmallestNonZeroComponent(maxFinalVel);

	if (minCornerFeedrate < maxOutputFeedrate) return minCornerFeedrate;
	else return maxOutputFeedrate;
}

float PathSmoother::chooseSmallestNonZeroComponent(FPoint3 fp) {
	float smallest = FLT_MAX;

	if (fp.x != 0 && fp.x < smallest) smallest = fp.x;
	else if (fp.y != 0 && fp.y < smallest) smallest = fp.y;
	else if (fp.z != 0 && fp.z < smallest) smallest = fp.z;

	return smallest;
}

FPoint3 PathSmoother::pointFromGCommand(std::shared_ptr<GCommand> comm) {
	FPoint3 point;
	switch (comm->getType()) {
		// G0 command, point is specified XYZ point
		case 0: {
			std::shared_ptr<G0> g0comm = std::static_pointer_cast<G0>(comm);

			point = FPoint3(g0comm->X(), g0comm->Y(), g0comm->Z());
			break;
		}
		// G1 command, point is specified XYZ point
		case 1: {
			std::shared_ptr<G1> g1comm = std::static_pointer_cast<G1>(comm);

			point = FPoint3(g1comm->X(), g1comm->Y(), g1comm->Z());
			break;
		}
		// G28 command, point is the specified HOME values
		case 28: {
			std::shared_ptr<G28> g28comm = std::static_pointer_cast<G28>(comm);

			point = FPoint3(X_HOME, Y_HOME, Z_HOME);
			break;
		}

		default:
			// logAlways("[ERROR] Could not identify GCode command with type %d\n", comm->getType());
			point = FPoint3(0, 0, 0);
	}

	return point;
}

void PathSmoother::processGCommand(std::string command) {
	// Split the command on spaces
	std::vector<std::string> tokens = split(command, ' ');

	// Get the GCode command number
	int commandNr = std::stoi(tokens[0].substr(1));

	// Handle the various GCode command types
	switch (commandNr) {
		// G0 & G1
		case 0:
		case 1:
			// Error check
			if (tokens.size() <= 1) return;

			// Indicates which parameters were provided
			bool hadX, hadY, hadZ;

			// Stores the parameter values
			double x, y, z;

			// Assume no parameters given
			hadX = hadY = hadZ = false;

			// Set the parameter values to the last values, we'll update these next
			x = lastX;
			y = lastY;
			z = lastZ;

			// Iterate through each parameter
			for (unsigned int token_idx = 1; token_idx < tokens.size(); ++token_idx) {
				// Retrieve the parameter
				std::string token = tokens[token_idx];

				// Check to see the parameter type and update accordingly
				char param = token[0];
				if (param == 'X') {
					hadX = true;
					x = std::stod(token.substr(1));
				}
				else if (param == 'Y') {
					hadY = true;
					y = std::stod(token.substr(1));
				}
				else if (param == 'Z') {
					hadZ = true;
					z = std::stod(token.substr(1));
				}
			}

			// If the G-command updated either the x, y, or z-position, add it in
			if (hadX || hadY || hadZ) {
				if (commandNr == 0) {
					auto command = std::make_shared<G0>(x, y, z);
					layerCommands.push_back(command);
				} else {
					auto command = std::make_shared<G1>(x, y, z);
					layerCommands.push_back(command);
				}

				layerPoints.push_back(FPoint3(x, y, z));

				// Update the last x, y, and z values to be the current values
				lastX = x;
				lastY = y;
				lastZ = z;
			}
			break;

		// G28
		case 28: {
			// Indicates which axes should be homed
			bool x, y, z;

			// If no axes specified, home all axes
			if (tokens.size() == 1) {
				x = y = z = true;
			}
			// Otherwise determine which were specified
			else {
				x = y = z = false;

				for (unsigned int idx = 0; idx < tokens.size(); ++idx) {
					std::string token = tokens[idx];

					if (token.compare("X") == 0) {
						x = true;
					}
					else if (token.compare("Y") == 0) {
						y = true;
					}
					else if (token.compare("Z") == 0) {
						z = true;
					}
				}
			}

			// Add the command to the list
			auto command = std::make_shared<G28>(x, y, z);
			layerCommands.push_back(command);
			layerPoints.push_back(FPoint3(x, y, z));
			break;
		}

		// Everything else
		default:
			break;
	}
}
