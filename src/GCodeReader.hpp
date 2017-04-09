#ifndef GCODE_READER_HPP
#define GCODE_READER_HPP

#include "../lib/SD/SD.h"
#include "GCommand.hpp"
#include "settings.h"
#include "Buffer.hpp"
#undef F

enum READ_STATE {
  SEEK,
  NUMBER,
  PARAMS
};

class GCodeReader {
private:
  GCommand newCommand;
  bool commandDone;
  bool newFile;
  READ_STATE state;
  unsigned int param_idx;
  unsigned int commandNr;

  Buffer<char, FILE_CHAR_BUFFER_SIZE> fileBuf;
  File gFile;

  void fillBuffer() {
    while (!fileBuf.isFull() && gFile.available()) {
      char c = gFile.read();
      fileBuf.push(c);
    }
  }

public:
  GCodeReader() {
    commandDone = false;
    state = SEEK;
    newFile = false;
  }

  void loadFile(File _gFile) {
    gFile = _gFile;
    fillBuffer();

    if (fileBuf.peek() == 'G') {
      fileBuf.pop();
      state = NUMBER;
    } else {
      state = SEEK;
    }
    newFile = true;
  }

  bool newCommandAvailable() {
    return commandDone;
  }

  GCommand getNewCommand() {
    commandDone = false;
    return newCommand;
  }

  void read() {
    // If there's nothing in our buffer or if there isn't enough characters
    // to seek, try to fill the buffer and return
    if (fileBuf.isEmpty() || (fileBuf.getSize() < 2 && state == SEEK)) {
      fillBuffer();
      return;
    }

    switch (state) {
      case SEEK: {
        char chars[2];
        chars[0] = fileBuf.pop();
        chars[1] = fileBuf.peek();
        for (unsigned int idx = 0; idx < 1; ++idx) {
          if (chars[0] == '\n' && chars[1] == 'G') {
            fileBuf.pop();
            commandNr = 0;
            // state = NUMBER;
            break;
          }
          chars[0] = fileBuf.pop();
          if (fileBuf.isEmpty()) {
            fileBuf.push(chars[0]);
            break;
          }
          chars[1] = fileBuf.peek();
        }
        commandDone = false;

        break;
      }

      case NUMBER: {
        char nextChar = fileBuf.peek();
        while (nextChar != ' ' && nextChar != '\n') {
          commandNr = 10 * commandNr + (nextChar - '0');

          fileBuf.pop();
          if (fileBuf.isEmpty()) {
            break;
          }
          nextChar = fileBuf.peek();
        }

        if (nextChar == ' ') {
          newCommand = GCommand(commandNr);
          param_idx = 0;
          fileBuf.pop();
          state = PARAMS;
        }
        else if (nextChar == '\n') {
          newCommand = GCommand(commandNr);
          newCommand.paramString[0] = '\0';
          commandDone = true;
          state = SEEK;
        }
        break;
      }

      case PARAMS: {
        char nextChar = fileBuf.peek();

        for (; param_idx < GCODE_MAX_LINE_WIDTH && nextChar != '\n'; ++param_idx) {
            newCommand.paramString[param_idx] = nextChar;
            fileBuf.pop();
            if (fileBuf.isEmpty()) {
              ++param_idx;
              break;
            }
            nextChar = fileBuf.peek();
        }

        if (nextChar == '\n') {
          newCommand.paramString[param_idx] = '\0';
          commandDone = true;
          state = SEEK;
        }
        break;
      }
    }
  }

  static void commandToSerial(GCommand comm) {
    Serial.print('G');
		Serial.print(comm.getType());
		Serial.print(' ');
		switch (comm.getType()) {
			case 0: {
				G0 g0 = GCodeReader::toG0(comm);

				if (g0.hasXParam()) {
					Serial.print('X');
					Serial.print(g0.X(), 3);
					Serial.print(' ');
				}
				if (g0.hasYParam()) {
					Serial.print('Y');
					Serial.print(g0.Y(), 3);
					Serial.print(' ');
				}
				if (g0.hasZParam()) {
					Serial.print('Z');
					Serial.print(g0.Z(), 3);
					Serial.print(' ');
				}
				if (g0.hasAParam()) {
					Serial.print('A');
					Serial.print(g0.A(), 3);
					Serial.print(' ');
				}
				if (g0.hasBParam()) {
					Serial.print('B');
					Serial.print(g0.B(), 3);
					Serial.print(' ');
				}
        if (g0.hasEParam()) {
          Serial.print('E');
          Serial.print(g0.E(), 5);
          Serial.print(' ');
        }
        if (g0.hasFParam()) {
          Serial.print('F');
          Serial.print(g0.F(), 3);
          Serial.print(' ');
        }
				break;
			}

			case 1: {
				G1 g1 = GCodeReader::toG1(comm);

				if (g1.hasXParam()) {
					Serial.print('X');
					Serial.print(g1.X(), 3);
					Serial.print(' ');
				}
				if (g1.hasYParam()) {
					Serial.print('Y');
					Serial.print(g1.Y(), 3);
					Serial.print(' ');
				}
				if (g1.hasZParam()) {
					Serial.print('Z');
					Serial.print(g1.Z(), 3);
					Serial.print(' ');
				}
				if (g1.hasAParam()) {
					Serial.print('A');
					Serial.print(g1.A(), 3);
					Serial.print(' ');
				}
				if (g1.hasBParam()) {
					Serial.print('B');
					Serial.print(g1.B(), 3);
					Serial.print(' ');
				}
        if (g1.hasEParam()) {
          Serial.print('E');
          Serial.print(g1.E(), 5);
          Serial.print(' ');
        }
        if (g1.hasFParam()) {
          Serial.print('F');
          Serial.print(g1.F(), 3);
          Serial.print(' ');
        }
				break;
			}

			case 28: {
				G28 g28 = GCodeReader::toG28(comm);

				if (g28.X()) {
					Serial.print('X');
					Serial.print(' ');
				}
				if (g28.Y()) {
					Serial.print('Y');
					Serial.print(' ');
				}
				if (g28.Z()) {
					Serial.print('Z');
					Serial.print(' ');
				}
				if (g28.A()) {
					Serial.print('A');
					Serial.print(' ');
				}
				if (g28.B()) {
					Serial.print('B');
					Serial.print(' ');
				}
				break;
			}
		}

    Serial.println();
	}

  static G0 toG0(GCommand comm) {
    G0 G0comm = G0();

    char* nextChar = comm.paramString;
    float val;
    while (*nextChar != '\0') {
      switch (*nextChar) {
        case 'X':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setX(val);
          break;

        case 'Y':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setY(val);
          break;

        case 'Z':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setZ(val);
          break;

        case 'A':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setA(val);
          break;

        case 'B':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setB(val);
          break;

        case 'E':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setE(val);
          break;

        case 'F':
          val = strtof(nextChar + 1, &nextChar);
          G0comm.setF(val);
          break;

        // If the character is a space, just move on to the next parameter
        default:
          ++nextChar;
      }
    }

    return G0comm;
  }

  static G1 toG1(GCommand comm) {
    G1 G1comm = G1();

    char* nextChar = comm.paramString;
    float val;
    while (*nextChar != '\0') {
      switch (*nextChar) {
        case 'X':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setX(val);
          break;

        case 'Y':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setY(val);
          break;

        case 'Z':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setZ(val);
          break;

        case 'A':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setA(val);
          break;

        case 'B':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setB(val);
          break;

        case 'E':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setE(val);
          break;

        case 'F':
          val = strtof(nextChar + 1, &nextChar);
          G1comm.setF(val);
          break;

        // If the character is a space, just move on to the next parameter
        default:
          ++nextChar;
      }
    }

    return G1comm;
  }

  static G28 toG28(GCommand comm) {
    G28 G28comm = G28();

    char* nextChar = comm.paramString;

    while (*nextChar != '\0') {
      switch (*nextChar) {
        case 'X':
          G28comm.setX(true);
          break;

        case 'Y':
          G28comm.setY(true);
          break;

        case 'Z':
          G28comm.setZ(true);
          break;

        case 'A':
          G28comm.setA(true);
          break;

        case 'B':
          G28comm.setB(true);
          break;
      }
      nextChar++;
    }
    return G28comm;
  }
};

#endif
