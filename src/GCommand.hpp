#ifndef GCOMMAND_HPP
#define GCOMMAND_HPP

#include <math.h>
#include "settings.h"
#undef F

class GCommand {
private:
	int type;

public:
	char paramString[GCODE_MAX_LINE_WIDTH + 1];

	GCommand() { GCommand(-1); }
	GCommand(int _type) : type(_type) {}
	int getType() { return type; }
};

class G0 : public GCommand {
private:
	bool hasX, hasY, hasZ, hasA, hasB, hasE, hasF;
	float x, y, z, a, b, e, f;

public:
	G0() : GCommand(0) {
		hasX = hasY = hasZ = hasA = hasB = hasE = hasF = false;
	}

	float X() { return x; }
	float Y() { return y; }
	float Z() { return z; }
	float A() { return a; }
	float B() { return b; }
	float E() { return e; }
	float F() { return f; }

	bool hasXParam() { return hasX; }
	bool hasYParam() { return hasY; }
	bool hasZParam() { return hasZ; }
	bool hasAParam() { return hasA; }
	bool hasBParam() { return hasB; }
	bool hasEParam() { return hasE; }
	bool hasFParam() { return hasF; }

	void setX(float _x) {
		hasX = true;
		x = _x;
	}
	void unsetX() {
		hasX = false;
	}

	void setY(float _y) {
		hasY = true;
		y = _y;
	}
	void unsetY() {
		hasY = false;
	}

	void setZ(float _z) {
		hasZ = true;
		z = _z;
	}
	void unsetZ() {
		hasZ = false;
	}

	void setA(float _a) {
		hasA = true;
	 	a = _a;
	}
	void unsetA() {
		hasA = false;
	}

	void setB(float _b) {
		hasB = true;
		b = _b;
	}
	void unsetB() {
		hasB = false;
	}

	void setE(float _e) {
		hasE = true;
		e = _e;
	}
	void unsetE() {
		hasE = false;
	}

	void setF(float _f) {
		hasF = true;
		f = _f;
	}
	void unsetF(float _f) {
		hasF = false;
	}
};

class G1 : public GCommand {
private:
	bool hasX, hasY, hasZ, hasA, hasB, hasE, hasF;
	float x, y, z, a, b, e, f;

public:
	G1() : GCommand(1) {
		hasX = hasY = hasZ = hasA = hasB = hasE = hasF = false;
	}

	float X() { return x; }
	float Y() { return y; }
	float Z() { return z; }
	float A() { return a; }
	float B() { return b; }
	float E() { return e; }
	float F() { return f; }

	bool hasXParam() { return hasX; }
	bool hasYParam() { return hasY; }
	bool hasZParam() { return hasZ; }
	bool hasAParam() { return hasA; }
	bool hasBParam() { return hasB; }
	bool hasEParam() { return hasE; }
	bool hasFParam() { return hasF; }

	void setX(float _x) {
		hasX = true;
		x = _x;
	}
	void unsetX() {
		hasX = false;
	}

	void setY(float _y) {
		hasY = true;
		y = _y;
	}
	void unsetY() {
		hasY = false;
	}

	void setZ(float _z) {
		hasZ = true;
		z = _z;
	}
	void unsetZ() {
		hasZ = false;
	}

	void setA(float _a) {
		hasA = true;
	 	a = _a;
	}
	void unsetA() {
		hasA = false;
	}

	void setB(float _b) {
		hasB = true;
		b = _b;
	}
	void unsetB() {
		hasB = false;
	}

	void setE(float _e) {
		hasE = true;
		e = _e;
	}
	void unsetE() {
		hasE = false;
	}

	void setF(float _f) {
		hasF = true;
		f = _f;
	}
	void unsetF(float _f) {
		hasF = false;
	}
};

class G28 : public GCommand {
private:
	bool x, y, z, a, b;

public:
	G28() : GCommand(28) {
		G28(false, false, false, false, false);
	}

	G28(bool _x, bool _y, bool _z, bool _a, bool _b) : GCommand(28) {
		setX(_x);
		setY(_y);
		setZ(_z);
		setA(_a);
		setB(_b);
	}

	void setX(bool val) { x = val; }
	void setY(bool val) { y = val; }
	void setZ(bool val) { z = val; }
	void setA(bool val) { a = val; }
	void setB(bool val) { b = val; }

	bool X() { return x; }
	bool Y() { return y; }
	bool Z() { return z; }
	bool A() { return a; }
	bool B() { return b; }
};

#endif
