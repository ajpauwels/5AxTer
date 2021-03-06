#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

#include "utility.hpp"

class ForwardKinematics {
private:
	float pos_00(float x_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float s_rho, float c_rho, float s_theta, float c_theta, float s_phi,
		float c_phi, float s_psi, float c_psi, float z_offset);
	float pos_10(float y_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float s_rho, float c_rho, float s_phi, float c_phi, float s_psi,
		float c_psi);
	float pos_20(float z_frame, float x_buildplate, float y_buildplate,
		float z_buildplate, float s_rho, float c_rho, float s_theta, float c_theta, float s_phi,
		float c_phi, float s_psi, float c_psi, float z_offset);

	float vel_jacobian_00() { return 1; }
	float vel_jacobian_01() { return 0; }
	float vel_jacobian_02() { return 0; }
	float vel_jacobian_03(float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
		float z_offset);
	float vel_jacobian_04(float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float c_theta, float s_phi, float c_phi, float s_psi,	float c_psi);
	float vel_jacobian_05(float s_rho, float c_rho, float s_theta, float c_theta, float c_phi);
	float vel_jacobian_06(float s_rho, float c_rho, float s_theta, float c_theta, float s_phi,
		float c_phi, float s_psi, float c_psi);
	float vel_jacobian_07(float s_rho, float c_rho, float s_theta, float c_theta, float s_phi,
		float c_phi, float s_psi, float c_psi);

	float vel_jacobian_10() { return 0; }
	float vel_jacobian_11() { return 1; }
	float vel_jacobian_12() { return 0; }
	float vel_jacobian_13() { return 0; }
	float vel_jacobian_14(float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_phi, float c_phi, float s_psi, float c_psi);
	float vel_jacobian_15(float c_rho, float s_phi);
	float vel_jacobian_16(float s_rho, float s_phi, float c_phi, float s_psi, float c_psi);
	float vel_jacobian_17(float s_rho, float s_phi, float c_phi, float s_psi,	float c_psi);

	float vel_jacobian_20() { return 0; }
	float vel_jacobian_21() { return 0; }
	float vel_jacobian_22() { return 1; }
	float vel_jacobian_23(float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi, float c_psi,
		float z_offset);
	float vel_jacobian_24(float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_theta, float s_phi, float c_phi, float s_psi, float c_psi);
	float vel_jacobian_25(float s_rho, float c_rho, float s_theta, float c_theta, float c_phi);
	float vel_jacobian_26(float s_rho, float c_rho, float s_theta, float c_theta, float s_phi,
		float c_phi, float s_psi, float c_psi);
	float vel_jacobian_27(float s_rho, float c_rho, float s_theta, float c_theta,	float s_phi,
		float c_phi, float s_psi, float c_psi);

	float acc_jacobian_00() { return 0; }
	float acc_jacobian_01() { return 0; }
	float acc_jacobian_02() { return 0; }
	float acc_jacobian_03(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot,float x_buildplate, float y_buildplate, float z_buildplate,
		float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi,
		float c_psi, float z_offset);
	float acc_jacobian_04(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
		float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi,
		float c_psi);
	float acc_jacobian_05(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi);
	float acc_jacobian_06(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi, float s_psi, float c_psi);
	float acc_jacobian_07(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi, float s_psi, float c_psi);

	float acc_jacobian_10() { return 0; }
	float acc_jacobian_11() { return 0; }
	float acc_jacobian_12() { return 0; }
	float acc_jacobian_13() { return 0; }
	float acc_jacobian_14(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate, float s_rho,
		float c_rho, float s_phi, float c_phi, float s_psi, float c_psi);
	float acc_jacobian_15(float phi_dot, float c_rho, float c_phi);
	float acc_jacobian_16(float phi_dot, float s_rho, float s_phi, float c_phi, float s_psi,
		float c_psi);
	float acc_jacobian_17(float phi_dot, float s_rho, float s_phi, float c_phi, float s_psi,
		float c_psi);

	float acc_jacobian_20() { return 0; }
	float acc_jacobian_21() { return 0; }
	float acc_jacobian_22() { return 0; }
	float acc_jacobian_23(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
		float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi,
		float c_psi);
	float acc_jacobian_24(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
		float s_rho, float c_rho, float s_theta, float c_theta, float s_phi, float c_phi, float s_psi,
		float c_psi);
	float acc_jacobian_25(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi);
	float acc_jacobian_26(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi, float s_psi, float c_psi);
	float acc_jacobian_27(float theta_dot, float phi_dot, float s_rho, float c_rho, float s_theta,
		float c_theta, float s_phi, float c_phi, float s_psi, float c_psi);

public:
	// Matrix dimensions are [# rows][# cols]
	Matrix3x1 posMatrix;
	Matrix3x8 velMatrix;
	Matrix3x8 accMatrix;

	ForwardKinematics() {}

	Matrix3x1 position(float x_frame, float y_frame, float z_frame, float x_buildplate,
		float y_buildplate, float z_buildplate, float rho, float theta, float phi,
		float psi, float z_offset);

	Matrix3x8 velocity_jacobian(float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);

	Matrix3x8 acceleration_jacobian(float x_dot_buildplate, float y_dot_buildplate, float z_dot_buildplate,
		float theta_dot, float phi_dot, float x_buildplate, float y_buildplate, float z_buildplate,
		float rho, float theta, float phi, float psi, float z_offset);
};

#endif
