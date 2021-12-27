#ifndef FORCEEST_H
#define FORCEEST_H

#include "ukf.h"
#include "iostream"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

//recovery
enum state {
	p_x = 0,
	p_y,
	p_z,
	v_x,
	v_y,
	v_z,
	e_x,
	e_y,
	e_z,
	F1_x,
	F1_y,
	F1_z,
	F2_x,
	F2_y,
	F2_z,
	statesize
};

enum measurement {
	mp_x = 0,
	mp_y,
	mp_z,
	mv_x,
	mv_y,
	mv_z,
	me_x,
	me_y,
	me_z,
	measurementsize
};


class forceest : public ukf
{

public:
	forceest(int x, int y) : ukf(x,y)
	{

	}
	double thrust;
	Eigen::Matrix3d R_IB;
	Eigen::Vector3d U;
//Eigen::Vector4d q_k;
	Eigen::Vector4d q_m_k;
	Eigen::Vector4d q_k1_0;
	Eigen::Vector4d qk1;
	Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
	Eigen::MatrixXd state_to_measure(Eigen::MatrixXd sigma_state);
	Eigen::Vector3d gausian_noise;
private:


};

#endif // FORCEEST_H
