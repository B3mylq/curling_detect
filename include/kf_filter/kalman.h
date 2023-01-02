#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;

class Kalman
{
public:
    int m_StateSize;
    int m_MeaSize;
    int m_USize;
    Eigen::VectorXd m_x;
    Eigen::VectorXd m_u;
    Eigen::VectorXd m_z;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;
    Eigen::MatrixXd m_P;
    Eigen::MatrixXd m_H;
    Eigen::MatrixXd m_R;
    Eigen::MatrixXd m_Q;
    Eigen::MatrixXd m_iden_mat;

public:
    Kalman()
    {
        cout << "Kalman construct function......" << endl;
    }
    Kalman(int statesize, int measize, int usize) : m_StateSize(statesize), m_MeaSize(measize), m_USize(usize)
    {
        if (m_StateSize == 0 && m_MeaSize == 0)
        {
            cout << "Init........." << endl;
        }

        m_x.resize(statesize);
        m_x.setZero();

        m_u.resize(usize);
        m_u.setZero();

        m_z.resize(measize);
        m_z.setZero();

        m_A.resize(statesize, statesize);
        m_A.setIdentity();

        m_B.resize(statesize, usize);
        m_B.setZero();

        m_P.resize(statesize, statesize);
        m_P.setIdentity();

        m_H.resize(measize, statesize);
        m_H.setZero();

        m_R.resize(measize, measize);
        m_R.setZero();

        m_Q.resize(statesize, statesize);
        m_Q.setZero();

        m_iden_mat.resize(statesize, statesize);
        m_iden_mat.setIdentity();
    }
    // void Init(Eigen::Matrix<double,6,1> &x,Eigen::Matrix<double,6,6> &P,Eigen::Matrix2d &R,Eigen::Matrix<double,6,6> &Q);
    // Eigen::VectorXd predict(Eigen::Matrix<double,6,6> &A);
    // Eigen::VectorXd predict(Eigen::Matrix<double,6,6> &A,Eigen::MatrixXd &B,Eigen::VectorXd &u);
    // void Update(Eigen::Matrix<double,2,6> &H,Eigen::Matrix<double,2,1> z_meas);

    void Init_Par(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &R, Eigen::MatrixXd &Q, Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::MatrixXd &H, Eigen::VectorXd &u);
    void Predict_State();
    void Predict_Cov();
    Eigen::VectorXd Mea_Resd(Eigen::VectorXd &z);
    Eigen::MatrixXd Cal_Gain();
    void Update_State();
    void Update_Cov();
};