#include <iostream>

using namespace std;

#include <ctime>

#include <Eigen/Core>

#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char **argv)
{
	//define a size of  2*3 matrix
	Matrix<float, 2, 3> matrix_23;

	Vector3d v_3d; //就是Eigen::Matrix<double,3,1>

	Matrix<float, 3, 1> vd_3d;

	Matrix3d matrix_33 = Matrix3d::Zero(); //initialization

	//如果不确定大小，可以使用动态大小的矩阵
	Matrix<double, Dynamic, Dynamic> matrix_dynamic;

	MatrixXd matrix_x;

	//Initialization
	matrix_23 << 1, 2, 3, 4, 5, 6;
	//Output
	cout << "matrix 2*3 from 1 to 6: \n"
		 << matrix_23 << endl;

	cout << "print matrix 2*3: " << endl;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cout << matrix_23(i, j) << "\t";
		}
		cout << endl;
	}

	//矩阵和向量相乘
	v_3d << 3, 2, 1;
	vd_3d << 4, 5, 6;

	//显示转换
	Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
	cout << "[1,2,3;4,5,6] * [3,2,1] =" << result.transpose() << endl;

	Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
	cout << "[1,2,3;4,5,6] * [4,5,6]= " << result2.transpose() << endl;

	//Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

	//  基础矩阵运算
	matrix_33 = Matrix3d::Random(); //  随机数矩阵
	cout << "random matrix: \n"
		 << matrix_33 << endl;
	cout << "transpose: \n"
		 << matrix_33.transpose() << endl;
	cout << "sum: " << matrix_33.sum() << endl;
	cout << "trace: " << matrix_33.trace() << endl;
	cout << "times 10: \n"
		 << 10 * matrix_33 << endl;
	cout << "inverse: \n"
		 << matrix_33.inverse() << endl;
	cout << "det: " << matrix_33.determinant() << endl;

	//  Eigenvalue
	//  实对称矩阵可以保证对角化成功
	SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
	cout << "Eigen values = \n"
		 << eigen_solver.eigenvalues() << endl;
	cout << "Eigen vectors =  \n"
		 << eigen_solver.eigenvectors() << endl;

	// 解方程
	// 我们求解 matrix_NN * x =v_Nd 的方程（即 Ax=b）
	// N的大小在前面的宏里定义，由随机数生成
	// 直接求逆是最自然的方法，但是工作量巨大

	Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
	matrix_NN = matrix_NN * matrix_NN.transpose(); // 保证半正定
	Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

	clock_t time_stt = clock(); // timer

	// inverse directly
	Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
	cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x.transpose() << endl;

	// Matrix decomposition ( such as QR decomposition, the speed improve a lot)
	time_stt = clock();
	x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
	cout << "time of Qr decomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "x =  " << x.transpose() << endl;

	// As for Positive-definite Matrix, we can use Cholesky Decomposition to solve it
	time_stt = clock();
	x = matrix_NN.ldlt().solve(v_Nd);
	cout << "time of ldlt devomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x.transpose() << endl;

	// 对大小为100*100的随机矩阵进行矩阵分解求解
	Matrix<double, Dynamic, Dynamic> matrix_dyn;
	Matrix<double, Dynamic, 1> v_dyn;

	// Random initialization
	v_dyn = MatrixXd::Random(100, 1);
	matrix_dyn = MatrixXd::Random(100, 100);
	Matrix<double, Dynamic, 1> x_dyn;
	matrix_dyn = matrix_dyn.transpose() * matrix_dyn; // 保证半正定

	// QR Decomposition
	time_stt = clock();
	x_dyn = matrix_dyn.colPivHouseholderQr().solve(v_dyn);
	cout << " time of Qr decomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "x = " << x_dyn.transpose() << endl;

	// Cholesky Decomposition
	time_stt = clock();
	x_dyn = matrix_dyn.ldlt().solve(v_dyn);
	cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "x= " << x_dyn.transpose() << endl;

	return 0;
}