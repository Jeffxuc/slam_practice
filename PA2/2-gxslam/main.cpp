#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//Ｅigen固定大小的矩阵最大支持到５０，超过５０的则需要用到动态矩阵来表示。
//Eigen::Matrix<double, a, b > matirx_ab ; 定义了一个ａ行，b列的双精度矩阵matrix_ab 。
/* Eigen::Matrix<double, a, a> matrix_aa ;定义了一个a*a的double类型方阵matrix_aa ，也可以用如下的方式表示
 * Eigen::Matrixad matrix_aa 。
 *
 * double类型的动态矩阵的表示方式如下两种形式：
 * Eigen::MatrixXd  matrix_x ; 或者　Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic > matrixXd ;
 *
 * double类型的向量数组表示如下：
 * Eigen::Matrix<double, a ,1 >  v_ad ; 或者　Eigen::Vectorad v_ad ;
 *
 * double类型的动态向量组：
 * Eigen::VectorXd v_Xd ;
 *
 * */
// #define MATRIX_SIZE 100;

int main(){
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    //Eigen::MatrixXd matrix_x;
    matrix_dynamic = Eigen::MatrixXd::Random(100, 100) ;
    // cout << matrix_dynamic <<endl<<endl ;
    Eigen::VectorXd b_Xd = Eigen::VectorXd::Random(100, 1) ;

    clock_t sttime = clock() ;
    Eigen::VectorXd x_Xd = matrix_dynamic.inverse() * b_Xd ;
    // cout << "the answer get by normal is " << '\n' << x_Xd << endl << endl ;
    cout << "the time inverse used is " << 1000*(clock()-sttime)/(double)CLOCKS_PER_SEC << "ms" << endl ;

    clock_t time = clock() ;
    Eigen::VectorXd y_Xd = matrix_dynamic.colPivHouseholderQr().solve(b_Xd) ;
    // cout << "the answer geted by QR is " << y_Xd << endl << endl ;
    cout << "the time used by QR is " << 1000*(clock() - time)/(double)CLOCKS_PER_SEC <<"ms"<< endl;

    clock_t csk = clock() ;
    Eigen::VectorXd z_Xd = matrix_dynamic.llt().solve(b_Xd) ;
    // cout << "the answer geted by Cholesky is " << z_Xd << endl << endl ;
    cout << "the time used by Cholesky is " << 1000*(clock()-csk)/(double)CLOCKS_PER_SEC <<"ms"<< endl;


    Matrix<double , 2 ,3 > A;
    A << 1,2,3,4,19,40 ;
    cout << "The Matrix A is as follows : "<< '\n' << A <<endl ;

    //动态矩阵生成行向量，要经常查看Eigen的　Quick reference guide来了解相关的应用。
    Matrix<int , 1 , 10 > B;
    B = RowVectorXi::Random(1 ,10) ;
    cout<< "The Matrix B is as follows : "<< '\n' << B << endl ;

    MatrixXd matrix_33 = Matrix3d::Random() ;
    cout << "The matrix_33 is : "<< '\n' << matrix_33 <<endl ;


    return 0 ;
}
