#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
    // the basic condition of the topic.
    //定义,初始化四元数，并进行归一化。四元数在使用前需要进行归一化,使用normalized()函数。
    Quaterniond q1 = Quaterniond(0.55,0.3,0.2,0.2).normalized() ;
    Quaterniond q2 = Quaterniond(-0.1,0.3,-0.7,0.2).normalized() ;
    Vector3d t1,t2,p1,p2 ;
    t1 << 0.7,1.1,0.2 ;
    t2 << -0.1,0.4,0.8 ;
    p1 << 0.5,-0.1,0.2 ;


    //将四元数转换成旋转矩阵R:
    Matrix3d R;
    R = Matrix3d(q1) ;
    cout << "R= \n"<< R << endl ;
    //将旋转矩阵R转换成旋转向量（轴角），
    AngleAxisd angle_rotate ;
    angle_rotate = AngleAxisd(R) ;
    cout << "The angle_rotate from R is \n" << angle_rotate.matrix() <<endl ;


    //将四元数转换成旋转向量（轴角），再以矩阵的形式输出。
    AngleAxisd v1 = AngleAxisd(q1) ;  //旋转向量不可直接输出，需要转换成旋转矩阵后才能用cout输出。
    cout << "The AngleAxisd v1 from q1 :\n"<< v1.matrix() << endl ;


   //Method1: the solution of the quaternion：p2=q2 * q1^{-1} * (p1-t1) + t2 ;
   p2 = q2 * q1.inverse() * (p1 - t1) +t2 ;
   cout << "The coordination of the point p2 is : " << '\n' << p2.transpose() << endl ;


   //Method2: the solution through isometry 。 p2 = T2 * T1^{-1} * p1 .
   //虽然为３d，但实际上是４x4矩阵，（即:R,t）。下面将四元数p1和平移向量t1转换成对应的变换矩阵: T1
   Isometry3d  T1 = Isometry3d::Identity() ;  //定义T1为4x4的转换矩阵，并进行初始化为单位矩阵。
   T1.rotate(q1) ;     //T1按照四元数q1来进行旋转变换：将四元数化成旋转矩阵。
   T1.pretranslate(t1) ;    // T1旋转之后再按照t1的向量来进行平移变换。
   cout << "The transform matrix T1 is : "<< '\n' << T1.matrix() << endl ;   //　将q1和t1以矩阵的形式进行保存、输出。
   //将四元数p2和平移向量t2转换成对应的变换矩阵: T2
   Isometry3d T2 = Isometry3d::Identity() ;
   T2.rotate(q2) ;
   T2.pretranslate(t2) ;
   cout << "The transform matrix T2 is : "<< '\n' << T2.matrix() <<endl ;
   //通过变换矩阵来求解p2的位置。
   p2 = T2 * T1.inverse() * p1 ;
   cout << "The coordination of the point p2 is : "<< '\n' << p2.transpose() << endl ;


   // 利用旋转向量（也叫角轴）来表示位姿的旋转：AngleAxis，用matrix来将其转换成矩阵才能输出，v_rotate()无法直接输出。
   AngleAxisd v_rotate(M_PI/4 , Vector3d(1,0,0)) ;  //旋转的轴向量，如:Vector3d必须进行归一化。
   //通过matrix()函数将旋转向量，转化成旋转矩阵:v_rotate.matrix()。旋转矩阵为正交阵。
   cout << "The matrix transform from angle_axis is : \n" << v_rotate.matrix() << endl ;
   //除了上面的方法，还有一种方法也能将旋转向量转换成旋转矩阵。通过toRotationMatrix()函数来进行操作，如下：
   Matrix3d A = v_rotate.toRotationMatrix() ;
   cout << "A = \n" << A <<endl ;
   //也可以不用将v_rotate.toRotationMatrix()的值赋给Matrix3d所定义的矩阵A，而是直接如下输出即可：
   cout << "v_rotate.toRotationMatrix() is :\n"<< v_rotate.toRotationMatrix() << endl ;

   //通过AngleAxis来对一个向量直接进行坐标变换，旋转向量可以直接与待变换的向量进行相乘。如下：
   Vector3d vector1(1,0,0) , vector2 , vector3;
   vector2 = v_rotate*vector1 ;
   cout << "The vector1 transferred by AngleAxis is : \n" << vector2 << endl ;
   //也可以先将旋转向量(v_rotate)转化成旋转矩阵(v_rotate.matrix()或A)，再用旋转矩阵来对目标向量(vector1)进行变换。如下:
   vector3 = v_rotate.matrix() * vector1 ;  //或者 vector3 = A*vector1 .
   cout << "The vector3 transferred by rotation matrix is :\n"<< vector3 << endl ;


   //将旋转向量（角轴）、旋转矩阵转化成四元数。然后使用四元数来做旋转变换：
   Quaterniond q3 , q4 ;
   q3 = Quaterniond (v_rotate) ; //将旋转向量（角轴）转换成四元数。
   cout << "The q3 from AngleAxis is :\n" << q3.coeffs() << endl ; //coeffs()输出（x,y,z,w）,w为实部
   cout << "The result through quaternion is :\n"<< q3 * vector1 <<endl ; //数学上为: q3 * vector * q3^{-1}
   q4 = Quaterniond (A) ;  // 将旋转矩阵转换成四元数。
   cout << "The q4 from rotate_matrix is \n"<< q4.coeffs() << endl ;
   cout << "The result through q4 is :\n"<< q4 * vector1 << endl ;


   //欧拉角(eulerAngle)表示旋转方法,如下将旋转向量转化成欧拉角：
   Vector3d euler_angle = v_rotate.matrix().eulerAngles(1,0,0) ;
   cout << "The eulerAngle :yaw pitch roll is :\n"<< euler_angle << endl ; //输出为：Z Y X
   //欧拉角-----> 旋转矩阵：
   //用轴角来模拟欧拉角，在Ｘ,Ｙ,Ｚ方向上分别旋转PI/4 , PI/2 , PI/3 ，然后将三个方向的欧拉角进行组合就形成了旋转矩阵
   //Vector3d::UnitX()表示X轴的单位向量(1,0,0)^T
   Matrix3d B ;
   B = AngleAxisd(M_PI/4 , Vector3d::UnitX())
           *AngleAxisd(M_PI/2 , Vector3d::UnitY())
           *AngleAxisd(M_PI/3 , Vector3d::UnitZ()) ;
   cout << "The matrix B is :\n"<< B << endl ;
   //isUnitary()函数判断一个矩阵在给定的精度下是否为单位矩阵。若是，则返回１；若不是，则返回０.
   cout << "is Unitary :\n"<< B.isUnitary() << endl ;


   //Identity()为生成单位矩阵.如下会生成４行９列的单位阵（后５列全为０）。
   cout<< Matrix<double , 4, 9>::Identity() << endl ;
   //如下将生成3x3的double类型单位矩阵。Matrix3d也经常用来表示旋转矩阵。
   Matrix3d matrix_rotate = Matrix3d::Identity() ;
   cout << matrix_rotate << endl ;


    return 0;
}