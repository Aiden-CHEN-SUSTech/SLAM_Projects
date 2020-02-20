#include <iostream>
#include <cmath>

using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main(int argc, char **argv) {
   Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();//Initial the matrix as E
	 Eigen::AngleAxisd rotation_vetor(M_PI / 4, Eigen::Vector3d(0,0,1));
 	 cout .precision(3);//Keep 3 decimal places
	 cout<<"rotation matrix:\n"<<rotation_vetor.matrix()<<endl;//Vector convert to matrix
	 
	 rotation_matrix = rotation_vetor.toRotationMatrix();
	 
	 Eigen::Vector3d v(1,0,0);
	 Eigen::Vector3d v_rotated = rotation_vetor * v;
	 cout<<"(1,0,0) after rotation:\n"<<v_rotated.transpose()<<endl;//Transpose the vector
	 
	 Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);//The oder is ZYX
	 cout<<"yaw pitch roll\n"<<euler_angles.transpose()<<endl;
	 
	 Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	 T.rotate(rotation_vetor);//T's rotation is rotation_vetor
	 T.pretranslate(Eigen::Vector3d(1,3,4));//T's pretranslate is (1,3,4)
	 cout<<"Transform matrix:\n"<<T.matrix()<<endl;
	 
	 Eigen::Vector3d v_transform = T * v;
	 cout<<"(1,0,0) after transform:\n"<<v_transform.transpose()<<endl; 
	 
	 Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vetor);//From rotation vector to quaterniond
	 cout<<"quaterniond = \n"<<q.coeffs()<<endl;
	 
	 v_rotated = q * v;//Have difference between code and formula
	 cout<<"(1,0,0) after roation =\n "<<v_rotated.transpose()<<endl;
	 
	 return 0;
}
