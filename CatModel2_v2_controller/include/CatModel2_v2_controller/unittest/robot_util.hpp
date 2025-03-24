#ifndef ROBOT_UTIL_HPP  
#define ROBOT_UTIL_HPP 

template <typename T>  
T baseClamp(T value, T min, T max) {  
    if (value < min) {  
        return min;  
    } else if (value > max) {  
        return max;  
    } else {  
        return value;  
    }  
}

template<typename T>
Eigen::Matrix<T, 3, 3> rotx(T val) {
  Eigen::Matrix<T, 3, 3> R;
  R << 1, 0, 0,
       0,  cos(val), -sin(val),
       0,  sin(val),  cos(val);
  return R;
}

template<typename T>
Eigen::Matrix<T, 3, 3> roty(T val) {
  Eigen::Matrix<T, 3, 3> R;
  R << cos(val),  0, sin(val),
              0,  1,        0,
      -sin(val),  0, cos(val);
  return R;
}

template<typename T>
Eigen::Matrix<T, 3, 3> rotz(T val) {
  Eigen::Matrix<T, 3, 3> R;
  R << cos(val), -sin(val), 0,
       sin(val),  cos(val), 0,
              0,         0, 1;
  return R;
}

template<typename T>
Eigen::Matrix<T, 3, 1> rotm2eul(const Eigen::Matrix<T, 3, 3>& val) {
  return Eigen::Matrix<T, 3, 1>(std::atan2(val(2, 1), val(2, 2)), 
                                std::atan2(-val(2, 0), std::sqrt(val(2, 1) * val(2, 1) + val(2, 2) * val(2, 2))), 
                                std::atan2(val(1, 0), val(0, 0)));
}

template<typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& val) {
  Eigen::Matrix<T, 3, 3> R;
  R << 0, -val(2), val(1),
       val(2),  0, -val(0),
       -val(1),         val(0), 0;
  return R;
}

template<typename T>
Eigen::Matrix<T, 3, 1> skewt(const Eigen::Matrix<T, 3, 3>& val) {
  return Eigen::Matrix<T, 3, 1>(-val(1, 2), val(0, 2), -val(0, 1));
}

template<typename T>
Eigen::Matrix<T, 3, 3> expm(Eigen::Matrix<T, 3, 1> val) {
  T theta = sqrt(val(0)*val(0) + val(1)*val(1) + val(2)*val(2));
  T h = 1e-10;
  if (theta < h) {
    theta = h;
  }
  Eigen::Matrix<T, 3, 3> V = skew(val);
  return Eigen::Matrix<T, 3, 3>::Identity() + (sin(theta)/theta)*V + ((1,0-cos(theta))/(theta*theta))*(V*V);
}

template<typename T>
Eigen::Matrix<T, 3, 1> logm(const Eigen::Matrix<T, 3, 3>& val) {
  T acosinput = (val(0, 0) + val(1, 1) + val(2, 2) - 1.0) / 2.0;
  T h = 1e-10;
  if (acosinput >= 1.0) {
    return Eigen::Matrix<T, 3, 1>(0, 0, 0);
  } else if (acosinput <= -1.0) {
    Eigen::Matrix<T, 3, 1> omg;
    omg.setZero();
    if (fabs(1 + val(2,2)) > h) {
      omg = (1.0/sqrt(2.0*(1.0+val(2,2))))*Eigen::Matrix<T, 3, 1>(val(0,2), val(1,2), 1.0+val(2,2));
    } else if (fabs(1.0 + val(1,1)) > h) {
      omg = (1.0/sqrt(2.0*(1.0+val(1,1))))*Eigen::Matrix<T, 3, 1>(val(0,1), 1.0+val(1,1), val(2,1));
    } else {
      omg = (1.0/sqrt(2.0*(1.0+val(0,0))))*Eigen::Matrix<T, 3, 1>(1.0+val(0,0), val(1,0), val(2,0));
    }
    return M_PI*omg;
  } else {
    T theta = acos(acosinput);
    Eigen::Matrix<T, 3, 3> R = val - val.transpose();
    return theta * (1.0 / (2.0 * sin(theta))) * skewt(R);
  }
  return Eigen::Matrix<T, 3, 1>(0, 0, 0);
}

template<typename T>
Eigen::Matrix<T, 3, 3> jl(const Eigen::Matrix<T, 3, 1>& val) {
  T theta = sqrt(val(0)*val(0) + val(1)*val(1) + val(2)*val(2));
  T h = 1e-10;
  if (theta < h) {
    theta = h;
  }
  Eigen::Matrix<T, 3, 3> V = skew(val)/theta;
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
  return (sin(theta)/theta)*I + (1.0-(sin(theta)/theta))*(V*V+I) + ((1.0-cos(theta))/(theta))*(V);
}

template<typename T>
Eigen::Matrix<T, 3, 3> jlt(const Eigen::Matrix<T, 3, 1>& val) {
  T theta = sqrt(val(0)*val(0) + val(1)*val(1) + val(2)*val(2));
  T h = 1e-10;
  if (theta < h) {
    theta = h;
  }
  Eigen::Matrix<T, 3, 3> V = skew(val)/theta;
  Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
  T cot = 1.0/tan(0.5*theta);
  return (0.5*cot*theta)*I + (1.0-(0.5*cot*theta))*(V*V+I) - (0.5*(theta))*(V);
}

#endif // ROBOT_UTIL_HPP