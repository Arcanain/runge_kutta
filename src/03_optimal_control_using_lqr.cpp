// ROS2のコア機能と標準メッセージ型をインクルード
#include "rclcpp/rclcpp.hpp"        // ROS2の基本的なノード機能を提供
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include <fstream>
#include "arcanain_modern_control_tutorial/matplotlibcpp.h"
/*
#include "cpp_robotics/system.hpp"
#include "cpp_robotics/controller.hpp"
#include "cpp_robotics/controller/modern_control.hpp"
*/

using namespace Eigen;
using namespace std;
// 時間リテラル（例：500ms）を使うための名前空間を使用
using namespace std::chrono_literals;
namespace plt = matplotlibcpp;
//namespace cr = cpp_robotics;

// SISOPublisherクラスはrclcpp::Nodeクラスを継承している
class SISOPublisher : public rclcpp::Node
{
public:
  // コンストラクタ
  SISOPublisher()
  : Node("optimal_control_using_lqr") // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
  {
    InitialResponse();
    // 500ミリ秒ごとにtimer_callback関数を呼び出すタイマーを作成
    timer_ = this->create_wall_timer(10ms, std::bind(&SISOPublisher::timer_callback, this));
    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();
  }

private:
  
  // タイマーのコールバック関数
  void timer_callback()
  {
    current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();
    tt += dt;
    X = get_RungeKutta(X, Af, dt);
    X2 = get_RungeKutta(X2, Af2, dt);
    Eigen::MatrixXd ut = - f * X;
    Eigen::MatrixXd ut2 = - f2 * X2;
    std::cout << "tt:" << tt << std::endl;
    std::cout << "x_1:" << X(1, 0) << std::endl;
    std::cout << "x2_1:" << X2(1, 0) << std::endl;
    std::cout << "ut:" << ut << std::endl;
    std::cout << "ut2:" << ut2 << std::endl;
    store_data(tt, X, X2, ut(0, 0), ut2(0, 0));
    last_time = current_time;
    if (tt > 5.0){
      plot_xt_response(data_x1, data_x2);
      plot_xt_response(data_x2_1, data_x2_2);
      plot_ut_response();
      rclcpp::shutdown();
    }
    
  }
  
  Eigen::MatrixXd get_RungeKutta(Eigen::MatrixXd tempX, const Eigen::MatrixXd Af, double dt) {
    MatrixXd k1 = Af*tempX;
    //std::cout << "k1:" << std::endl << k1 << std::endl;
    MatrixXd k2 = Af*(tempX + 0.5*k1*dt);
    //std::cout << "k2:" << std::endl << k2 << std::endl;
    MatrixXd k3 = Af*(tempX + 0.5*k2*dt);
    //std::cout << "k3:" << std::endl << k3 << std::endl;
    MatrixXd k4 = Af*(tempX + k3*dt);
    //std::cout << "k4:" << std::endl << k4 << std::endl;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    //std::cout << "k:" << std::endl << k << std::endl;
    tempX += k;
    //std::cout << "X:" << std::endl << X << std::endl;
    return tempX;
  }

  Eigen::MatrixXd care(const Eigen::MatrixXd A,const Eigen::MatrixXd B,const Eigen::MatrixXd Q,const Eigen::MatrixXd R){
    int n = (int)A.rows();
    // Hamilton Matrix
    Eigen::MatrixXd Ham(2*n, 2*n);
    Ham << A, -B*R.inverse()*B.transpose(), -Q, -A.transpose();

    // EigenVec, Value
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
    if (Eigs.info() != Eigen::Success) abort();

    // eigenvector storage
    Eigen::MatrixXcd eigvec(2*n, n);
    int j = 0;

    // store those with negative real number
    for(int i = 0; i < 2*n; ++i){
        if(Eigs.eigenvalues()[i].real() < 0){
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2*n, 1);
            ++j;
        }
    }

    Eigen::MatrixXcd U(n, n);
    Eigen::MatrixXcd V(n, n);

    U = eigvec.block(0,0,n,n);
    V = eigvec.block(n,0,n,n);

    return (V * U.inverse()).real();
}

  void InitialResponse(){
    //評価関数の重みQとRを与える
    Q = Eigen::MatrixXd(2, 2);
    Q << 13.0, 0.0, 0.0, 1.0;

    Q2 = Eigen::MatrixXd(2, 2);
    Q2 << 1.0, 0.0, 0.0, 1.0;


    //r = 1
    r = Eigen::MatrixXd(1, 1);
    r << 1.0;
    //r = 3
    r2 = Eigen::MatrixXd(1, 1);
    r2 << 3.0;

    X = Eigen::MatrixXd(2, 1);
    X(0, 0) = -1;
    X(1, 0) = 0;

    X2 = Eigen::MatrixXd(2, 1);
    X2(0, 0) = X(0, 0);
    X2(1, 0) = X(1, 0);

    A = Eigen::MatrixXd(2, 2);
    B = Eigen::MatrixXd(2, 1);
    C = Eigen::MatrixXd(1, 2);
    A << 0.0, 1.0, -(K / M), -(D / M);
    B << 0, 1/M;
    C << 1.0, 0.0;

    P = care(A, B, Q, r);
    P2 = care(A, B, Q2, r2);
    f = r.inverse() * B.transpose() * P;
    f2 = r2.inverse() * B.transpose() * P2;

    std::cout << "P:" << std::endl << P << std::endl;
    std::cout << "f:" << std::endl << f << std::endl;

    // calculate the matrix Af
    Af = A - B * f;
    Af2 = A - B * f2;

    Af.resize(2, 2);
    Af2.resize(2, 2);

    //std::cout << "Af:" << std::endl << Af << std::endl;
    
  }

  void store_data(double time, const Eigen::MatrixXd X, const Eigen::MatrixXd X2, double ut, double ut2){
    // 時間と値を保存
    data_time.push_back(time);
    data_x1.push_back(X(0, 0));
    data_x2.push_back(X(1, 0));
    data_x2_1.push_back(X2(0, 0));
    data_x2_2.push_back(X2(1, 0));
    data_ut.push_back(ut);
    data_ut2.push_back(ut2);
  }

  void plot_xt_response(std::vector<double> temp_x1, std::vector<double> temp_x2){
    //plt::clf();  // グラフをクリア
    //plt::ion();  // インタラクティブモードを有効にする
    map<string, string> args1{
            {"label", "x_1(t)"},
            {"c", "red"}
    };
    plt::plot(data_time, temp_x1, args1);
 
    map<string, string> args2{
            {"label", "x_2(t)"},
            {"c", "blue"}
    };
    plt::plot(data_time, temp_x2, args2); 
    plt::ylim(-1.2, 2.2);
    //plt::pause(0.1);  // グラフを更新
    plt::xlabel("time [s]"); // X軸のラベルを設定
    plt::ylabel("x_1(t), x_2(t)"); // Y軸のラベルを設定
    plt::legend(); // 凡例を表示
    plt::draw();  // グラフを更新
    plt::show();
    //plt::save("feedback_control_of_siso_system.png");
  }

  void plot_ut_response(){
    //plt::clf();  // グラフをクリア
    //plt::ion();  // インタラクティブモードを有効にする
    map<string, string> args1{
            {"label", "r = 1"},
            {"c", "red"}
    };
    plt::plot(data_time, data_ut, args1);
    
    map<string, string> args2{
            {"label", "r = 3"},
            {"c", "blue"}
    };
    plt::plot(data_time, data_ut2, args2); 
    plt::ylim(-0.2, 1.2);
    //plt::pause(0.1);  // グラフを更新
    plt::xlabel("time [s]"); // X軸のラベルを設定
    plt::ylabel("u(t)"); // Y軸のラベルを設定
    plt::legend(); // 凡例を表示
    plt::draw();  // グラフを更新
    plt::show();
    //plt::save("response_u(t).png");
  }

  //マスーばねーダンパシステムの定義
  double M = 1.0, D = 5.0, K = 6.0;

  double tt = 0.0;
  Eigen::MatrixXd Af;
  Eigen::MatrixXd Af2;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd Q2;
  Eigen::MatrixXd r;
  Eigen::MatrixXd r2;
  Eigen::MatrixXd P;
  Eigen::MatrixXd P2;
  Eigen::MatrixXd X;
  Eigen::MatrixXd X2;
  Eigen::MatrixXd f;
  Eigen::MatrixXd f2;
  std::vector<double> data_time;
  std::vector<double> data_x1;
  std::vector<double> data_x2;
  std::vector<double> data_x2_1;
  std::vector<double> data_x2_2;
  std::vector<double> data_ut;
  std::vector<double> data_ut2;
  rclcpp::TimerBase::SharedPtr timer_;  // タイマーを保持するためのスマートポインタ
  rclcpp::Time current_time, last_time;
  ofstream ofs;
};

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // ROS2システムの初期化
  // MinimalPublisherノードのインスタンスを作成し、スピンすることでコールバック関数を実行可能にする
  rclcpp::spin(std::make_shared<SISOPublisher>());
  rclcpp::shutdown();  // ROS2システムのシャットダウン
  return 0;            // プログラム終了
}