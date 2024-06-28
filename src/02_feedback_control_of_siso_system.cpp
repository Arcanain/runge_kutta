// ROS2のコア機能と標準メッセージ型をインクルード
#include "rclcpp/rclcpp.hpp"        // ROS2の基本的なノード機能を提供
#include "Eigen/Dense"
#include "Eigen/Core"
#include <iostream>
#include <fstream>
#include "arcanain_modern_control_tutorial/matplotlibcpp.h"
#include "cpp_robotics/system.hpp"
#include "cpp_robotics/controller.hpp"
#include "cpp_robotics/controller/modern_control.hpp"

using namespace Eigen;
using namespace std;
// 時間リテラル（例：500ms）を使うための名前空間を使用
using namespace std::chrono_literals;
namespace plt = matplotlibcpp;
namespace cr = cpp_robotics;

// SISOPublisherクラスはrclcpp::Nodeクラスを継承している
class SISOPublisher : public rclcpp::Node
{
public:
  // コンストラクタ
  SISOPublisher()
  : Node("feedback_control_of_siso_system") // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
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
    get_RungeKutta(dt);
    std::cout << "tt:" << tt << std::endl;
    std::cout << "x1:" << X(1, 0) << std::endl;
    store_data(tt, X(0, 0), X(1, 0));
    last_time = current_time;
    if (tt > 5.0){
      graph_output();
      rclcpp::shutdown();
    }
  }
  
  void get_RungeKutta(double dt) {
    MatrixXd k1 = Af*X;
    //std::cout << "k1:" << std::endl << k1 << std::endl;
    MatrixXd k2 = Af*(X + 0.5*k1*dt);
    //std::cout << "k2:" << std::endl << k2 << std::endl;
    MatrixXd k3 = Af*(X + 0.5*k2*dt);
    //std::cout << "k3:" << std::endl << k3 << std::endl;
    MatrixXd k4 = Af*(X + k3*dt);
    //std::cout << "k4:" << std::endl << k4 << std::endl;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    //std::cout << "k:" << std::endl << k << std::endl;
    X = X + k;
    //std::cout << "X:" << std::endl << X << std::endl;
  }

  void InitialResponse(){
    X = Eigen::MatrixXd(2, 1);
    X(0, 0) = 1;
    X(1, 0) = 1;

    A = Eigen::MatrixXd(2, 2);
    B = Eigen::MatrixXd(2, 1);
    C = Eigen::MatrixXd(1, 2);
    A << 0.0, 1.0, -2.0, -3.0;
    B << 0, 1;
    C << 0, 1;
    //calculate eigenvalues
    Eigen::EigenSolver<Eigen::MatrixXf> s(A.cast<float>());
    std::cout << "pole " << std::endl << s.eigenvalues() << std::endl;
    std::cout << "vector" << std::endl << s.eigenvectors() << std::endl;

    //calculate feedback gain
    const double dt = 0.01;
    cr::StateSpaceSystem sys(A, B, C, dt);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "pole place" << std::endl;
    std::vector<double> pole1 = {-2, -3};
    //std::vector<double> pole2 = {-5, -6};
    Eigen::MatrixXd f1 = cr::place(sys, pole1);
    //Eigen::MatrixXd f2 = cr::place(sys, pole2);

    // calculate the matrix Af
    Af = A - B * f1.transpose();
    Af.resize(2, 2);
    std::cout << "Af:" << std::endl << Af << std::endl;
    
  }

  void store_data(double time, double x1, double x2){
    // 時間と値を保存
    data_time.push_back(time);
    data_x1.push_back(x1);
    data_x2.push_back(x2);
  }

  void graph_output(){
    //plt::clf();  // グラフをクリア
    //plt::ion();  // インタラクティブモードを有効にする
    map<string, string> args1{
            {"label", "x_1(t)"},
            {"c", "red"}
    };
    plt::plot(data_time, data_x1, args1);
 
    map<string, string> args2{
            {"label", "x_2(t)"},
            {"c", "blue"}
    };
    plt::plot(data_time, data_x2, args2); 
    //plt::pause(0.1);  // グラフを更新
    plt::xlabel("time [s]"); // X軸のラベルを設定
    plt::ylabel("x_1(t), x_2(t)"); // Y軸のラベルを設定
    plt::legend(); // 凡例を表示
    plt::draw();  // グラフを更新
    plt::show();
    plt::save("feedback_control_of_siso_system.png");
  }

  double tt = 0.0;
  Eigen::MatrixXd Af;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd X;
  std::vector<double> data_time;
  std::vector<double> data_x1;
  std::vector<double> data_x2;
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