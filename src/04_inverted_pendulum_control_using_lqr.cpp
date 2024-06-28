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

// LQRPublisherクラスはrclcpp::Nodeクラスを継承している
class LQRPublisher : public rclcpp::Node
{
public:
  // コンストラクタ
  LQRPublisher()
  : Node("inverted_pendulum_control_using_lqr") // Nodeクラスのコンストラクタを呼び出し、ノード名を指定
  {
    InitialResponse();
    // 500ミリ秒ごとにtimer_callback関数を呼び出すタイマーを作成
    timer_ = this->create_wall_timer(10ms, std::bind(&LQRPublisher::timer_callback, this));
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
    std::cout << "tt:" << tt << std::endl;

    X = get_RungeKutta(X, Af, dt);
    Eigen::MatrixXd ut = - f * X;
    std::cout << "x:" << X(1, 0) << std::endl;
    std::cout << "ut:" << ut << std::endl;
    store_data(tt, X, ut(0, 0));
    last_time = current_time;

    //15秒後にグラフを出力
    if (tt > 15.0){
      std::cout << "X:" << data_X[0] << std::endl;
      std::cout << "X.rows():" << data_X[0].size() << std::endl;

      std::vector<std::string> lbls = {"$p$ [m]", "$\\theta$ [deg]", "$\\dot{p}$ [m/s]", "$\\dot{\\theta}$ [deg/s]"};
      std::vector<double> scls = {1.0, 180.0 / M_PI, 1.0, 180.0 / M_PI};
      
      //xの応答を出力
      for (int i = 0; i < int(data_X[0].size()); ++i){
        std::vector<double> response_data;
        for (int j = 0; j < data_X.size(); ++j){
          response_data.push_back(data_X[j](i, 0) * scls[i]);
        }
        plot_graph(response_data,lbls[i]);
      }
      plot_graph(data_ut,"u(t)");

      //倒立振子をアニメーションで可視化
      for (int i = 0; i < int(data_X.size()); i+=5){
        draw_pendulum(data_time[i], data_X[i](0, 0), data_X[i](1, 0), l);
        plt::pause(0.01);
      }

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
    //A,B,X0を定義
    A = Eigen::MatrixXd(4, 4);
    B = Eigen::MatrixXd(4, 1);
    X = Eigen::MatrixXd(4, 1);

    A << 0, 0, 1, 0,
        0, 0, 0, 1,
        0, g * m2 / m1, 0, 0,
        0, g * (m1 + m2) / (l * m1), 0, 0;

    B << 0, 0, 1 / m1, 1 / m2;

    X << 0, 0.5, 0, 0;
    std::cout << "A:" << std::endl << A << std::endl;

    //評価関数の重みQとRを与える
    Q = Eigen::MatrixXd(4, 4);
    Q << 1, 0, 0, 0,
         0, 5, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    r = Eigen::MatrixXd(1, 1);
    r << 10;

    P = care(A, B, Q, r);
    f = r.inverse() * B.transpose() * P;

    std::cout << "P:" << std::endl << P << std::endl;
    std::cout << "f:" << std::endl << f << std::endl;

    // calculate the matrix Af
    Af = A - B * f;
    Af.resize(4, 4);
    std::cout << "Af:" << std::endl << Af << std::endl;

    
  }

  void draw_pendulum(double t, double xt, double theta, double l) {
      plt::clf();
      plt::ion();
      double cart_w = 0.8;
      double cart_h = 0.3;
      double radius = 0.1;
      std::vector<double> cx = {-0.3, 0.3, 0.3, -0.3, -0.3};
      for (auto& val : cx) {
          val *= cart_w;
          val += xt;
      }
      std::vector<double> cy = { 0.0, 0.0, 1.0, 1.0, 0.0 };
      for (auto& val : cy) {
          val *= cart_h;
      }
      // thetaが-πからπの範囲になるように変換
      while (theta < -M_PI) {
          theta += 2 * M_PI;
      }
      while (theta > M_PI) {
          theta -= 2 * M_PI;
      }

      std::vector<double> bx = { xt, xt + l * sin(-theta)};
      std::vector<double> by = { cart_h , l * cos(-theta) + cart_h };
      
      plt::plot(cx, cy, "-k");
      plt::plot(bx, by, "-k");
      //plt::axis("equal");
      plt::xlim(-cart_w, cart_w);
      plt::ylim(0, int((cart_h + l) * 2) );
      plt::title("t:" + std::to_string(t) + " x:" + std::to_string(xt) + " theta:" + std::to_string(theta));
      plt::draw();
      //plt::show();
  }

  void store_data(double time, const Eigen::MatrixXd X, double ut){
    // 時間と値を保存
    data_time.push_back(time);
    data_x1.push_back(X(0, 0));
    data_x2.push_back(X(1, 0));
    data_X.push_back(X);
    data_ut.push_back(ut);
  }

  void plot_graph(const std::vector<double>& data, const std::string& lbl){
    //plt::clf();  // グラフをクリア
    //plt::ion();  // インタラクティブモードを有効にする
    map<string, string> args1{
            {"label", lbl},
            {"c", "black"}
    };
    plt::plot(data_time, data, args1);
    //plt::ylim(-1.2, 2.2);
    //plt::pause(0.1);  // グラフを更新
    plt::xlabel("time [s]"); // X軸のラベルを設定
    plt::ylabel(lbl); // Y軸のラベルを設定
    plt::legend(); // 凡例を表示
    plt::draw();  // グラフを更新
    plt::show();
    //plt::save("feedback_control_of_siso_system.png");
  }

  //倒立振り子システムの定義
  double m1 = 1.0, m2 = 0.1, l = 0.8;
  //重力加速度
  double g = 9.80665;

  double tt = 0.0;
  Eigen::MatrixXd Af;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd r;
  Eigen::MatrixXd P;
  Eigen::MatrixXd X;
  Eigen::MatrixXd f;
  std::vector<Eigen::MatrixXd> data_X;
  std::vector<double> data_time;
  std::vector<double> data_x1;
  std::vector<double> data_x2;
  std::vector<double> data_ut;
  rclcpp::TimerBase::SharedPtr timer_;  // タイマーを保持するためのスマートポインタ
  rclcpp::Time current_time, last_time;
  ofstream ofs;
};

// main関数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // ROS2システムの初期化
  // MinimalPublisherノードのインスタンスを作成し、スピンすることでコールバック関数を実行可能にする
  rclcpp::spin(std::make_shared<LQRPublisher>());
  rclcpp::shutdown();  // ROS2システムのシャットダウン
  return 0;            // プログラム終了
}