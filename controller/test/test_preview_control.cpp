#include "lib/preview_control.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

int main()
{
  PreviewControl preview_control(0.01, 1.6, 0.27);

  std::vector<Eigen::Vector3d> zmp;
  zmp.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  zmp.emplace_back(Eigen::Vector3d(0.32, 0.0, 0.05));
  zmp.emplace_back(Eigen::Vector3d(0.64, 0.1, -0.05));
  zmp.emplace_back(Eigen::Vector3d(0.96, 0.2, 0.05));
  zmp.emplace_back(Eigen::Vector3d(1.28, 0.2, 0.0));
  zmp.emplace_back(Eigen::Vector3d(3.28, 0.2, 0.0));

  preview_control.set_reference_zmp(zmp);

  std::ofstream ofs("com.csv");

  std::vector<Eigen::Vector2d> com;
  std::vector<Eigen::Vector2d> ref_zmp;
  while (1) {
    Eigen::Vector2d com_pos;
    Eigen::Vector2d com_vel;
    bool result = preview_control.update(com_pos, com_vel);

    if (!result) break;

    Eigen::Vector3d curr_zmp = preview_control.get_reference_zmp();

    com.emplace_back(com_pos);
    ref_zmp.emplace_back(curr_zmp.block<2, 1>(1, 0).cast<double>());

    ofs << com_pos[0] << "," << com_pos[1] << "," << curr_zmp[1] << "," << curr_zmp[2] << std::endl;
  }
  return 0;
}