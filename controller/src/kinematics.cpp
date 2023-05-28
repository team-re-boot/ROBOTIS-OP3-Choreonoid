#include "kinematics.hpp"

Kinematics::Kinematics() {}

void Kinematics::forward_kinematics(const int root)
{
  if (root == -1) return;

  if (root != 0) {
    const int parent = links_[root].parent;
    links_[root].p = links_[parent].p + links_[parent].R * links_[root].offset_trans;
    links_[root].R =
      links_[parent].R * get_rotation_matrix(links_[root].joint_axis, links_[root].q);
  }
  // recursive call
  forward_kinematics(links_[root].sibling);
  forward_kinematics(links_[root].child);
}

Eigen::Vector3d Kinematics::rotation_matrix_to_euler(const Eigen::Matrix3d matrix)
{
  double alpha = (matrix(0, 0) + matrix(1, 1) + matrix(2, 2) - 1.0) / 2.0;
  double theta;
  Eigen::Vector3d vector(Eigen::Vector3d::Zero());

  if (std::fabs(alpha - 1) < 1e-06) return Eigen::Vector3d::Zero();

  theta = acos(alpha);
  vector << matrix(2, 1) - matrix(1, 2), matrix(0, 2) - matrix(2, 0), matrix(1, 0) - matrix(0, 1);
  return 0.5 * theta / std::sin(theta) * vector;
}

Eigen::VectorXd Kinematics::get_link_error(const Link reference, const Link current)
{
  Eigen::Vector3d p_err = reference.p - current.p;
  Eigen::Vector3d rot_err = current.R * rotation_matrix_to_euler(reference.R - current.R);
  Eigen::VectorXd err(6);
  err << p_err, rot_err;
  return err;
}

std::vector<int> Kinematics::find_route(const int from, const int to)  // set joint ID
{
  std::vector<int> joint_path;
  int num = to;

  while (num != from) {
    joint_path.emplace_back(num);
    num = links_[num].parent;
  }
  std::reverse(joint_path.begin(), joint_path.end());
  return joint_path;
}

Eigen::MatrixXd Kinematics::jacobian(const std::vector<int> joint_path)
{
  std::size_t size = joint_path.size();
  Eigen::Vector3d target = links_[joint_path.back()].p;
  Eigen::MatrixXd J;
  J.resize(6, size);

  for (std::size_t i = 0; i < size; i++) {
    const int j = joint_path[i];
    Eigen::Vector3d a = links_[j].R * links_[j].joint_axis;
    Eigen::Vector3d b = a.cross(target - links_[j].p);
    J(0, i) = b(0);
    J(1, i) = b(1);
    J(2, i) = b(2);
    J(3, i) = a(0);
    J(4, i) = a(1);
    J(5, i) = a(2);
  }

  return J;
}

// TODO: redundant joint
bool Kinematics::inverse_kinematics(const Link base, const Link target)
{
  Eigen::MatrixXd J;
  Eigen::VectorXd delta_q;

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver;
  const double damping_constant_sqr = 1.0e-12;
  const double lambda = 0.5;
  const int max_iteration = 50;

  // call forward kinematics
  forward_kinematics(base.id);

  std::vector<int> joint_path = find_route(base.id, target.id);
  const std::size_t joint_size = joint_path.size();

  J.resize(6, joint_size);
  J = Eigen::MatrixXd::Zero(joint_size, joint_size);
  delta_q.resize(joint_size);
  delta_q = Eigen::VectorXd::Zero(joint_size);

  Eigen::VectorXd err(joint_size);
  for (int i = 0; i < max_iteration; i++) {
    J = jacobian(joint_path);
    err = get_link_error(target, links_[target.id]);
    if (err.norm() < 1e-06) return true;

    if (6 < joint_size) {
      Eigen::MatrixXd JJ =
        J * J.transpose() + damping_constant_sqr * Eigen::MatrixXd::Identity(J.rows(), J.rows());
      delta_q = J.transpose() * solver.compute(JJ).solve(err) * lambda;
    } else {
      delta_q = lambda * (J.inverse() * err);
    }

    for (std::size_t j = 0; j < joint_size; j++) {
      links_[joint_path[j]].q += delta_q(j);
    }
    forward_kinematics(base.id);
  }

  return false;
}