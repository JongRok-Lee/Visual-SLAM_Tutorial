#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <string>

void DrawTrajectory(std::vector<Eigen::Isometry3d> &poses);

int main (int argc, char** argv)
{
  std::vector<Eigen::Isometry3d> poses;

  std::string trajectory_file = argv[1];
  std::ifstream fin(trajectory_file);
  if (!fin) {
    std::cout << "Cannot find trajectory file at " << trajectory_file << std::endl;
    return 1;
  }

  double time, tx, ty, tz, qx, qy, qz, qw;
  while (!fin.eof()) {
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  std::cout<< "Read toal " << poses.size() << " pose entries" << std::endl;

  // Draw Trajectory in pangolin
  DrawTrajectory(poses);
}

void DrawTrajectory(std::vector<Eigen::Isometry3d> &poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
  
  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    for (size_t i = 0; i < poses.size(); ++i) {
      // draw three axes of each pose
      Eigen::Vector3d Ow = poses[i].translation();
      Eigen::Vector3d Xw = poses[i] * (0.1 * Eigen::Vector3d(1, 0, 0));
      Eigen::Vector3d Yw = poses[i] * (0.1 * Eigen::Vector3d(0, 1, 0));
      Eigen::Vector3d Zw = poses[i] * (0.1 * Eigen::Vector3d(0, 0, 1));

      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow(0,0), Ow(1,0), Ow(2,0));
      glVertex3d(Xw(0,0), Xw(1,0), Xw(2,0));

      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow(0,0), Ow(1,0), Ow(2,0));
      glVertex3d(Yw(0,0), Yw(1,0), Yw(2,0));

      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow(0,0), Ow(1,0), Ow(2,0));
      glVertex3d(Zw(0,0), Yw(1,0), Yw(2,0));
      glEnd();
    }

    // draw a connection
    for (size_t i = 0; i < poses.size() - 1; ++i) {
      Eigen::Isometry3d pose1 = poses[i], pose2 = poses[i + 1];

      glBegin(GL_LINES);
      glColor3f(0.0, 0.0, 0.0);
      glVertex3d(pose1.translation()(0,0), pose1.translation()(1,0), pose1.translation()(2,0));
      glVertex3d(pose2.translation()(0,0), pose2.translation()(1,0), pose2.translation()(2,0));
      glEnd();
    }

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));   // sleep 5 ms
  }
}