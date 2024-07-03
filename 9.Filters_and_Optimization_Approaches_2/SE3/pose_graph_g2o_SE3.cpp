#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << std::endl;
    return 1;
  }
  std::ifstream fin(argv[1]);
  if (!fin) {
    std::cout << "File " << argv[1] << " does not exist." << std::endl;
    return 1;
  }

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  int vertexCnt = 0, edgeCnt = 0;
  while (!fin.eof()) {
    std::string name;
    fin >> name;
    if (name == "VERTEX_SE3:QUAT") {
      g2o::VertexSE3 *v = new g2o::VertexSE3();
      int index = 0;
      fin >> index;
      v->setId(index);
      v->read(fin);
      optimizer.addVertex(v);
      vertexCnt++;
      if (index == 0) {
        v->setFixed(true);
      }
    } else if (name == "EDGE_SE3:QUAT") {
      g2o::EdgeSE3 *e = new g2o::EdgeSE3();
      int idx1, idx2;
      fin >> idx1 >> idx2;
      e->setId(edgeCnt++);
      e->setVertex(0, optimizer.vertex(idx1));
      e->setVertex(1, optimizer.vertex(idx2));
      e->read(fin);
      optimizer.addEdge(e);
    }
    if (!fin.good()) break;
  }

  std::cout << "Total vertices: " << vertexCnt << std::endl;
  std::cout << "Total edges: " << edgeCnt << std::endl;

  std::cout << "Optimizing..." << std::endl;
  optimizer.initializeOptimization();
  optimizer.optimize(30);

  std::cout << "Saving optimization results..." << std::endl;
  optimizer.save("result.g2o");


  return 0;
}


