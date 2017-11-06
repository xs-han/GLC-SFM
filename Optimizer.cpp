//
// Created by xushen on 10/23/17.
//

#include "Optimizer.h"
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

void Optimizer::GlobalBundleAdjustment(vector <KeyFrame *> & kfs, vector <MapPoint *> & pc, Mat & cameraMatrix, int iter) {
    bool ROBUST_KERNEL = true;
    bool STRUCTURE_ONLY = false;
    bool DENSE = false;

    cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
    cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
    cout << "DENSE: "<<  DENSE << endl;

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    if (DENSE) {
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else {
        linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    }
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    // camera matrix
    assert(cameraMatrix.type() == CV_64F);
    double focal_length= (cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(1,1)) / 2;
    Vector2d principal_point(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
    g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
    cam_params->setId(0);
    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }

    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
    int vertex_id = 0;
    for (size_t i=0; i<kfs.size(); ++i) {
        Eigen::Matrix3d r;
        cv::cv2eigen(kfs[i]->rmat, r);
        Eigen::Quaterniond q(r);

        assert(kfs[i]->tvec.type() == CV_64F);
        Vector3d trans(kfs[i]->tvec.at<double>(0),
                       kfs[i]->tvec.at<double>(1),
                       kfs[i]->tvec.at<double>(2));
        g2o::SE3Quat pose(q,trans);
        g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(kfs[i]->kfId);
        if (i<2){
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        optimizer.addVertex(v_se3);
        true_poses.push_back(pose);
        if(kfs[i]->kfId >= vertex_id){
            vertex_id = kfs[i]->kfId;
        }
    }

    int true_id=vertex_id + 1;
    int minkfid = (*kfs.begin())->kfId;
    int maxkfid = (*(kfs.end()-1))->kfId;
    unordered_map<int,int> pointid_2_trueid;
    for(int i = 0; i < pc.size(); i++){
        if(pc[i]->good) {
            pointid_2_trueid.insert(make_pair(i,true_id));
            g2o::VertexSBAPointXYZ *v_p = new g2o::VertexSBAPointXYZ();
            v_p->setId(true_id);
            v_p->setMarginalized(true);
            v_p->setEstimate(Vector3d(pc[i]->x, pc[i]->y, pc[i]->z));

            optimizer.addVertex(v_p);
            for(int j = 0; j < pc[i]->kfs.size(); j++){
                if(pc[i]->kfs[j]->kfId > maxkfid || pc[i]->kfs[j]->kfId < minkfid){
                    continue;
                }
                Point2f & p = pc[i]->kfs[j]->kps[pc[i]->kps[j]].pt;
                Vector2d z(p.x, p.y);

                g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                (optimizer.vertices().find(pc[i]->kfs[j]->kfId)->second));
                e->setMeasurement(z);
                e->information() = Matrix2d::Identity();
                if (ROBUST_KERNEL) {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                }
                e->setParameterId(0, 0);
                optimizer.addEdge(e);
            }

            true_id++;
        }
    }

    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    if (STRUCTURE_ONLY){
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:"   << endl;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3)
                points.push_back(v);
        }
        structure_only_ba.calc(points, iter);
    }
    optimizer.optimize(iter);

    for(int i = 0; i < pc.size(); i++){
        if(pc[i]->good){
            int true_id = pointid_2_trueid[i];
            g2o::HyperGraph::VertexIDMap::iterator v_it
                    = optimizer.vertices().find(true_id);
            if (v_it==optimizer.vertices().end()){
                cerr << "Vertex " << true_id << " not in graph!" << endl;
                exit(-1);
            }
            g2o::VertexSBAPointXYZ * v_p
                    = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
            if (v_p==0){
                cerr << "Vertex " << true_id << "is not a PointXYZ!" << endl;
                exit(-1);
            }
            Vector3d newp = v_p->estimate();
            pc[i]->x = newp(0); pc[i]->y = newp(1); pc[i]->z = newp(2);
        }
    }

    for(int i = 0; i < kfs.size(); i++){
        g2o::HyperGraph::VertexIDMap::iterator v_it
                = optimizer.vertices().find(kfs[i]->kfId);
        if (v_it==optimizer.vertices().end()){
            cerr << "Frame Vertex " << kfs[i]->kfId << " not in graph!" << endl;
            exit(-1);
        }
        g2o::VertexSE3Expmap * v_se3
                = dynamic_cast< g2o::VertexSE3Expmap * > (v_it->second);
        if (v_se3==0){
            cerr << "Frame Vertex " << kfs[i]->kfId << "is not a PointXYZ!" << endl;
            exit(-1);
        }
        g2o::SE3Quat newpose = v_se3->estimate();
        Eigen::Matrix3d r = newpose.rotation().matrix();
        Eigen::Vector3d trans = newpose.translation();
        Mat Rmat; eigen2cv(r, Rmat);
        Mat tvec(3,1,CV_64F);
        tvec.at<double>(0) = trans(0);tvec.at<double>(1) = trans(1);tvec.at<double>(2) = trans(2);
        kfs[i]->setRmat(Rmat.clone());
        kfs[i]->setTvec(tvec.clone());
    }
}
