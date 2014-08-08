
#include "akin/Robot.h"
#include "osgAkin/AkinCallback.h"
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "HuboKin/DrcHubo.h"

using namespace akin;
using namespace std;

const int w = 2;
const int n = w+1;
const int m = 2;

Robot* create_simple_robot()
{
    Robot* robot = new Robot;
    
    robot->createRootLink("root");
    robot->link(0).mass = 0;
    robot->link(0).com = Translation(0.5);
    
    robot->createJointLinkPair(robot->link(0), "link1", "joint1",
                               Transform(Translation(1,0,0)), Vec3(0,0,1),
                               Joint::REVOLUTE, -M_PI, M_PI);
    robot->link(1).mass = 1;
    robot->link(1).com = Translation(0.5);
    
    robot->createJointLinkPair(robot->link(1), "link2", "joint2",
                               Transform(Translation(1,0,0)), Vec3(0,0,1),
                               Joint::REVOLUTE, -M_PI, M_PI);
    robot->link(2).mass = 1;
    robot->link(2).com = Translation(0.5);
    
//    robot->createJointLinkPair(robot->link(2), "link3", "joint3",
//                               Transform(Translation(1,0,0)), Vec3(0,0,1),
//                               Joint::REVOLUTE, -M_PI, M_PI);
//    robot->link(2).mass = 1;
//    robot->link(2).com = Translation(0.5);
    
    return robot;
}

void mangle_mass_properties(Robot* model)
{
    double max_mass_error_factor = 0.05;
    double max_com_error = 0.01;
//    int resolution = 100000;
//    for(size_t i=1; i<model->numLinks(); ++i)
//    {
//        double error_bound = max_mass_error_factor*model->link(i).mass;
//        model->link(i).mass += (2*((double)(rand()%resolution)/(double)(resolution-1))-1)
//                               * error_bound;
//        for(size_t k=0; k<w; ++k)
//            model->link(i).com[k] += (2*((double)(rand()%resolution)/(double)(resolution-1))-1)
//                                     * max_com_error;
//    }
    
    for(size_t i=1; i<model->numLinks(); ++i)
    {
        model->link(i).mass += max_mass_error_factor*model->link(i).mass;
        model->link(i).com[0] += max_com_error;
    }
}

Eigen::VectorXd compute_model_error(Robot* model, Robot* actual)
{
    Eigen::VectorXd error(n*(model->numLinks()-1));
    
    for(size_t i=0; i<model->numLinks()-1; ++i)
    {
        error.block<w,1>(n*i,0) = actual->link(i+1).com.respectToRef().block<w,1>(0,0) 
                - model->link(i+1).com.respectToRef().block<w,1>(0,0);
        error[n*i+w] = actual->link(i+1).mass - model->link(i+1).mass;
    }
    
    
    return error;
}

void generate_regression_matrices(Eigen::MatrixXd& X, Eigen::VectorXd& y,
                                  const std::vector<Eigen::VectorXd>& configs,
                                  Robot* model, Robot* actual)
{
    size_t C = configs.size();
    size_t N = model->numLinks()-1;
    X.resize(m*C+1, n*N);
    y.resize(m*C+1);
    
    for(size_t s=0; s<C; ++s)
    {
        for(size_t j=0; j<model->numJoints(); ++j)
        {
            model->joint(j).value(configs[s][j]);
            actual->joint(j).value(configs[s][j]);
        }
        Translation p_robot = actual->com().respectToWorld();
        
        for(size_t i=0; i<N; ++i)
        {
            double mi = model->link(i+1).mass;
            X.block<m,w>(m*s,n*i) = mi*model->link(i+1).respectToWorld()
                                    .rotation().matrix().block<m,w>(0,0);
            X.block<m,1>(m*s,n*i+w) = (model->link(i+1).com.respectToWorld()
                                       - p_robot).block<m,1>(0,0);
        }
        
        Eigen::Vector2d v(0,0);
        for(size_t i=0; i<N; ++i)
            v += (p_robot-model->link(i+1).com.respectToWorld()).block<m,1>(0,0)
                    *model->link(i+1).mass;
        y.block<m,1>(m*s,0) = v;
    }
    
    for(size_t i=0; i<N; ++i)
    {
        X.block<1,w>(m*C,n*i) = Eigen::Vector3d::Zero().transpose().block<1,w>(0,0);
        X(m*C,n*i+w) = 1;
    }
    
    double sum_m = 0;
    for(size_t i=0; i<N; ++i)
        sum_m += model->link(i+1).mass;
    y[m*C] = actual->mass() - sum_m;
}

std::vector<Eigen::VectorXd> generate_configs(Robot* model)
{
    size_t J = model->numJoints();
    
    Eigen::VectorXi alt(J); alt.setOnes();
    std::vector<Eigen::VectorXd> keys;
    Eigen::VectorXd key(J);
    bool crawling = true;
    while(crawling)
    {
        for(int i=0; i<alt.size(); ++i)
        {
            if(alt[i] == -2)
            {
                if(i+1==(int)J)
                {
                    crawling = false;
                    break;
                }
                    
                alt[i+1] -= 1;
                for(int j=i; j>=0; --j)
                    alt[j] = 1;
            }
        }
        
        if(!crawling)
            break;
        
        for(int i=0; i<alt.size(); ++i)
        {
            if(alt[i]==1)
                key[i] = model->joint(i).max();
            else if(alt[i]==0)
                key[i] = 0;
            else if(alt[i]==-1)
                key[i] = model->joint(i).min();
        }
        keys.push_back(key);
        
        alt[0] -= 1;
    }
    
    size_t interp=10;
    std::vector<Eigen::VectorXd> configs;
    configs.reserve(interp*keys.size()+1);
    Eigen::VectorXd config(J);
    for(size_t k=1; k<keys.size(); ++k)
    {
        for(size_t i=0; i<interp; ++i)
        {
            config = (keys[k]-keys[k-1])*((double)i)/((double)interp)+keys[k-1];
            configs.push_back(config);
        }
    }
    configs.push_back(keys.back());
    
    std::cout << "configs size: " << configs.size() << std::endl;
    return configs;
}

Eigen::VectorXd perform_regression(const Eigen::MatrixXd& X, const Eigen::VectorXd& y)
{
    return (X.transpose()*X).inverse()*X.transpose()*y;
}

void apply_delta_model(Robot* model, const Eigen::VectorXd& b)
{
    for(size_t i=0; i<model->numLinks()-1; ++i)
    {
        model->link(i+1).com.block<w,1>(0,0) += b.block<w,1>(n*i,0);
        model->link(i+1).mass += b[n*i+w];
    }
}

int main(int , char* [])
{
    srand(time(NULL));
    
    Robot* actual = create_simple_robot();
    Robot* model = create_simple_robot();
    mangle_mass_properties(model);
    Eigen::VectorXd error = compute_model_error(model, actual);
    
    std::vector<Eigen::VectorXd> configs = generate_configs(model);
//    std::cout << "configs:\n";
//    for(size_t i=0; i<configs.size(); ++i)
//        std::cout << configs[i].transpose() << std::endl;
//    std::cout << "\n";
    
    Eigen::MatrixXd X;
    Eigen::VectorXd y;
    Eigen::VectorXd b;
    
    double clamp = 0.01;
    Eigen::VectorXd original_error = compute_model_error(model, actual);
    for(size_t i=0; i<20; ++i)
    {
        generate_regression_matrices(X, y, configs, model, actual);
        b = perform_regression(X, y);
        std::cout << b.transpose() << std::endl;
        double b_norm = b.norm();
        if(b_norm > clamp)
            b *= clamp/b_norm;
        error = compute_model_error(model, actual);
        std::cout << error.transpose() << std::endl;
        std::cout << b.transpose() << std::endl;
        apply_delta_model(model, b);
        std::cout << "(" << i << ")\t" << original_error.norm() << " --> " << error.norm() << std::endl;
        Eigen::FullPivLU<Eigen::MatrixXd> decomp(X.transpose()*X);
        std::cout << "Rank: " << decomp.rank() << "\t(" << (X.transpose()*X).cols() << ")" << std::endl;
        Eigen::EigenSolver<Eigen::MatrixXd> es(X.transpose()*X);
        std::cout << "EVal: " << es.eigenvalues().transpose() << std::endl;
        std::cout << "\n";
    }
    
//    std::cout << "(" << (original_error-error).norm() << ")\t" 
//              << (original_error-error).transpose() << std::endl;
//    std::cout << original_error.norm() << " --> " << error.norm() << std::endl;
    
//    generate_regression_matrices(X, y, configs, model, actual);
    
//    b = perform_regression(X, y);
//    std::cout << "X\n" << X << std::endl;
//    std::cout << "\ny: " << y.transpose() << std::endl;
    
//    std::cout << b.transpose() << std::endl;
//    std::cout << "\n" << error.transpose() << std::endl;
    
    return 0;
}
