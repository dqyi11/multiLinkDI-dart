#include <fstream>
#include "jsoncpp/json/json.h"
#include "load_problem_cmdline.h"
#include "MultiLinkDI.hpp"

int main(int argc, char* argv[])
{
  gengetopt_args_info args;
  if( cmdline_parser( argc, argv, &args ) != 0 ){
    exit(1);
  }

  std::string problemFilename = "problem.json";
  std::string pathFilename = "path.txt";

  if(args.problem_given)
  {
    problemFilename = args.problem_arg;
  }
  if(args.path_given)
  {
    pathFilename = args.path_arg;
  }

  Json::Reader reader;
  Json::Value root;
  std::ifstream problemFile(problemFilename, std::ifstream::binary);
  bool parseSuccess = reader.parse(problemFile, root, false);
  if(parseSuccess)
  {

    int link_num = root["link_num"].asInt();
    int dimension = link_num * 2;

    Eigen::VectorXd startVec(dimension);
    Eigen::VectorXd goalVec(dimension);
    Json::Value startVal = root["start"];
    Json::Value goalVal = root["goal"];
    for(int i=0;i<startVal.size();i++)
    {
      startVec[i] = startVal[i].asDouble();
      goalVec[i] = goalVal[i].asDouble();
    }

    Eigen::Vector3d diPos;
    Json::Value diPosVal = root["di_pos"];
    for(int i=0;i<diPosVal.size();i++)
    {
      diPos[i] = diPosVal[i].asDouble();
    }

    MultiLinkDI di(link_num, diPos);

    Json::Value obstacles = root["obstacles"];
    for(int i=0;i<obstacles.size();i++)
    {
      std::string type = obstacles[i].get("type","").asString();
      if(type == "hypercube")
      {
        Json::Value centerVal, sizeVal;

        Eigen::Vector3d obs_center;
        Eigen::Vector3d obs_size;

        centerVal = obstacles[i].get("center", 0);
        for(int j=0;j<centerVal.size();j++)
        {
          obs_center[j] = centerVal[j].asDouble();
        }
        sizeVal = obstacles[i].get("size",0);
        for(int j=0;j<sizeVal.size();j++)
        {
          obs_size[j] = sizeVal[j].asDouble();
        }
        di.addCube(obs_center, obs_size);
      }
    }

    di.initVisualization();
  }

}
