#include "load_problem_cmdline.h"
#include "MultiLinkDI.hpp"
#include "MultiLinkDIUtil.hpp"

int main(int argc, char* argv[])
{
  gengetopt_args_info args;
  if( cmdline_parser( argc, argv, &args ) != 0 ){
    exit(1);
  }

  std::string problemFilename = "problem.json";
  std::string pathFilename = "";

  if(args.problem_given)
  {
    problemFilename = args.problem_arg;
  }
  if(args.path_given)
  {
    pathFilename = args.path_arg;
  }

  std::shared_ptr<MultiLinkDI> di = createMultiLinkDI(problemFilename);
  if(pathFilename!="")
  {
    loadMultiLinkDIPath( di, pathFilename );
  }
  di->initVisualization();
}
