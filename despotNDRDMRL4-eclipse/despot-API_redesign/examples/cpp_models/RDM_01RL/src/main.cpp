#include <despot/planner.h>

#include "rdm01RL.h"

using namespace despot;

class MyPlanner: public Planner {
public:
  MyPlanner() {
  }
 
  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = new Rdm();
    return model;
  }
  
  World* InitializeWorld(std::string&  world_type, DSPOMDP* model, option::Option* options)
  {
	return InitializePOMDPWorld(world_type, model, options);
  }

  void InitializeDefaultParameters() {
  }

  std::string ChooseSolver(){
	  return "DESPOT";
  }
};

int main(int argc, char* argv[]) {
  return MyPlanner().RunEvaluation(argc, argv);
}