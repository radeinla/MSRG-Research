#include "stage.hh"
using namespace Stg;

extern "C" int Init( Model* mod ){
  World* world = mod->GetWorld();
  Model* model = world->GetModel("a");
  std::cout << model;
  return 0;
}
