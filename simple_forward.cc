#include "stage.hh"
using namespace Stg;

static const double cruisespeed = 0.4;

typedef struct{
  ModelPosition* pos;
  int steps;
} robot_t;

int PositionUpdate( Model* mod, robot_t* robot );

extern "C" int Init( Model* mod, CtrlArgs* args ){
  robot_t* robot = new robot_t;

  robot->steps = 0;

  robot->pos = (ModelPosition*)mod;

  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->pos->Subscribe();

  robot->pos->SetXSpeed( cruisespeed );
//  robot->pos->SetTurnSpeed( 0 );

  puts("Finished initializing!");
  return 0;
}

int PositionUpdate( Model* mod, robot_t* robot ){
  if (robot->steps == 30){
    puts("Stopping...");
    robot->pos->SetXSpeed( 0.0 );
    return 1;
  }
  printf( "%d\n", robot->steps );
  robot->steps++;
  return 0;
}
