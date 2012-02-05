#include <string>
#include <iostream>
#include <sstream>
#include "stage.hh"
using namespace Stg;

static const double cruisespeed = 0.4;

typedef struct{
  ModelPosition* pos;
  ModelFiducial* fiducial;
  int steps;
} robot_t;

int PositionUpdate( Model* mod, robot_t* robot );
int MSRGBotDetectorUpdate( ModelFiducial* mod, robot_t* robot );

extern "C" int Init( Model* mod, CtrlArgs* args ){
  robot_t* robot = new robot_t;

  robot->steps = 0;

  robot->pos = (ModelPosition*)mod;

  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );
  robot->pos->Subscribe();

  robot->fiducial = (ModelFiducial*)robot->pos->GetUnusedModelOfType( "fiducial" );
  robot->fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)MSRGBotDetectorUpdate, robot->fiducial );
  robot->fiducial->Subscribe();

//  robot->pos->SetXSpeed( cruisespeed );
//  robot->pos->SetTurnSpeed( 0 );

  puts("Finished initializing!");
  return 0;
}

int MSRGBotDetectorUpdate( ModelFiducial* mod, robot_t* robot ){
  std::vector<ModelFiducial::Fiducial>& fids = mod->GetFiducials();
  World* world =  robot->pos->GetWorld();
  for( unsigned int i = 0; i < fids.size(); i++ ){
    std::ostringstream name;
    std::cout << "\n\nfid: " << fids[i].id << "\n\n";
//    name << "msrg" << fids[i].id;
//    ModelPosition* robotInWifiZone = (ModelPosition*) world->GetModel(name.str());
//    if (robotInWifiZone != NULL){
//      puts("found a nearby robot");
//    }
  }
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
