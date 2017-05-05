# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Property
*
* IDL structure to set/show advanced parameters.
*/
struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* testingGraspmodule_IDL
*
* IDL Interface to \ref testing-module services.
*/

service testingGraspmodule_IDL
{

    bool set_streaming_mode(1: string entry);

    bool  set_hand(1: string entry);
}


