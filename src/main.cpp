/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>

#include "graspModule.h"

using namespace yarp::os;


/************************************************************************/
int main(int argc, char *argv[])
{
   Network yarp;
   if (!yarp.checkNetwork())
   {
       yError()<<"YARP server not available!";
       return 1;
   }

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("config.ini");
   rf.setDefaultContext("superquadric-grasp");
   rf.setDefault("grasp_model_type","springy");
   rf.setDefault("grasp_model_file_right","grasp_model_right.ini");
   rf.setDefault("grasp_model_file_left","grasp_model_left.ini");
   rf.setDefault("hand_sequences_file","hand_sequences.ini");
   rf.setDefault("name","actionPrimitivesMod");
   rf.configure(argc,argv);

   GraspingModule mod;
   return mod.runModule(rf);
}
