/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
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
