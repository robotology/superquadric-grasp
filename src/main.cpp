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

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>

#include <yarp/os/Port.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>
#include <iCub/perception/sensors.h>
#include <iCub/perception/tactileFingers.h>

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;
using namespace iCub::iKin;

class Grasping: public RFModule
{
protected:
    AFFACTIONPRIMITIVESLAYER *action;

    PolyDriver robotDevice;
    PolyDriver robotDevice2;

    ICartesianControl *icart_arm;
    IEncoders *enc;

    string robot;
    string left_or_right;

    deque<Vector> trajectory;
    Vector pose;


public:

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
        {
            yError("Robot name not provided!");
            return false;
        }

        left_or_right=rf.find("hand").asString().c_str();
        if(rf.find("hand").isNull())
        {
            yError("Hand not provided!");
            return false;
        }

        Property option_arm("(device cartesiancontrollerclient)");
        option_arm.put("remote","/"+robot+"/cartesianController/"+left_or_right+"_arm");
        option_arm.put("local","/superquadric-grasping/cartesian/"+left_or_right+"_arm");

        robotDevice.open(option_arm);
        if (!robotDevice.isValid())
        {
            yErr("Device index not available!");
            return false;
        }

        robotDevice.view(icart_arm);

        Property option_arm2("(device remote_controlboard)");
        option_arm2.put("remote","/"+robot+"/"+left_or_right+"_arm");
        option_arm2.put("local","/superquadric-grasping/joint/"+left_or_right+"_arm");

        robotDevice2.open(option_arm2);
        if (!robotDevice2.isValid())
        {
            yErr("Device not available!");
            return false;
        }

        robotDevice2.view(enc);

        configGrasp(rf);

        pose.resize(6,0.0);



    }

    /****************************************************************/
    bool close()
    {
        robotDevice.close();
        robotDevice2.close();

        return true;

    }

    /****************************************************************/
    bool interruptModule()
    {
        robotDevice.close();
        robotDevice2.close();

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        pose=computeSuperq();

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }








};
