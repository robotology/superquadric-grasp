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
#include <cmath>
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <opencv2/opencv.hpp>


#include "src/testingGraspmodule_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class sendSuperq : public RFModule,
                    testingGraspmodule_IDL
{
    RpcClient graspRpc;
    RpcServer portRpc;

    BufferedPort<Property > superqPort;

    Mutex mutex;

    bool streaming;
    string hand;
    Vector sol;

    ResourceFinder *rf;

public:

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    bool  set_hand(const string &entry)
    {
        if (entry=="right" || entry=="left" || entry=="both")
        {
            hand=entry;
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool getperiod()
    {
        return 0.0;
    }

    /**********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        hand=rf.check("hand", Value("right")).asString();

        readSuperq("object", sol, 11, this->rf);

        cout<<"sol"<<sol.toString()<<endl;

        portRpc.open("/testing-graspmodule/rpc");
        graspRpc.open("/testing-graspmodule/superq:rpc");

        attach(portRpc);
        return true;
    }

    /****************************************************************/
    bool readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
    {
        if (Bottle *b=rf->find(name_obj.c_str()).asList())
        {
            if (b->size()>=dimension)
            {
                for(size_t i=0; i<b->size();i++)
                    x.push_back(b->get(i).asDouble());
            }
            return true;
        }
    }


    /**********************************************************************/
    bool close()
    {
        if (portRpc.asPort().isOpen())
            portRpc.close();

        if (graspRpc.asPort().isOpen())
            graspRpc.close();

        return true;
    }

    /**********************************************************************/
    bool updateModule()
    {
        if (norm(sol)>0.0)
        {
            Bottle cmd, reply;
            cmd.addString("get_grasping_pose");

            Bottle &b1=cmd.addList();
            Bottle &b2=b1.addList();
            b2.addString("dimensions");
            Bottle &b3=b2.addList();
            b3.addDouble(sol[0]); b3.addDouble(sol[1]); b3.addDouble(sol[2]);

  
            Bottle &b5=b1.addList();
            b5.addString("exponents");
            Bottle &b6=b5.addList();
            b6.addDouble(sol[3]); b6.addDouble(sol[4]);

  
            Bottle &b8=b1.addList();
            b8.addString("center");
            Bottle &b9=b8.addList();
            b9.addDouble(sol[5]); b9.addDouble(sol[6]); b9.addDouble(sol[7]);

    
            Bottle &b11=b1.addList();
            b11.addString("orientation");
            Bottle &b12=b11.addList();
            Vector orient=dcm2axis(euler2dcm(sol.subVector(8,10)));
            b12.addDouble(orient[0]); b12.addDouble(orient[1]); b12.addDouble(orient[2]); b12.addDouble(orient[3]);

            cmd.addString(hand);

            cout<<"Command asked "<<cmd.toString()<<endl;


            graspRpc.write(cmd, reply);
            cout<<"Received solution: "<<reply.toString()<<endl;
        }

        return true;
    }
};

/**********************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    sendSuperq mod;
    ResourceFinder rf;
    rf.setDefaultContext("testing-graspmodule");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
