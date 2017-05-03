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

#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <string>
#include <deque>
#include <map>


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <opencv2/opencv.hpp>

/*******************************************************************************/
class GraspVisualization : public yarp::os::RateThread
{
protected:
    std::string eye;
    std::string left_or_right;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;
    yarp::sig::Matrix K,H;

    yarp::sig::Vector poseR, poseL;

public:

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /***********************************************************************/
    GraspVisualization(int rate, const std::string &_eye,std::string left_or_right,
                        yarp::dev::IGazeControl *_igaze, const yarp::sig::Matrix _K);

    /***********************************************************************/
    bool showPoses(yarp::sig::Vector &pose, yarp::sig::Vector &pose2, const int &n_poses, int change_color);

    /***********************************************************************/
    void addSuperq(const yarp::sig::Vector &x, yarp::sig::ImageOf<yarp::sig::PixelRgb> &imgOut,const int &col);

    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /***********************************************************************/
    bool showTrajectory();

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void getObjectSuperq(yarp::os::Property &superq);
    
    /***********************************************************************/
    void getPoses(yarp::os::Property &poses);

    /***********************************************************************/
    double getTime();

};

#endif
