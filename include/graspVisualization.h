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

/**
  * This class shows the computed grasping pose and trajectory overlapped
  * to the camera images.
  */
/*******************************************************************************/
class GraspVisualization : public yarp::os::RateThread
{
protected:
    // Gaze variables
    std::string eye;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;
    yarp::sig::Matrix K,H;

    // pose and trajectory variables
    yarp::sig::Vector poseR, poseL, solR, solL;
    yarp::sig::Vector hand_in_poseL, hand_in_poseR;
    std::deque<yarp::sig::Vector> trajectory_right;
    std::deque<yarp::sig::Vector> trajectory_left;
    yarp::sig::Vector point2D, point, point1, superq;
    std::deque<yarp::sig::Vector> trajectory;
    yarp::sig::Matrix R;

public:

    // Options for the visualizer
    double t_vis;
    bool show_hand;
    bool look_object;
    bool show_only_pose;
    std::string left_or_right;
    yarp::os::Mutex mutex;
    const yarp::sig::Vector &object;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    // Parameters for visualization and shared variables
    yarp::os::Property vis_par;
    yarp::sig::Vector &hand, &hand1;
    const yarp::os::Property &complete_sol;

    // Port for input and output image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;

    /***********************************************************************/
    GraspVisualization(int rate, const std::string &_eye, yarp::dev::IGazeControl *_igaze,
                       const yarp::sig::Matrix _K,  std::string left_or_right, const yarp::os::Property &complete_sol,
                       const yarp::sig::Vector &_object,  yarp::sig::Vector &hand,  yarp::sig::Vector &hand1, yarp::os::Property &vis_par );

    /* Overlap superquadric on the camera image */
    /***********************************************************************/
    void addSuperq(const yarp::sig::Vector &x, yarp::sig::ImageOf<yarp::sig::PixelRgb> &imgOut,const int &col);

    /* Convert 3D point to 3D point */
    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /* Show the computed trajectory */
    /***********************************************************************/
    bool showTrajectory(const std::string &hand);

    /* Init function of the thread */
    /***********************************************************************/
    virtual bool threadInit();

    /* Run function of the thread */
    /***********************************************************************/
    virtual void run();

     /* release function of the thread */
    /***********************************************************************/
    virtual void threadRelease();

     /* Get time required for showing the superquadrics and poses */
    /***********************************************************************/
    double getTime();

     /* Acquire poses from other threads */
    /***********************************************************************/
    void getPoses(const yarp::os::Property &poses);

     /* Set parameters for visualization */
    /***********************************************************************/
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /* Set parameters used for visualization */
    /***********************************************************************/
    yarp::os::Property getPar();
};

#endif
