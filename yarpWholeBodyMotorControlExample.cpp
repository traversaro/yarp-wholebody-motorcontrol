/*
 * Copyright: (C) 2017 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the MIT Licence
 */

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <vector>

class WholeBodyRobotControl
{
public:
    yarp::dev::PolyDriver m_robotDevice;
    int m_nj;

    std::vector<double> m_initialMeasuredTorquesInNm;

    std::vector<double> m_currentEncodersInDegrees;
    std::vector<double> m_currentMeasuredTorquesInNm;

    // This is the interface for reading the encoders
    yarp::dev::IEncoders *m_encs;

    // This is the interface for reading joint torques and setting desired joint torques
    yarp::dev::ITorqueControl *m_trq;

    // This is the interface to switch joint-level control mode on the flight
    // See http://wiki.icub.org/images/c/cf/ICub_Control_Modes_1_1.pdf for a nice
    // overview on the joint-level control mode handling on the iCub
    yarp::dev::IControlMode2 *m_ctrlMode;

    // Configuration
    bool configure();

    // Control loop
    bool readSensorsAndSetTorques();

    // Close method
    bool close();
};

bool WholeBodyRobotControl::configure()
{
    // We create a RemoteControlBoardRemapper class to handle the communication
    // to the controlboards of the robot, throught the yarp::os::PolyDriver factory
    // See http://www.yarp.it/classyarp_1_1dev_1_1RemoteControlBoardRemapper.html
    // for more informations
    yarp::os::Property deviceOptions;
    deviceOptions.put("device","remotecontrolboardremapper");
    yarp::os::Bottle axesNames;
    yarp::os::Bottle & axesList = axesNames.addList();

    // This is the list of the joints (axes, in YARP jargon)
    // that will be exposed by the opened device. Tipically this
    // list are loaded by a configuration files to ensure that the
    // program do not contain hardcoded assumptions on the robot they control

    // Torso
    axesList.addString("torso_pitch");
    axesList.addString("torso_roll");
    axesList.addString("torso_yaw");

    // Left arm
    axesList.addString("l_shoulder_pitch");
    axesList.addString("l_shoulder_roll");
    axesList.addString("l_shoulder_yaw");
    axesList.addString("l_elbow");

    // Right arm
    axesList.addString("r_shoulder_pitch");
    axesList.addString("r_shoulder_roll");
    axesList.addString("r_shoulder_yaw");
    axesList.addString("r_elbow");

    // Left leg
    axesList.addString("l_hip_pitch");
    axesList.addString("l_hip_roll");
    axesList.addString("l_hip_yaw");
    axesList.addString("l_knee");
    axesList.addString("l_ankle_pitch");
    axesList.addString("l_ankle_roll");

    // Right leg
    axesList.addString("r_hip_pitch");
    axesList.addString("r_hip_roll");
    axesList.addString("r_hip_yaw");
    axesList.addString("r_knee");
    axesList.addString("r_ankle_pitch");
    axesList.addString("r_ankle_roll");

    deviceOptions.put("axesNames",axesNames.get(0));
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();

    // This is the list of the YARP controlboardwrapper to which this device
    // will connect. Again, this kind of information are tipically contained in configuration files
    remoteControlBoardsList.addString("/icubSim/torso");
    remoteControlBoardsList.addString("/icubSim/head");
    remoteControlBoardsList.addString("/icubSim/left_arm");
    remoteControlBoardsList.addString("/icubSim/right_arm");
    remoteControlBoardsList.addString("/icubSim/left_leg");
    remoteControlBoardsList.addString("/icubSim/right_leg");
    deviceOptions.put("remoteControlBoards",remoteControlBoards.get(0));
    deviceOptions.put("localPortPrefix","/yarpMotorControlWholeBody");

    // This are option passed to the underlyng remote_controlboard device
    // see http://www.yarp.it/classyarp_1_1dev_1_1RemoteControlBoard.html for more info
    yarp::os::Property & remoteControlBoardsOpts = deviceOptions.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict","on");

    // Actually create the device using the PolyDriver class
    m_robotDevice.open(deviceOptions);

    if (!m_robotDevice.isValid())
    {
        std::cerr << "Problem in opening the remote controlboard remapper." << std::endl;
        return false;
    }

    // Specific features of a given object are exposed through C++ interfaces
    // (i.e. pure virtual classes). The view method of the PolyDriver is just a
    // dynamic_cast on the object created by the factory through  the open method

    bool ok;
    ok = m_robotDevice.view(m_trq);
    ok = ok && m_robotDevice.view(m_encs);
    ok = ok && m_robotDevice.view(m_ctrlMode);

    if (!ok) {
        std::cerr << "Problem in opening the remote controlboard remapper" << std::endl;
        m_robotDevice.close();
        return EXIT_FAILURE;
    }

    // Sanity check on the number of the axis exposed in the controlboard
    m_encs->getAxes(&m_nj);

    if( m_nj != axesList.size() )
    {
        std::cerr << "Mismatch in the number of joints controlled." << std::endl;
        m_robotDevice.close();
        return false;
    }

    // Resize buffers
    m_initialMeasuredTorquesInNm.resize(m_nj,0.0);
    m_currentEncodersInDegrees.resize(m_nj,0.0);
    m_currentMeasuredTorquesInNm.resize(m_nj,0.0);

    // We wait on the encoders readings to make sure that the first sensor message
    // was received by the device.
    // ********************
    // Pay attention that the encoders and all the
    // angular information are expressed in *****DEGREES*****
    // ********************
    while(!m_trq->getTorques(m_initialMeasuredTorquesInNm.data()))
    {
        yarp::os::Time::delay(0.01);
    }

    // The typical initial CONTROL_MODE for the robot is VOCAB_CM_POSITION ,
    // i.e. position loop + trajectory generator

    // We then switch to VOCAB_CM_TORQUE to use a low-level joint torque control
    // Note that while is not shown here, it is possible to change the control mode
    // of each joint independently, i.e. controlling some joints in torque control
    // and some joint in position control . This is useful for example for handling
    // joints that is not possible to control in torque mode (in iCub's case, neck,
    // eyes and hand) but for which it is important to get position feedback
    std::vector<int> desiredControlMode(m_nj, VOCAB_CM_TORQUE);

    ok = m_ctrlMode->setControlModes(desiredControlMode.data());
    // Set an initial reference torque
    m_trq->setRefTorques(m_initialMeasuredTorquesInNm.data());

    if( !ok )
    {
        std::cerr << "Problem in setting control mode." << std::endl;
        m_robotDevice.close();
        return false;
    }

    return true;
}

bool WholeBodyRobotControl::readSensorsAndSetTorques()
{
    // Ignoring return values for now, but they are important IRL to check for timeouts

    // Readings joint sensors
    m_encs->getEncoders(m_currentEncodersInDegrees.data());
    m_trq->getTorques(m_currentMeasuredTorquesInNm.data());

    // Send desired torques (for the sake of example, we just send the initial torques)
    m_trq->setRefTorques(m_initialMeasuredTorquesInNm.data());

    return true;
}

bool WholeBodyRobotControl::close()
{
    // Switch the control mode back to VOCAB_CM_POSITION .
    std::vector<int> desiredControlMode(m_nj, VOCAB_CM_POSITION);

    bool ok = m_ctrlMode->setControlModes(desiredControlMode.data());

    if( !ok )
    {
        std::cerr << "Problem in setting control mode back to position." << std::endl;
    }

    // Close the device
    return m_robotDevice.close();
}


int main(int argc, char *argv[]) 
{
    // This class handles automatically
    // the initialization of the YARP network.
    // It can be substituted by a initial call to
    // yarp::os::Network::init() and a final call
    // to yarp::os::Network::fini()
    yarp::os::Network yarp;

    WholeBodyRobotControl wholeBodyRobotControl;
    
    //////////////////////////////////////////////////////////////////////////
    ////////// DEVICE AND CONTROL (CHANGE CONTROL MODE) CONFIGURATION
    //////////////////////////////////////////////////////////////////////////
    bool ok = wholeBodyRobotControl.configure();
    if (!ok)
    {
        return EXIT_FAILURE;
    }

    //////////////////////////////////////////////////////////////////////////
    ////////// SENSOR AND CONTROL LOOP
    //////////////////////////////////////////////////////////////////////////

    // Run a simple loop of reading encoders + sending torque for 50 times at 100 hz
    for(int i=0; i < 50; i++)
    {
        wholeBodyRobotControl.readSensorsAndSetTorques();

        // Dummy delay
        yarp::os::Time::delay(0.01);
    }

    //////////////////////////////////////////////////////////////////////////
    ////////// CLOSING DEVICE AND CLEANUP
    //////////////////////////////////////////////////////////////////////////
    wholeBodyRobotControl.close();

    return 0;
}