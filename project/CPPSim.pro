TEMPLATE = app
CONFIG += c++11 console
CONFIG -= qt

TARGET = CPPSim

INCLUDEPATH += ../src
INCLUDEPATH += ../lib

SOURCES += ../src/*.cpp
SOURCES += ../src/Drawing/*.cpp
SOURCES += ../src/Math/*.cpp
SOURCES += ../src/Simulation/*.cpp
SOURCES += ../src/Utility/*.cpp
SOURCES += ../src/MavlinkNode/*.cpp

HEADERS += ../src/*.h
HEADERS += ../src/Drawing/*.h
HEADERS += ../src/Math/*.h
HEADERS += ../src/Simulation/*.h
HEADERS += ../src/Utility/*.h
HEADERS += ../src/MavlinkNode/*.h
HEADERS += ../lib/matrix/*.hpp
HEADERS += ../lib/mavlink/*.h
HEADERS += ../lib/mavlink/common/*.h

LIBS += -lglut -lGLU -lGL -lpthread

QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-local-typedefs

DISTFILES += \
    ../config/traj/AttitudeTest.txt \
    ../config/traj/CircleNoFF.txt \
    ../config/traj/FigureEight.txt \
    ../config/traj/FigureEightFF.txt \
    ../config/traj/HelixNoFF.txt \
    ../config/traj/HelixUpDownNoFF.txt \
    ../config/traj/Origin.txt \
    ../config/traj/SpiralNoFF.txt \
    ../config/traj/Square.txt \
    ../config/traj/Turn.txt \
    ../config/01_Intro.txt \
    ../config/02_AttitudeControl.txt \
    ../config/03_PositionControl.txt \
    ../config/04_Nonidealities.txt \
    ../config/05_TrajectoryFollow.txt \
    ../config/06_SensorNoise.txt \
    ../config/07_AttitudeEstimation.txt \
    ../config/08_PredictState.txt \
    ../config/09_PredictCovariance.txt \
    ../config/10_MagUpdate.txt \
    ../config/11_GPSUpdate.txt \
    ../config/QuadControlParams.txt \
    ../config/QuadEstimatorEKF.txt \
    ../config/QuadPhysicalParams.txt \
    ../config/Scenarios.txt \
    ../config/SimulatedSensors.txt \
    ../config/Simulation.txt \
    ../config/X_DebugAttitudeInterp.txt \
    ../config/X_MonteCarloTest.txt \
    ../config/X_Scenarios.txt \
    ../config/X_TestManyQuads.txt \
    ../config/X_TestMavlink.txt \
    ../config/traj/MakeCircleTrajectory.py \
    ../config/traj/MakeHelixTrajectory.py \
    ../config/traj/MakeHelixUpDownTrajectory.py \
    ../config/traj/MakePeriodicTrajectory.py \
    ../config/traj/MakeSpiralTrajectory.py
