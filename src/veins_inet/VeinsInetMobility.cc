//
// Copyright (C) 2006-2018 Christoph Sommer <sommer@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

//
// Veins Mobility module for the INET Framework
// Based on inet::MovingMobilityBase of INET Framework v4.0.0
//

#include "veins_inet/VeinsInetMobility.h"

#include "inet/common/INETMath.h"
#include "inet/common/Units.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"

namespace veins {

using namespace inet::units::values;

Register_Class(VeinsInetMobility);


//const simsignal_t TraCIMobility::collisionSignal = registerSignal("org_car2x_veins_modules_mobility_collision");
namespace {
const double MY_INFINITY = (std::numeric_limits<double>::has_infinity ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::max());
}

void VeinsInetMobility::Statistics::initialize()
{
    firstRoadNumber = MY_INFINITY;
    startTime = simTime();
    totalTime = 0;
    stopTime = 0;
    minSpeed = MY_INFINITY;
    maxSpeed = -MY_INFINITY;
    totalDistance = 0;
    totalCO2Emission = 0;
}

void VeinsInetMobility::Statistics::watch(cSimpleModule&)
{
}

void VeinsInetMobility::Statistics::recordScalars(cSimpleModule& module)
{
    if (firstRoadNumber != MY_INFINITY) module.recordScalar("firstRoadNumber", firstRoadNumber);
    module.recordScalar("startTime", startTime);
    module.recordScalar("totalTime", totalTime);
    module.recordScalar("stopTime", stopTime);
    if (minSpeed != MY_INFINITY) module.recordScalar("minSpeed", minSpeed);
    if (maxSpeed != -MY_INFINITY) module.recordScalar("maxSpeed", maxSpeed);
    module.recordScalar("totalDistance", totalDistance);
    module.recordScalar("totalCO2Emission", totalCO2Emission);
}





VeinsInetMobility::VeinsInetMobility()
{
}

VeinsInetMobility::~VeinsInetMobility()
{
    delete vehicleCommandInterface;
}

void VeinsInetMobility::preInitialize(std::string external_id, const inet::Coord& position, std::string road_id, double speed, double angle)
{
    Enter_Method_Silent();
    this->external_id = external_id;
    lastPosition = position;
    lastVelocity = inet::Coord(cos(angle), -sin(angle)) * speed;
    lastOrientation = inet::Quaternion(inet::EulerAngles(rad(-angle), rad(0.0), rad(0.0)));
}

void VeinsInetMobility::initialize(int stage)
{
    MobilityBase::initialize(stage);

    // We patch the OMNeT++ Display String to set the initial position. Make sure this works.
    ASSERT(hasPar("initFromDisplayString") && par("initFromDisplayString"));

    if (stage == 0){
    //currentPosXVec.setName("posx");
    //currentPosYVec.setName("posy");
    currentSpeedVec.setName("speed");
    currentAccelerationVec.setName("acceleration");
    //currentCO2EmissionVec.setName("co2emission");

    statistics.initialize();
    statistics.watch(*this);
    }
}

void VeinsInetMobility::nextPosition(const inet::Coord& position, std::string road_id, double speed, double angle)
{
    Enter_Method_Silent();

    lastPosition = position;
    lastVelocity = inet::Coord(cos(angle), -sin(angle)) * speed;
    lastOrientation = inet::Quaternion(inet::EulerAngles(rad(-angle), rad(0.0), rad(0.0)));

    changePosition(speed);

    // Update display string to show node is getting updates
    auto hostMod = getParentModule();
    if (std::string(hostMod->getDisplayString().getTagArg("veins", 0)) == ". ") {
        hostMod->getDisplayString().setTagArg("veins", 0, " .");
    }
    else {
        hostMod->getDisplayString().setTagArg("veins", 0, ". ");
    }

    emitMobilityStateChangedSignal();
}

void VeinsInetMobility::changePosition(double speed)
{
    // ensure we're not called twice in one time step
    ASSERT(lastUpdate != simTime());

    //Coord nextPos = calculateHostPosition(roadPosition);
    //nextPos.z = move.getStartPosition().z;

    // keep statistics (for current step)
    //currentPosXVec.record(nextPos.x);
    //currentPosYVec.record(nextPos.y);

    // keep statistics (relative to last step)
    if (statistics.startTime != simTime()) {
        simtime_t updateInterval = simTime() - this->lastUpdate;

        //double distance = std::move.getStartPos().distance(nextPos);
        //statistics.totalDistance += distance;
        statistics.totalTime += updateInterval;
        //double speed =
        if (speed != -1) {
            statistics.minSpeed = std::min(statistics.minSpeed, speed);
            statistics.maxSpeed = std::max(statistics.maxSpeed, speed);
            currentSpeedVec.record(speed);
            if (last_speed != -1) {
                double acceleration = (speed - last_speed) / updateInterval;
                //double co2emission = calculateCO2emission(speed, acceleration);
                currentAccelerationVec.record(acceleration);
                //currentCO2EmissionVec.record(co2emission);
                //statistics.totalCO2Emission += co2emission * updateInterval.dbl();
            }
            last_speed = speed;
        }
        else {
            last_speed = -1;
            speed = -1;
        }
    }
    this->lastUpdate = simTime();
}

#if INET_VERSION >= 0x0403
const inet::Coord& VeinsInetMobility::getCurrentPosition()
{
    return lastPosition;
}

const inet::Coord& VeinsInetMobility::getCurrentVelocity()
{
    return lastVelocity;
}

const inet::Coord& VeinsInetMobility::getCurrentAcceleration()
{
    throw cRuntimeError("Invalid operation");
}

const inet::Quaternion& VeinsInetMobility::getCurrentAngularPosition()
{
    return lastOrientation;
}

const inet::Quaternion& VeinsInetMobility::getCurrentAngularVelocity()
{
    return lastAngularVelocity;
}

const inet::Quaternion& VeinsInetMobility::getCurrentAngularAcceleration()
{
    throw cRuntimeError("Invalid operation");
}
#else

inet::Coord VeinsInetMobility::getCurrentPosition()
{
    return lastPosition;
}

inet::Coord VeinsInetMobility::getCurrentVelocity()
{
    return lastVelocity;
}

inet::Coord VeinsInetMobility::getCurrentAcceleration()
{
    throw cRuntimeError("Invalid operation");
}

inet::Quaternion VeinsInetMobility::getCurrentAngularPosition()
{
    return lastOrientation;
}

inet::Quaternion VeinsInetMobility::getCurrentAngularVelocity()
{
    return lastAngularVelocity;
}

inet::Quaternion VeinsInetMobility::getCurrentAngularAcceleration()
{
    throw cRuntimeError("Invalid operation");
}
#endif
void VeinsInetMobility::setInitialPosition()
{
    subjectModule->getDisplayString().setTagArg("p", 0, lastPosition.x);
    subjectModule->getDisplayString().setTagArg("p", 1, lastPosition.y);
    MobilityBase::setInitialPosition();
}

void VeinsInetMobility::finish()
{
    statistics.stopTime = simTime();

    statistics.recordScalars(*this);

    //cancelAndDelete(startAccidentMsg);
    //cancelAndDelete(stopAccidentMsg);

    isPreInitialized = false;
}

void VeinsInetMobility::handleSelfMessage(cMessage* message)
{
}

std::string VeinsInetMobility::getExternalId() const
{
    if (external_id == "") throw cRuntimeError("TraCIMobility::getExternalId called with no external_id set yet");
    return external_id;
}

TraCIScenarioManager* VeinsInetMobility::getManager() const
{
    if (!manager) manager = TraCIScenarioManagerAccess().get();
    return manager;
}

TraCICommandInterface* VeinsInetMobility::getCommandInterface() const
{
    if (!commandInterface) commandInterface = getManager()->getCommandInterface();
    return commandInterface;
}

TraCICommandInterface::Vehicle* VeinsInetMobility::getVehicleCommandInterface() const
{
    if (!vehicleCommandInterface) vehicleCommandInterface = new TraCICommandInterface::Vehicle(getCommandInterface()->vehicle(getExternalId()));
    return vehicleCommandInterface;
}

} // namespace veins
