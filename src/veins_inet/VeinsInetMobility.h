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

#pragma once

namespace omnetpp {
}
using namespace omnetpp;

#include "inet/mobility/base/MobilityBase.h"

#include "veins_inet/veins_inet.h"

#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

namespace veins {

class VEINS_INET_API VeinsInetMobility : public inet::MobilityBase {
public:
    class VEINS_API Statistics {
    public:
        double firstRoadNumber; /**< for statistics: number of first road we encountered (if road id can be expressed as a number) */
        simtime_t startTime; /**< for statistics: start time */
        simtime_t totalTime; /**< for statistics: total time travelled */
        simtime_t stopTime; /**< for statistics: stop time */
        double minSpeed; /**< for statistics: minimum value of currentSpeed */
        double maxSpeed; /**< for statistics: maximum value of currentSpeed */
        double totalDistance; /**< for statistics: total distance travelled */
        double totalCO2Emission; /**< for statistics: total CO2 emission */

        void initialize();
        void watch(cSimpleModule& module);
        void recordScalars(cSimpleModule& module);
    };

    //static const simsignal_t collisionSignal;
    //const static simsignal_t parkingStateChangedSignal;  //this is already handled in a different way

    VeinsInetMobility();

    virtual ~VeinsInetMobility();

    /** @brief called by class VeinsInetManager */
    virtual void preInitialize(std::string external_id, const inet::Coord& position, std::string road_id, double speed, double angle);

    virtual void initialize(int stage) override;

    virtual void finish() override;

    /** @brief called by class VeinsInetManager */
    virtual void nextPosition(const inet::Coord& position, std::string road_id, double speed, double angle);

    virtual void changePosition(double speed);

#if INET_VERSION >= 0x0403
    virtual const inet::Coord& getCurrentPosition() override;
    virtual const inet::Coord& getCurrentVelocity() override;
    virtual const inet::Coord& getCurrentAcceleration() override;

    virtual const inet::Quaternion& getCurrentAngularPosition() override;
    virtual const inet::Quaternion& getCurrentAngularVelocity() override;
    virtual const inet::Quaternion& getCurrentAngularAcceleration() override;
#else
    virtual inet::Coord getCurrentPosition() override;
    virtual inet::Coord getCurrentVelocity() override;
    virtual inet::Coord getCurrentAcceleration() override;

    virtual inet::Quaternion getCurrentAngularPosition() override;
    virtual inet::Quaternion getCurrentAngularVelocity() override;
    virtual inet::Quaternion getCurrentAngularAcceleration() override;
#endif

    virtual std::string getExternalId() const;
    virtual TraCIScenarioManager* getManager() const;
    virtual TraCICommandInterface* getCommandInterface() const;
    virtual TraCICommandInterface::Vehicle* getVehicleCommandInterface() const;

protected:
    /** @brief The last velocity that was set by nextPosition(). */
    inet::Coord lastVelocity;

    /** @brief The last angular velocity that was set by nextPosition(). */
    inet::Quaternion lastAngularVelocity;

    mutable TraCIScenarioManager* manager = nullptr; /**< cached value */
    mutable TraCICommandInterface* commandInterface = nullptr; /**< cached value */
    mutable TraCICommandInterface::Vehicle* vehicleCommandInterface = nullptr; /**< cached value */

    std::string external_id; /**< identifier used by TraCI server to refer to this node */




    int accidentCount; /**< number of accidents */

    cOutVector currentPosXVec; /**< vector plotting posx */
    cOutVector currentPosYVec; /**< vector plotting posy */
    cOutVector currentSpeedVec; /**< vector plotting speed */
    cOutVector currentAccelerationVec; /**< vector plotting acceleration */
    cOutVector currentCO2EmissionVec; /**< vector plotting current CO2 emission */

    Statistics statistics; /**< everything statistics-related */

    bool isPreInitialized; /**< true if preInitialize() has been called immediately before initialize() */

    double hostPositionOffset; /**< front offset for the antenna on this car */
    bool setHostSpeed; /**< whether to update the speed of the host (along with its position)  */

    simtime_t lastUpdate; /**< updated by nextPosition() */
    Coord roadPosition; /**< position of front bumper, updated by nextPosition() */
    std::string road_id; /**< updated by nextPosition() */
    Heading heading; /**< updated by nextPosition() */
    VehicleSignalSet signals; /**<updated by nextPosition() */

    cMessage* startAccidentMsg = nullptr;
    cMessage* stopAccidentMsg = nullptr;
    double last_speed;

    bool isParking;

    //void fixIfHostGetsOutside() override; /**< called after each read to check for (and handle) invalid positions */

    /**
     * Returns the amount of CO2 emissions in grams/second, calculated for an average Car
     * @param v speed in m/s
     * @param a acceleration in m/s^2
     * @returns emission in g/s
     */
    double calculateCO2emission(double v, double a) const;

    /**
     * Calculates where the OMNeT++ module position of this car should be, given its front bumper position
     */
    Coord calculateHostPosition(const Coord& vehiclePos) const;



protected:
    virtual void setInitialPosition() override;

    virtual void handleSelfMessage(cMessage* message) override;
};

} // namespace veins

namespace veins {
class VEINS_INET_API VeinsInetMobilityAccess {
public:
    VeinsInetMobility* get(cModule* host)
    {
        VeinsInetMobility* m = FindModule<VeinsInetMobility*>::findSubModule(host);
        ASSERT(m);
        return m;
    };
};
} // namespace veins
