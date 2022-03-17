//
// Copyright (C) 2018 Christoph Sommer <sommer@ccs-labs.org>
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

#include "veins_inet/VeinsInetSampleApplication.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"

#include "inet/common/packet/printer/PacketPrinter.h"

#include "inet/common/StringFormat.h"


#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"

#include "veins_inet/VeinsInetSampleMessage_m.h"

using namespace inet;

Define_Module(VeinsInetSampleApplication);

VeinsInetSampleApplication::VeinsInetSampleApplication()
{
    std::cout << "VeinsInetSampleApplication activated!";
}

bool VeinsInetSampleApplication::startApplication()
{
    // host[0] should stop at t=20s
    if (getParentModule()->getIndex() == 0)
    {
        auto callback = [this]()
        {
            getParentModule()->getDisplayString().setTagArg("i", 1, "green");

            //traciVehicle->setDecel(5);

            traciVehicle->setSpeed(0);

            auto payload = makeShared<VeinsInetSampleMessage>();
            payload->setChunkLength(B(100));
            payload->setRoadId(traciVehicle->getRoadId().c_str());
            //std::string str = (const char*)traciVehicle->getSpeed();
            //payload->getVehicleSpeed(str.c_str());
            //setRoadId(traciVehicle->getRoadId().c_str());

            payload->setRoadSpeed(traciVehicle->getSpeed());
            payload->setAcceleration(traciVehicle->getAccel());
            payload->setRoadHumidity("80");


            //speedPayload(payload);

            timestampPayload(payload);


            //string newstr = traciVehicle->getSpeed();
            //std::string info = "speed: 45, acceleration: 5, humidity: 50";
            auto packet = createPacket("obstacle!");
            haveForwarded = false;
            packet->insertAtBack(payload);
            ////Packet(const char *name, const Ptr<const Chunk>& content);
            //packet->insertAtBack(traciVehicle->getSpeed());
            sendPacket(std::move(packet));

            // host should continue after 30s
            auto callback = [this]()
            {
                traciVehicle->setSpeed(-1);
                //traciVehicle->setSpeed(10);
            };
            timerManager.create(veins::TimerSpecification(callback).oneshotIn(SimTime(12, SIMTIME_S)));
        };
        timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(15, SIMTIME_S)));
    }

    if (getParentModule()->getIndex() == 4)
    {
        auto callback = [this]()
        {
            getParentModule()->getDisplayString().setTagArg("i", 1, "red");

            //traciVehicle->setDecel(5);

            traciVehicle->setSpeed(0);

            auto payload = makeShared<VeinsInetSampleMessage>();
            payload->setChunkLength(B(100));
            payload->setRoadId(traciVehicle->getRoadId().c_str());

            timestampPayload(payload);


            payload->setRoadSpeed(traciVehicle->getSpeed());
            payload->setAcceleration(traciVehicle->getAccel());
            payload->setRoadHumidity("40");


            auto packet = createPacket("accident!");
            haveForwarded = false;

            packet->insertAtBack(payload);
            sendPacket(std::move(packet));

            // host should continue after 30s
            auto callback = [this]()
            {
                traciVehicle->setSpeed(-1);
                //traciVehicle->setSpeed(10);
            };
            timerManager.create(veins::TimerSpecification(callback).oneshotIn(SimTime(20, SIMTIME_S)));
        };
        timerManager.create(veins::TimerSpecification(callback).oneshotAt(SimTime(24, SIMTIME_S)));
    }

    return true;
}

bool VeinsInetSampleApplication::stopApplication()
{
    return true;
}

VeinsInetSampleApplication::~VeinsInetSampleApplication()
{
}

void VeinsInetSampleApplication::processPacket(std::shared_ptr<inet::Packet> pk)
{
    auto payload = pk->peekAtFront<VeinsInetSampleMessage>();

    EV_INFO << "Received packet: " << payload << endl;



    getParentModule()->getDisplayString().setTagArg("i", 1, "green");

    traciVehicle->changeRoute(payload->getRoadId(), 999.9);

    std::cout << "speed: " << payload->getRoadSpeed();
    std::cout << "  " << "acceleration: " << payload->getAcceleration();
    std::cout << "  " << "humidity: " << payload->getRoadHumidity() << endl;

    if (haveForwarded) return;

    auto packet = createPacket("Got it!");
    packet->insertAtBack(payload);
    sendPacket(std::move(packet));

    haveForwarded = true;
}
