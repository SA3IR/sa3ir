/*
    Copyright (C) 2020  University of Extremadura, University of Málaga, Blue Ocean Robotics.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

		Author: Adrian Romero, argarces@uma.es
		Maintainer: Renan Freitas, renan028@gmail.com
*/

#pragma once

#include <RoqmeReaderImpl.h>
#include <query_client.hpp>
#include <variant_client.hpp>
#include <mutex>

using namespace Roqme;

class CurrentAction
{
    
public:
    CurrentAction();
    std::string get();
    void set(const std::string& value);
private:
    std::mutex lock;
    std::string action;
};


class DistanceToGoal
{
public:
    DistanceToGoal();
    double get();
    void set(const double& value);
private:
    std::mutex lock;
    double distance;
};


/*
 *  Roqme context listeners
 */
#ifdef SUBSCRIBE_TO_INT_CONTEXTS
class IntContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeIntContext>
{
	public:
		void dataAvailable(const RoqmeDDSTopics::RoqmeIntContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
};
#endif
#ifdef SUBSCRIBE_TO_UINT_CONTEXTS
class UIntContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeUIntContext>
{
	public:
		void dataAvailable(const RoqmeDDSTopics::RoqmeUIntContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
};
#endif
#ifdef SUBSCRIBE_TO_BOOL_CONTEXTS
class BoolContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeBoolContext>
{
	public:
		void dataAvailable(const RoqmeDDSTopics::RoqmeBoolContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
};
#endif
#ifdef SUBSCRIBE_TO_ENUM_CONTEXTS
class EnumContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeEnumContext>
{
    public:
        EnumContextListener(std::shared_ptr<CurrentAction>& currentAction);
		void dataAvailable(const RoqmeDDSTopics::RoqmeEnumContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
        
    private:
        std::shared_ptr<CurrentAction> currentAction_;
};
#endif
#ifdef SUBSCRIBE_TO_EVENT_CONTEXTS
class EventContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeEventContext>
{
	public:
		void dataAvailable(const RoqmeDDSTopics::RoqmeEventContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
};

#endif

#ifdef SUBSCRIBE_TO_DOUBLE_CONTEXTS
class DoubleContextListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeDoubleContext>
{
	public:
        DoubleContextListener(std::shared_ptr<DistanceToGoal>& distance);
        
		void dataAvailable(const RoqmeDDSTopics::RoqmeDoubleContext &data, 
			const dds::sub::SampleInfo &sampleInfo);
        
    private:
        std::shared_ptr<DistanceToGoal> distance_;
};
#endif




/*
 * Roqme reasoner estimation listener
 */

class EstimateListener : public RoqmeDDSListener<RoqmeDDSTopics::RoqmeEstimate>
{
	public:
		EstimateListener(   std::shared_ptr<DistanceToGoal> distance,
                            std::shared_ptr<CurrentAction> currentAction,
                            std::shared_ptr<QueryClient> queryClient,
                            std::shared_ptr<VariantClient> variantClient);
		~EstimateListener() = default;
		void dataAvailable(const RoqmeDDSTopics::RoqmeEstimate &data, 
			const dds::sub::SampleInfo &sampleInfo);
	private:
		std::shared_ptr<QueryClient> query_client_;
		std::shared_ptr<VariantClient> variant_client_;
        std::shared_ptr<DistanceToGoal> distance_;
        std::shared_ptr<CurrentAction> currentAction_;
		double safety_threshold_;
        double power_threshold_;
        double mission_completion_threshold_;
		double tmp_safety_;
        double tmp_power_;
        double tmp_mission_completion_;
		bool flag_power_autonomy_;
        bool flag_safety_;
        bool flag_mission_completion_;
        bool flag_variant_pickup_;
        bool flag_variant_dock_;

	private:
		void changeVelocity(double value);
		void abortCurrentSkill(double value);
        void abortAction(const std::string& variant);
        bool detectingObject(const std::string& currentAction, const std::string& prevAction);
};

/*
 * This class contains all the listeners denfined above
 */

class MironDDSListener
{
	public:
		MironDDSListener(std::shared_ptr<QueryClient> queryClient,
			std::shared_ptr<VariantClient> variantClient);
		~MironDDSListener() = default;

private:
#ifdef SUBSCRIBE_TO_INT_CONTEXTS
	std::unique_ptr<RoqmeIntReader> intReaderPtr;
#endif
#ifdef SUBSCRIBE_TO_UINT_CONTEXTS
	std::unique_ptr<RoqmeUIntReader> uintReaderPtr;
#endif
#ifdef SUBSCRIBE_TO_BOOL_CONTEXTS
	std::unique_ptr<RoqmeBoolReader> boolReaderPtr;
#endif
#ifdef SUBSCRIBE_TO_ENUM_CONTEXTS
	std::unique_ptr<RoqmeEnumReader> enumReaderPtr;
#endif
#ifdef SUBSCRIBE_TO_EVENT_CONTEXTS
	std::unique_ptr<RoqmeEventReader> eventReaderPtr;
#endif
#ifdef SUBSCRIBE_TO_DOUBLE_CONTEXTS
    std::shared_ptr<RoqmeDoubleReader> doubleReaderPtr;
#endif
	std::unique_ptr<RoqmeEstimateReader> estimateReaderPtr;
    std::shared_ptr<DistanceToGoal> distance_;
    std::shared_ptr<CurrentAction> currentAction_;
};
