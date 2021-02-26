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

#include "mironDDS_listener.hpp"
#include "change_velocity.hpp"
#include "abort_current_skill.hpp"

using namespace zmqserver;

MironDDSListener::MironDDSListener(std::shared_ptr<QueryClient> queryClient, std::shared_ptr<VariantClient> variantClient)
{
    currentAction_ = std::make_shared<CurrentAction>();
    distance_ = std::make_shared<DistanceToGoal>();
#ifdef SUBSCRIBE_TO_INT_CONTEXTS
	intReaderPtr = std::unique_ptr<RoqmeIntReader>(new RoqmeIntReader(new IntContextListener));
#endif
#ifdef SUBSCRIBE_TO_UINT_CONTEXTS
	uintReaderPtr = std::unique_ptr<RoqmeUIntReader>(new RoqmeUIntReader(new UIntContextListener));
#endif
#ifdef SUBSCRIBE_TO_BOOL_CONTEXTS
	boolReaderPtr = std::unique_ptr<RoqmeBoolReader>(new RoqmeBoolReader(new BoolContextListener));
#endif
#ifdef SUBSCRIBE_TO_ENUM_CONTEXTS
	enumReaderPtr = std::unique_ptr<RoqmeEnumReader>(new RoqmeEnumReader(new EnumContextListener(currentAction_)));
#endif
#ifdef SUBSCRIBE_TO_EVENT_CONTEXTS
	eventReaderPtr = std::unique_ptr<RoqmeEventReader>(new RoqmeEventReader(new EventContextListener));
#endif
#ifdef SUBSCRIBE_TO_DOUBLE_CONTEXTS
    doubleReaderPtr = std::unique_ptr<RoqmeDoubleReader>(new RoqmeDoubleReader(new DoubleContextListener(distance_)));
#endif
	estimateReaderPtr = std::unique_ptr<RoqmeEstimateReader>(
			new RoqmeEstimateReader(new EstimateListener(distance_, currentAction_, queryClient, variantClient)));
}

/* 
 * Roqme estimate listener implementation
 */
EstimateListener::EstimateListener( std::shared_ptr<DistanceToGoal> distance,
                                    std::shared_ptr<CurrentAction> currentAction,
                                    std::shared_ptr<QueryClient> queryClient, 
                                    std::shared_ptr<VariantClient> variantClient) : 
    distance_(distance),
    currentAction_(currentAction),
    query_client_(queryClient),
    variant_client_(variantClient),
    safety_threshold_(0.1),
    power_threshold_(0.1),
    mission_completion_threshold_(0.1),
    tmp_safety_(-1),
    tmp_power_(-1),
    tmp_mission_completion_(-1),
    flag_power_autonomy_(false),
    flag_safety_(false),
    flag_mission_completion_(false),
    flag_variant_pickup_(false),
    flag_variant_dock_(false)
{        
}

void EstimateListener::changeVelocity(double value)
{
	Velocity vel(0, value);
	ChangeVelocity vel_msg(query_client_->getID(), vel);
	query_client_->setMsg(vel_msg.dump());
	query_client_->send();
}

void EstimateListener::abortCurrentSkill(double value)
{
	ApproachDist app_dist(100000);
	AbortCurrentSkill abort_msg(query_client_->getID(), app_dist);
	query_client_->setMsg(abort_msg.dump());
	query_client_->send();
}

void EstimateListener::abortAction(const std::string& variant)
{
    query_client_->setMsg("abort_current_action");
    query_client_->send();
    //std::cout << "Abortando mission.. 2s" << std::endl;
    //sleep(2);
    variant_client_->sendVariant(variant);
}


void EstimateListener::dataAvailable(const RoqmeDDSTopics::RoqmeEstimate &data, const dds::sub::SampleInfo &sampleInfo)
{

    double value = data.value();
	const std::string name = data.name();
 
#ifdef ADAPTATION_LOGIC
    if ( name == "safety" )
    {
        if(!flag_safety_ && value <= 0.50)
        {
            std::cout << "1.- Safety: " << value << ". Abortando mission" << std::endl;
            abortAction("deliver");
            flag_safety_ = true;
        }
        else if (flag_safety_ && value > 0.50) 
        {
            std::cout << "2.- Safety: " << value << ". Abortando mission" << std::endl;
            abortAction("pick");
            flag_safety_ = false;
        }
        
    } 
    else if ( name == "power_autonomy" )
	{
        if(!flag_variant_dock_ && value <= 0.4)
		{
            std::cout << "1.- Power autonomy: " << value << ". Abortando mission" << std::endl;
            abortAction("dock");
			flag_variant_dock_ = true;	 
		}
		else if(flag_power_autonomy_ == true && value > 0.4)
        {
            flag_variant_dock_ = false;
        }
    }

#endif    
}



bool EstimateListener::detectingObject(const std::string& currentAction, const std::string& prevAction)
{
    return ((currentAction == "objectRecognition") || (currentAction == "move" && prevAction == "objectRecognition"));
}



/*
void EstimateListener::dataAvailable(const RoqmeDDSTopics::RoqmeEstimate &data, const dds::sub::SampleInfo &sampleInfo)
{

	static std::string prevAction;    
    double value = data.value();
	const std::string name = data.name();
    std::string currentAction = currentAction_->get();
 
#ifdef ADAPTATION_LOGIC
    if ( name == "mission_completion" )
    {
         std::cout << "mission_completion " <<  value << " " << currentAction << " " << prevAction << std::endl;
		if(detectingObject(currentAction, prevAction))
        {
            if(!flag_mission_completion_ && value <= 0.60 && !flag_safety_)
            {
                std::cout << "1.- MissionCompletion: " << value << ". Abortando mission" << std::endl;
                abortAction("picktwo");
                flag_mission_completion_ = true;
            }
            else if ( ( flag_mission_completion_ && value <= 0.4 && !flag_variant_dock_)  ||
                (!flag_mission_completion_ && flag_safety_ && value < 0.75) ) //se ha ido al picking point 2 por el safety
             {
                std::cout << "2.- MissionCompletion: " << value << ". Abortando mission" << std::endl;
                abortAction("dock");
                flag_variant_dock_ = true;
                flag_mission_completion_ = true;
            }
        }
    }
    else if ( name == "safety" )
    {
        std::cout << "safety " <<  value << " " << currentAction << " " << prevAction << std::endl;
        if(/*currentAction == "objectRecognition" ||*/ /* currentAction == "approach" )
        {
           
            if(!flag_safety_ && value <= 0.75 && !flag_mission_completion_)
            {
                std::cout << "1.- Safety: " << value << ". Abortando mission" << std::endl;
                abortAction("picktwo");
                flag_safety_ = true;
            }
            else if ((flag_safety_ && value < 0.5 && !flag_variant_dock_) || 
                (!flag_safety_ && value < 0.8 && flag_mission_completion_ && !flag_variant_dock_))//TODO: comprobar estos valores en tiempo de ejecución
            {
                std::cout << "2.- Safety: " << value << ". Abortando mission" << std::endl;
                abortAction("dock");
                flag_variant_dock_ = true;
                flag_safety_ = true;
            }
        }
    } 
    else if ( name == "power_autonomy" )
	{
        if(flag_variant_dock_ == false && value <= 0.4)
		{
            std::cout << "2.- Power autonomy: " << value << ". Abortando mission" << std::endl;
            abortAction("dock");
			flag_variant_dock_ = true;	 
		}
		else if(flag_power_autonomy_ == true && value > 0.4)
        {
            flag_variant_dock_ = false;
           // flag_variant_pickup_ = false;
        }
    }
    
   
    if (prevAction.compare(currentAction))
    {
        prevAction = currentAction;
    }
    
#endif    
}*/

/**
 * DistanceGoal implementation
 */

DistanceToGoal::DistanceToGoal(): distance(-1)
{    
}
    
double DistanceToGoal::get()
{
    std::lock_guard<std::mutex> g_lock(lock);
    return distance;
}
  
void DistanceToGoal::set(const double& value)
{
    std::lock_guard<std::mutex> g_lock(lock);
    distance = value;
}


/**
 * CurrentAction implementation
 */

CurrentAction::CurrentAction()
{    
}
    
std::string CurrentAction::get()
{
    std::lock_guard<std::mutex> g_lock(lock);
    return action;
}
  
void CurrentAction::set(const std::string& value)
{
    std::lock_guard<std::mutex> g_lock(lock);
    action = value;
}


#ifdef SUBSCRIBE_TO_DOUBLE_CONTEXTS
DoubleContextListener::DoubleContextListener(std::shared_ptr<DistanceToGoal>& distance):distance_(distance)
{ 
}

void DoubleContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeDoubleContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	
    const std::string& name = data.name();
    if(!name.compare("DistanceToGoal")) {
        distance_->set(data.value().at(0));
    }
        
}
#endif



/* 
 * Roqme context listeners implementation
 */

#ifdef SUBSCRIBE_TO_INT_CONTEXTS
void IntContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeIntContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	/* cout << "INT sample available" << endl;
	cout << "\t name: " << data.name() << endl;
	for(auto elem:data.value())
	{
		cout << "\t " << elem << endl;
	}*/
}

#endif

#ifdef SUBSCRIBE_TO_UINT_CONTEXTS
void UIntContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeUIntContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	/*cout << "UINT sample available:" << endl;
	cout << "\t name: " << data.name() << endl;
	for(auto elem:data.value())
	{
		cout << "\t " << elem << endl;
	}*/
}

#endif

#ifdef SUBSCRIBE_TO_BOOL_CONTEXTS
void BoolContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeBoolContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	/*cout << "BOOL sample available:" << endl;
	cout << "\t name: " << data.name() << endl;
	for(auto elem:data.value())
	{
		cout << "\t " << elem << endl;
	}*/
}

#endif

#ifdef SUBSCRIBE_TO_ENUM_CONTEXTS

EnumContextListener::EnumContextListener(std::shared_ptr<CurrentAction>& currentAction):
    currentAction_(currentAction)
{
}

void EnumContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeEnumContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	//std::cout << "ENUM sample available:" << std::endl;
	if(!data.name().compare("Action")) 
    {
      //  std::cout << "\t name: " << data.name() << std::endl;
        for (auto elem : data.value())
        {
            //std::cout << "\t " << elem << std::endl;
            currentAction_->set(elem);
        }
    }
	
}

#endif

#ifdef SUBSCRIBE_TO_EVENT_CONTEXTS
void EventContextListener::dataAvailable(
		const RoqmeDDSTopics::RoqmeEventContext &data,
		const dds::sub::SampleInfo &sampleInfo)
{
	/*cout << "EVENT sample available:" << endl;
	cout << "\t name: " << data.name() << endl;
	for(auto elem:data.value())
	{
		cout << "\t " << elem << endl;
	}*/
}

#endif


