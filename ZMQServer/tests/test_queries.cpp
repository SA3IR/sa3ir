#include <gtest/gtest.h> 
#include <change_velocity.hpp>
#include <positions.hpp>

using namespace zmqserver;

TEST(QueryTest, ChangeVelocity) 
{
    std::string request_1 = "{\n "
        "\"id\": 1,\n "
        "\"msg-type\": \"query\",\n "
        "\"query\": {\n  "
            "\"Component\": \"SmartCdlServer\",\n  "
            "\"Parameter\": \"transvel\",\n  "
            "\"ParameterSet\": \"CdlParameter\",\n  "
            "\"ParameterSetRepository\": \"CommNavigationObjects\",\n  "
            "\"type\": \"change-parameter\",\n  "
            "\"values\": {\n   "
                "\"1\": 1.0,\n   "
                "\"2\": 5.5\n  "
            "}\n "
        "}\n"
    "}";

    std::string request_2 = "{\n "
        "\"id\": 1,\n "
        "\"msg-type\": \"query\",\n "
        "\"query\": {\n  "
            "\"Component\": \"SmartCdlServer\",\n  "
            "\"Parameter\": \"transvel\",\n  "
            "\"ParameterSet\": \"CdlParameter\",\n  "
            "\"ParameterSetRepository\": \"CommNavigationObjects\",\n  "
            "\"type\": \"change-parameter\",\n  "
            "\"values\": {\n   "
                "\"1\": 4.3,\n   "
                "\"2\": 5.5\n  "
            "}\n "
        "}\n"
    "}";
    
    Velocity vel(1.0, 5.5);
    ChangeVelocity query(1, vel);
    ASSERT_EQ(request_1, query.dump());

    vel.values["1"] = 4.3;
    query.setParameter(vel);
    ASSERT_EQ(request_2, query.dump());
}

TEST(QueryTest, Positions)
{
    std::string request = "{\n "
        "\"id\": 2,\n "
        "\"msg-type\": \"query\",\n "
        "\"query\": {\n  "
            "\"type\": \"get-all-positions\"\n "
        "}\n"
    "}";
    Positions query(2);
    ASSERT_EQ(request, query.dump());
} 