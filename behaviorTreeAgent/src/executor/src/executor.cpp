#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include <list>

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "Intralogistic/args.hpp"
#include "Intralogistic/skill_action.hpp"
#include "Intralogistic/variant_action.hpp"
#include "Intralogistic/deliver_object.hpp"
#include "Intralogistic/pick_object.hpp"
#include "Intralogistic/detect_object.hpp"
#include "Intralogistic/get_position.hpp"
#include "Intralogistic/move_roboter_position.hpp"

using namespace BT;

struct Arguments{
    std::string skills_file;
    std::string tree_file;
    std::string IP;
};

Arguments ParseArguments(int argc, char** argv);

int main(int argc, char** argv)
{   
    Arguments arg = ParseArguments(argc, argv);

    std::cout << "\ttree file: "   << arg.tree_file << std::endl;
    std::cout << "\tIP: "          << arg.IP << std::endl;
    std::cout << "\tskills file: " << arg.skills_file << std::endl;

    //------------------------------------------------------
    // Populate the factory from Skills file and
    // create the tree from BehaviorTree file.

    zmq::context_t zmq_context(1);

    BehaviorTreeFactory factory;

    // Register moveRoboterPosition Action
    TreeNodeManifest manif;
    manif.type = NodeType::ACTION;
    manif.registration_ID = "moveRoboterPosition";
    manif.ports = BT::PortsList{
        BT::InputPort<std::string>("approachRadius"),
	BT::InputPort<std::string>("x"),
	BT::InputPort<std::string>("y")};
    auto move_roboter_position = [](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new MoveRoboterPosition(name, config));
    };
    factory.registerBuilder(manif, move_roboter_position);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "DeliverObject";
    manif.ports = BT::PortsList{
        BT::InputPort<std::string>("x"),
	BT::InputPort<std::string>("y"),
	BT::InputPort<std::string>("z")};
    auto deliver_object = [](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new DeliverObject(name, config));
    };
    factory.registerBuilder(manif, deliver_object);


    manif.type = NodeType::ACTION;
    manif.registration_ID = "DetectObject";
    manif.ports = BT::PortsList{
        BT::OutputPort<std::string>("x"),
	BT::OutputPort<std::string>("y"),
	BT::OutputPort<std::string>("z")};
    auto detect_oject = [](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new DetectObject(name, config));
    };
    factory.registerBuilder(manif, detect_oject);

   /* manif.type = NodeType::ACTION;
    manif.registration_ID = "GetPosition";
    manif.ports = BT::PortsList{
        BT::OutputPort<std::string>("x"),
	BT::OutputPort<std::string>("y")};
    auto get_position = [](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new GetPosition(name, config));
    };
    factory.registerBuilder(manif, get_position);
*/
    manif.type = NodeType::ACTION;
    manif.registration_ID = "PickObject";
    manif.ports = BT::PortsList{
        BT::InputPort<std::string>("x"),
	BT::InputPort<std::string>("y"),
	BT::InputPort<std::string>("z")};
    auto pick_object = [](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new PickObject(name, config));
    };
 factory.registerBuilder(manif, pick_object);

    manif.type = NodeType::ACTION;
    manif.registration_ID = "VariantAction";
    manif.ports = BT::PortsList{BT::OutputPort<std::string>("value")};
    auto creator = [&zmq_context, &arg](const std::string& name, const NodeConfiguration& config)
    {
        return std::unique_ptr<TreeNode>(new VariantAction(name, config, 
            arg.IP.c_str(), zmq_context));
    };
    factory.registerBuilder(manif, creator);

    for (const auto& model : factory.manifests())
    {
        std::cout << model.first << std::endl;
    }

    auto tree = factory.createTreeFromFile(arg.tree_file);

    // add loggers
    StdCoutLogger logger_cout(tree);
    FileLogger logger_file(tree, "ulm_trace.fbl");
    PublisherZMQ publisher_zmq(tree);

    //------------------------------------------------------
    // Execute the tree
    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while( status == NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}

///////////////////////////////////////////////////////////////

Arguments ParseArguments(int argc, char** argv)
{
    // Read more about args here: https://github.com/Taywee/args

    args::ArgumentParser parser("BehaviorTree.CPP Executor", "Load one or "
        "multiple plugins and the XML with the tree definition.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

    args::Group arguments(parser, "arguments", 
        args::Group::Validators::DontCare, args::Options::Global);

    args::ValueFlag<std::string> tree_path(arguments, "path",
        "The XML containing the BehaviorTree ", {'t', "tree"},
                                           args::Options::Required);

    args::ValueFlag<std::string> skills_path( arguments, "path",
        "JSON file containing the list of SmartSoft skills", {'s', "skills"},
                                              args::Options::Required);

    args::ValueFlag<std::string> server_ip(arguments, "ip",
        "IP of the server", {"ip"}, "localhost" );
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (const args::Completion& e)
    {
        std::cout << e.what();
        exit(0);
    }
    catch (const args::Help&)
    {
        std::cout << parser;
        exit(0);
    }
    catch (const args::ParseError& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        exit(0);
    }
    catch (const args::RequiredError& e)
    {
        std::cerr << "One of the mandatory arguments is missing:" << std::endl;
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        exit(0);
    }

    Arguments output;

    output.skills_file =  args::get(skills_path);
    output.tree_file   =  args::get(tree_path);
    output.IP =  args::get(server_ip);

    return output;
}
