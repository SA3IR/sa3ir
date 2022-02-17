#
# Mira
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=labMalaga.mcf -p 1234'

#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=labPhaseTwo.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=santantoniabatbiblioteca.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=prueba.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=automatica.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=salaReunionesTE.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=santantoniabatplantabaja.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd ~ && source .bashrc && mira MiraNavigation:etc/SCITOS-application.xml MiraNavigation:etc/MiraNavigation.xml -v MCFFile=santantoniabatplantauno.mcf -p 1234'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'mira'

#sleep 3

# rcremoteserver
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcremoteserver'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcremoteserver'

sleep 2

rcremote localhost IS /home/robocomp/robocomp/components/sa3ir/etc/ rcnode

sleep 2

# AGMServer
#rcremote localhost AGMExecutive /home/robocomp/AGM/tools/AGMExecutive_robocomp python AGMExecutive_robocomp.py --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/agmexecutive.conf
rcremote localhost AGMExecutive /home/robocomp/robocomp/tools/AGM/tools/AGMExecutive_robocomp python AGMExecutive_robocomp.py --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/agmexecutive.conf

sleep 2

# ZQMServer
#rcremote localhost ZMQServer /home/robocomp/robocomp/components/sa3ir/zmqserver ./bin/zmq_dummy_server
rcremote localhost ZMQServer /home/robocomp/robocomp/components/sa3ir/build/ZMQServer ./ZMQServer_client

sleep 2

# BehaviorTree
rcremote localhost BehaviorTree /home/robocomp/robocomp/components/sa3ir/behaviorTreeAgent ./bin/behaviorTree /home/robocomp/robocomp/components/sa3ir/etc/config/behaviorTree.conf

sleep 1 

#
# MironAgent
rcremote localhost mironagent /home/robocomp/robocomp/components/sa3ir/mironAgent ./bin/miron /home/robocomp/robocomp/components/sa3ir/etc/config/mironAgent.conf

sleep 1;

#
# FuzzyReasoner
#rcremote localhost fuzzyReasoner /home/robocomp/robocomp/components/sa3ir/fuzzyReasoner ./bin/fuzzyReasoner

sleep 1

#
# Mission
rcremote localhost mission /home/robocomp/AGM/tools/agmmission agmmission --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/mission.conf

sleep 1

#
# deliverAgent
rcremote localhost deliveragent /home/robocomp/robocomp/components/sa3ir/deliverAgent/bin ./deliver --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/deliverAgent.conf

sleep 1

#
# webInterfaceAgent
rcremote localhost webinterface /home/robocomp/robocomp/components/sa3ir/webInterfaceAgent/bin ./webInterfaceAgent --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/webInterfaceAgent.conf

#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/sa3ir/webInterfaceAgent/bin && ./#webInterfaceAgent --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/webInterfaceAgent.conf'

#sleep 1

#
# objectRecognitionAgent
rcremote localhost objectrecognitionagent /home/robocomp/robocomp/components/sa3ir/objectRecognitionAgent/bin ./objectRecognition --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/objectRecognitionAgent.conf

sleep 1

#
# pickUpAgent
rcremote localhost pickupagent /home/robocomp/robocomp/components/sa3ir/pickUpAgent/bin ./pickUp --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/pickUpAgent.conf

sleep 1

#
# LocalNavigation
rcremote localhost localnavigation /home/robocomp/robocomp/components/sa3ir/mira/localNavigationComp/bin ./localnavigationComp --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/localNavigation.conf

#
# NavigationAgent
rcremote localhost navigationagent /home/robocomp/robocomp/components/sa3ir/navigationAgent/bin ./navigationagent --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/navigationAgent.conf

sleep 1


#
# PersonAgent
rcremote localhost personAgent /home/robocomp/robocomp/components/sa3ir/personAgent ./bin/personAgent /home/robocomp/robocomp/components/sa3ir/etc/config/agentPerson.conf

sleep 1;


#
# RealsenseAgent
#rcremote localhost realsense /home/robocomp/robocomp/components/sa3ir/realsenseComp ./bin/realsense /home/robocomp/robocomp/components/sa3ir/etc/config/realsenseAgent.conf

#sleep 1;

#
# TrolleyDetector
rcremote localhost trolley /home/robocomp/robocomp/components/sa3ir/trolleyDetectorComp ./bin/trolleyDetector /home/robocomp/robocomp/components/sa3ir/etc/config/trolleyDetectorComp.conf

sleep 1;

#
# ContextProvider
rcremote localhost contextProvider /home/robocomp/robocomp/components/sa3ir/contextProviderComp/bin ./contextProvider --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/contextProvider.conf

sleep 1;

#
# ChatbotAgent
rcremote localhost chatbot /home/robocomp/robocomp/components/sa3ir/chatbotAgent/bin ./chatbot --Ice.Config=/home/robocomp/robocomp/components/sa3ir/etc/config/chatbot.conf

