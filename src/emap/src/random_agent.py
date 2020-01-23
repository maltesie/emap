#! /usr/bin/env python3

import rospy
from mac_ros_bridge.msg import SimEnd, SimStart, RequestAction, Agent
from planner import Knowledge, TaskManager


class RandomAgent:    
    
    def __init__(self):

        self._initialized = False
        rospy.init_node('agent_node', anonymous=True)
        agent_name = rospy.get_param('~agent_name', 'UNKNOWN')
        nb_agents = int(rospy.get_param('~nb_agents', 6))
        
        rospy.loginfo("{}: begin init".format(agent_name))
        
        self.knowledge = Knowledge(agent_name, nb_agents)
        
        rospy.Subscriber('/bridge_node_' + agent_name + '/start', SimStart, self.knowledge._initialize)
        rospy.Subscriber('/bridge_node_' + agent_name + '/end', SimEnd, self.knowledge._end_simulation)
        
        while (not self.knowledge.simulation_running):
            rospy.sleep(0.05)
            
        for agent_number in range(1, nb_agents+1):
            rospy.Subscriber('/bridge_node_agent' + self.knowledge.team + str(agent_number) + '/agent', Agent, callback=self.knowledge._update_agents)
        rospy.Subscriber('/bridge_node_' + agent_name + '/request_action', RequestAction, self._callback_request_action)
        
        while not self.knowledge.received_first_percept:
            rospy.sleep(0.05)
        
        self.task_manager = TaskManager(self.knowledge, random_behaviour=True)
        
        self._initialized = True
        rospy.loginfo("{}: end init".format(agent_name))
        

    def _callback_request_action(self, msg):
        self.knowledge.update_world(msg)
        if self._initialized: self.task_manager.step()
        #rospy.logdebug("Agent::callback %s", str(msg))
    


if __name__ == '__main__':
    try:
        dummy = RandomAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
