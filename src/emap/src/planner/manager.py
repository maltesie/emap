from .task import TaskProvider
from .distance import Distance
import rospy
import numpy as np
from mac_ros_bridge.msg import GenericAction
from emap.msg import SOHeartBeat, SOMeetingRequest, SOMeetingReply, SOMeetingRemove, SOResourceMessage, SOJobMessage, SOJobRemove
from .conditions import ConditionProvider
from .behaviour import GatherUpgradeStore, ExploreBuildDismantle, AssembleDeliver
from .self_organisation import SOJob, SOMeeting, SOResource
from .optimizer import OptimizationProvider


class LogManager(object):
    
    def __init__(self, knowledge):
        self._log_file_name = './one_giant.log'
        self._knowledge = knowledge
        self._failed_jobs = []
        self._successful_jobs = []
        self._resources_gathered = {res.name:0 for res in knowledge.get_all_resources()}
        self._auctions_bid_on = []
        self._failed_missions = []
        a = ''
        for res in self._resources_gathered: a += ' ' + res
        with open(self._log_file_name,'a') as f: f.write(self._knowledge.my_name+' begins new simulation\n')
        
    def write_step_to_file(self):
        my_step = '{} {} {} {} {} {} {} {} {} {} {} {}'.format(self._knowledge.my_name, self._knowledge.step, len(self._failed_jobs), \
                   len(self._successful_jobs), self._knowledge.massium, self._knowledge.score, len(self._knowledge.own_wells),\
                   len([job for job in self._failed_jobs if job[1] == 'auction']), len([job for job in self._successful_jobs if job[1] == 'auction']),\
                   len(self._auctions_bid_on), len([job for job in self._failed_jobs if job[1] == 'mission']), len([job for job in self._successful_jobs if job[1] == 'mission']))
        for res in self._resources_gathered: my_step += ' ' + str(self._resources_gathered[res])
        my_step += '\n'
        with open(self._log_file_name, 'a') as f:
            f.write(my_step)


class ActionManager(object):
    
    def __init__(self, agent_name):
        
        self._pub_generic_action = rospy.Publisher('bridge_node_' + agent_name + '/generic_action', GenericAction, queue_size=1)
        self._agent_name = agent_name
    
    def send_action(self, action):
        ##print(self._agent_name, 'send action', action.name)
        msg = GenericAction()
        if action._needs_evaluation:
            msg.params = action.params.evaluate()
        else:
            msg.params = action.params
        msg.action_type = action.action_type
        msg.id = ''
        msg.target_deadline = 0
        self._pub_generic_action.publish(msg)
    

class ChargeManager(object):
    
    def __init__(self, knowledge, condition_provider, distance_calc):
        self._condition_provider = condition_provider
        self._knowledge = knowledge
        self._distance_calc = distance_calc
        self._can_reach_nearest_charging_station_condition = self._condition_provider.can_reach_nearest_charging_station_condition()
        self._charging_positions = np.array([self._knowledge.charging_station_positions[name] for name in self._knowledge.charging_station_positions])
        self._charging_names = np.array([name for name in self._knowledge.charging_station_positions])
        self._recharge_now_condition = self._condition_provider.recharge_now_condition()
        
    def get_charge(self):
        return self._knowledge.my_info.charge
    
    def get_max_charge(self):
        return self._knowledge.my_info.charge_max
        
    def can_reach_nearest_charging_station(self):
        return self._can_reach_nearest_charging_station_condition.check()
        
    def can_reach_current_goal_position(self, current_task):
        own_position = self._knowledge.my_position
        max_steps = self.get_charge()-1
        ##print('steps to goal', self._distance_calc.steps_between(own_position, current_task.goal_position), 'max steps', max_steps)
        if current_task._has_position: return (self._distance_calc.steps_between(own_position, current_task.goal_position)<=max_steps)
        else: return True
    
    def charge_at_nearest_station_task(self):
        own_position = self._knowledge.my_position
        steps_needed = []
        temp_positions = []
        for charging_position in self._charging_positions:
            temp_positions.append(charging_position)
            steps_needed.append(self._distance_calc.steps_between(own_position, charging_position))
        goal_position = temp_positions[np.argmin(steps_needed)]
        position_watcher = self._condition_provider.position_watcher(goal_position)
        charge_at_least_x_condition = self._condition_provider.charge_at_least_x_condition(self.get_max_charge())
        return TaskProvider.charge_task(position_watcher, charge_at_least_x_condition)
        
    def shortest_detour_charging_task(self, next_task):
        own_position = self._knowledge.my_position
        max_steps = self._knowledge.my_info.charge
        best_pos = self._charging_station_position_shortest_detour_between(own_position, next_task.goal_position, max_steps)
        if best_pos is not None: 
            charge_condition = self._condition_provider.charge_at_least_x_condition(self.get_max_charge())
            return TaskProvider.charge_task(self._condition_provider.position_watcher(best_pos), charge_condition)
        else: return None
    
    def _charging_station_position_shortest_detour_between(self, a, b, max_steps):
        bestpos, beststeps = None, np.inf
        for pos in self._charging_positions:
            steps_to_station, steps_from_station = self._distance_calc.steps_between(a, pos), self._distance_calc.steps_between(pos, b)
            steps_without = self._distance_calc.steps_between(a, b)
            steps = (steps_to_station + steps_from_station) - steps_without
            if (steps < beststeps) and (steps_to_station < max_steps): bestpos, beststeps = pos, steps
        return bestpos
    
    
class BehaviourManager(object):
    
    def __init__(self, knowledge, distance_calc, so_manager, job_manager, condition_provider, optimization_provider):
        
        self._knowledge = knowledge
        self._so_manager = so_manager
        self._gather_upgrade_store_behaviour = GatherUpgradeStore(knowledge, condition_provider, optimization_provider)
        self._explore_build_dismantle_behaviour = ExploreBuildDismantle(knowledge, condition_provider, optimization_provider, distance_calc)
        self._assemble_deliver_behaviour = AssembleDeliver(job_manager)
        self._distance = distance_calc
        self._behaviour_type = None
        self._waiting_assemble_tasks = None
        self._waiting_job = None
        self._my_requested_meetings = []
        self._my_auction_tasks = None
        
    def _kill_waiting_job(self):
        self._waiting_assemble_tasks = None
        self._waiting_job = None
        
    def check_behaviour(self, behaviour_type):
        if self._knowledge.role.name != 'drone':
            if behaviour_type == 'explore': return False
            else: return True
        else:
            if behaviour_type == 'explore': return True
            else: return True
            
    def set_behaviour(self, behaviour_type):
        self._behaviour_type = behaviour_type
        if self._behaviour_type == 'gather': self._behaviour = self._gather_upgrade_store_behaviour
        elif self._behaviour_type == 'explore': self._behaviour = self._explore_build_dismantle_behaviour
        elif self._behaviour_type == 'assemble': self._behaviour = self._assemble_deliver_behaviour
    
    def get_tasks(self):
        if self._waiting_assemble_tasks is not None:
            if self._so_manager.is_my_job(self._waiting_job.id):
                for task in self._waiting_assemble_tasks:
                    if task.name.startswith('retrieve'):
                        _, resource, amount, storage = task.name.split('_')
                        if not self._so_manager.is_my_resource(storage, resource, int(amount)): 
                            return [TaskProvider.skip_task()]
                for i, task in enumerate(self._waiting_assemble_tasks):
                    if task.name.startswith('assemble'): break
                meeting_step = int(sum(self._distance.steps_per_task(self._waiting_assemble_tasks[:i]))+self._knowledge.step) + 6
                #print(self._knowledge.step, self._knowledge.my_name + ' needs assistance at step ' + str(meeting_step) + ' by '+ str(self._waiting_job.required_roles) + ' for job '+str(self._waiting_job.id))
                #print(self._knowledge.step, self._knowledge.my_name + 'has to build ' + str([(item, self._waiting_job.items[item], self._waiting_job._needed_items[item]) for item in self._waiting_job.items]))
                meeting_position = task.goal_position
                for required_role in self._waiting_job.required_roles:
                    if (required_role != self._knowledge.role.name): 
                        #print(self._knowledge.my_name + ' is requesting '+required_role+' for '+self._waiting_job.id)
                        self._my_requested_meetings.append(self._so_manager.request_meeting(required_role, meeting_position, meeting_step))
                tasks = self._waiting_assemble_tasks
                self._kill_waiting_job()
                return tasks
            else:
                for task in self._waiting_assemble_tasks:
                    if task.name.startswith('retrieve'):
                        _, resource, amount, storage = task.name.split('_')
                        self._so_manager.release_resource_at_storage(storage, resource, int(amount))
                self._kill_waiting_job()
        if self._knowledge.role.name in ['truck']:
            self.set_behaviour(behaviour_type='assemble')
            job, tasks = self._behaviour.get_tasks()
            #print('time spent in assemble behaviour:', time.time()- t0)
        else: job, tasks = None, []
        if job is None: 
            #tasks = [TaskProvider.skip_task()]
            for behaviour_type in ('explore', 'gather'):
                if self.check_behaviour(behaviour_type):
                    self.set_behaviour(behaviour_type)
                    job, tasks = self._behaviour.get_tasks()
                    #print(self._knowledge.my_name, [task.name if task is not None else 'NONE' for task in tasks])
                    break
        else:
            ##print('can work on a job')
            if tasks[0].name.startswith('best_bid'):
                #print('Im bidding')
                start_in_steps = int(tasks[0].name.split('_')[-1]) - self._knowledge.step
                for i in range(start_in_steps-1):
                    tasks = [TaskProvider.skip_task()] + tasks
                return tasks
            self._so_manager.apply_for_job(job.id)
            self._waiting_assemble_tasks = tasks
            self._waiting_job = job
            for task in self._waiting_assemble_tasks:
                    if task.name.startswith('retrieve'):
                        _, resource, amount, storage = task.name.split('_')
                        self._so_manager.block_resource_at_storage(storage, resource, int(amount))
            tasks = [TaskProvider.skip_task()]
            #print('waiting for resources to be blocked for me')
        return tasks
    
    
class TaskManager(object):
    
    def __init__(self, knowledge, random_behaviour=False):
        
        self._knowledge = knowledge
        self._log_manager = LogManager(knowledge)
        self._distance_calc = Distance(knowledge)
        self._so_manager = SOManager(knowledge)
        self._condition_provider = ConditionProvider(knowledge, self._so_manager, self._distance_calc)
        self._optimization_provider = OptimizationProvider(random=random_behaviour)
        self._charge_manager = ChargeManager(knowledge, self._condition_provider, self._distance_calc)
        self._action_manager = ActionManager(knowledge.my_name)
        self._random = random_behaviour
        self._job_manager = JobManager(self._so_manager, self._optimization_provider,  self._condition_provider, self._distance_calc, knowledge)
        self._behaviour_manager = BehaviourManager(knowledge, self._distance_calc, self._so_manager, self._job_manager, self._condition_provider, self._optimization_provider)
        self.tasks = []
        self.set_current_task()
        self._current_meeting = SOMeetingRequest()
        self._current_resource_for_gathering = None
    
    
    def _set_current_task(self):
        try: self.current_task = self.tasks[0]
        except IndexError: 
            self.tasks = self._behaviour_manager.get_tasks()
            self.current_task = self.tasks[0] 
        while (self.current_task.current_action() is None):
            try: 
                del self.tasks[0]
                self.current_task = self.tasks[0]
            except IndexError:  
                self.tasks = self._behaviour_manager.get_tasks()
                self.current_task = self.tasks[0]
        #print('time in _set_current_task total:', time.time()-t0)self._so_manager.release_resource_at_storage(storage, resource, int(amount))
    
    def _process_charge_manager(self):
        if (self._charge_manager.get_charge() == 0) and (self.current_task.name!='recharge_now') and (self._knowledge.my_position != self._charge_manager.charge_at_nearest_station_task().goal_position).all():
            self.tasks = [TaskProvider.recharge_now_task(self._charge_manager._recharge_now_condition)] + self.tasks
            self.current_task = self.tasks[0]
        elif (not self._charge_manager.can_reach_nearest_charging_station()) and (self.current_task.name not in ['charge', 'recharge_now']): 
            self.tasks = [self._charge_manager.charge_at_nearest_station_task()] + self.tasks
            self.current_task = self.tasks[0]
        elif (not self._charge_manager.can_reach_current_goal_position(self.current_task)) and (self.current_task.name not in ['charge', 'recharge_now']):
            detour_charging_task = self._charge_manager.shortest_detour_charging_task(self.current_task)
            if detour_charging_task is not None:
                self.tasks = [detour_charging_task] + self.tasks
                self.current_task = self.tasks[0]
    
    def _answer_meeting_requests(self):   
        open_requests = self._so_manager._meeting.get_open_meeting_requests()
        for request in open_requests:
            if (request.step - self._knowledge.step) >= 5: continue
            #print(self._knowledge.my_name, 'answering request', request.id)
            arrival_step = self._distance_calc.steps_between(self._knowledge.my_position, request.position) + self._knowledge.step
            self._so_manager.reply_meeting(int(arrival_step), request.id)
    
    def _process_meetings(self):
        my_meeting = self._so_manager.get_my_next_meeting()
        if (my_meeting is not None) and (my_meeting.id != self._current_meeting.id) and not self.current_task.name.startswith('assist'):
            #print(self._knowledge.step, self._knowledge.my_name, self._knowledge.role.name, 'meeting', my_meeting.agent_name)
            self._current_meeting = my_meeting
            position_watcher = self._condition_provider.position_watcher(my_meeting.position)
            meeting_removed_condition = self._condition_provider.meeting_removed_condition(my_meeting.id)
            task = TaskProvider.assist_assemble_task(position_watcher, meeting_removed_condition, my_meeting.agent_name, name='assist_'+str(my_meeting.id))
            self.add_task_at(task, 0)
    
    def _process_action_results(self):
        
        if self._knowledge.last_action == 'build':
            well_name = self.current_task.name.split('_')[-1]
            if well_name not in self._knowledge.wells_in_vision:
                if well_name in self._knowledge.own_wells: del self._knowledge.own_wells[well_name]
            for well in self._knowledge.wells_in_vision:
                w_info = self._knowledge.wells_in_vision[well]
                if (w_info.team == self._knowledge.team) and (w_info.efficiency==0):
                    position_watcher = self._condition_provider.position_watcher(self._knowledge.facility_positions[well])
                    well_does_not_improve_condition = self._condition_provider.well_does_not_improve_condition(well)
                    task = TaskProvider.build_up_well_task(position_watcher=position_watcher, well_does_not_imporve_condition=well_does_not_improve_condition)
                    if (task.name != self.current_task.name): self.tasks = [task] + self.tasks
                    break
                
        if self._knowledge.last_action == 'dismantle':
            well_name = self.current_task.name.split('_')[-1]
            if well_name not in self._knowledge.wells_in_vision:
                if well_name in self._knowledge.opponent_wells: del self._knowledge.opponent_wells[well_name]
        
        if self.current_task.name.startswith('gather'):
            self._current_resource_for_gathering = self.current_task.name.split('_')[1]
            
        if self._knowledge.last_action == 'gather' and self._knowledge.last_action_result == 'successful':
            self._log_manager._resources_gathered[self._current_resource_for_gathering] += 1
        
        if self.current_task.name.startswith('best_bid_for'):
            job_name = self.current_task.name.split('_')[3]
            if job_name not in self._log_manager._auctions_bid_on: 
                self._log_manager._auctions_bid_on.append(job_name)
        
        if self._knowledge.last_action == 'randomFail':
            self.current_task.reset_last_action()
        
        if self._knowledge.last_action == 'build' and self._knowledge.last_action_result!='successful':
            remove = False
            for well in self._knowledge.own_wells:
                if self._distance_calc.are_same(self._knowledge.facility_positions[well], self._knowledge.my_position): 
                    remove=True
                    break
            if False and remove: 
                del self._knowledge.own_wells[well]
                del self._knowledge.facility_positions[well]
                
        if self.current_task.name.startswith('deliver_job'):
            for meeting_id in self._behaviour_manager._my_requested_meetings:
                self._so_manager.remove_meeting(meeting_id)
            self._behaviour_manager._my_requested_meetings = []
        
        if self._knowledge.last_action == 'deliver_job':
            self._so_manager.remove_job(self.current_task.name.split('_')[2])
        
        if self._knowledge.last_action == 'retrieve':
            _, resource, amount, storage = self.current_task.name.split('_')
            self._so_manager.release_resource_at_storage(storage, resource, int(amount))
       
        if self._knowledge.last_action == 'deliver_job' and self._knowledge.last_action_result=='successful':
            job_id = self.current_task.name.split('_')[2]
            print(self._knowledge.step, self._knowledge.my_name, 'FINISHED JOB SUCCESFULLY', job_id, self._knowledge._all_job_types[job_id])
            self._log_manager._successful_jobs.append((job_id, self._knowledge._all_job_types[job_id]))
        elif self._knowledge.last_action == 'deliver_job' and self._knowledge.last_action_result!='successful':
            job_id = self.current_task.name.split('_')[2]
            self._log_manager._failed_jobs.append((job_id, self._knowledge._all_job_types[job_id]))
            print(self._knowledge.step, self._knowledge.my_name, 'JOB DELIVERY FAILED', job_id, self._knowledge._all_job_types[job_id], self._knowledge.last_action_result)
            if job_id in self._knowledge.auction_jobs: print('Job failed not because of time:', self._knowledge.auction_jobs[job_id].job_type, self._knowledge.auction_jobs[job_id].end)
            elif job_id in self._knowledge.mission_jobs: print('Job failed not because of time:', self._knowledge.mission_jobs[job_id].job_type, self._knowledge.mission_jobs[job_id].end)
            elif job_id in self._knowledge.priced_jobs: print('Job failed not because of time:', self._knowledge.priced_jobs[job_id].job_type, self._knowledge.priced_jobs[job_id].end)
        
    def _random_gather_cancel(self):
        if len(self._knowledge.get_all_resources()) != len(self._knowledge.get_known_resources()): return
        if (self._knowledge.role.name == 'truck') and (self._knowledge.get_my_resources_count() == 0) and (self._behaviour_manager._behaviour_type == 'gather'):
            if np.random.rand() < 0.05: 
                self.tasks = []
        
    def set_current_task(self):
        self._random_gather_cancel()
        self._set_current_task()
        self._process_meetings()
        self._process_charge_manager()
        self._set_current_task()

    def add_tasks(self, tasks):
        self.tasks += tasks
        
    def add_task_at(self, task, at_index):
        self.tasks.insert(at_index, task)
        
    def send_current_action(self):
        self._action_manager.send_action(self.current_task.current_action())
    
    def step(self):
        self._answer_meeting_requests()
        self._so_manager.send_heartbeat()
        self._so_manager.sync()
        self._process_action_results()
        self.set_current_task()
        self.send_current_action()
        self.current_task.step()
        self._log_manager.write_step_to_file()
        
                 

class JobManager(object):
    
    def __init__(self, so_manager, optimization_provider, condition_provider, distance, knowledge):
        self._agent_name = knowledge.my_name
        self._condition_provider = condition_provider
        self._knowledge = knowledge
        self._distance = distance
        self._so_manager = so_manager
        self._optimization_provider = optimization_provider
        self._needed_items = {}
        self._needed_roles = {}
        for item in knowledge.product_types:
            self._needed_items[item] = []
            for consumed in knowledge.product_types[item].consumed_items:
                self._needed_items[item] += [consumed]
            self._needed_roles[item] = []
            for role in knowledge.product_types[item].required_roles:
                self._needed_roles[item] += [role]
    
    def get_priced_tasks(self):
        priced_tasks = None
        best_job = None
        best_reward = 0
        jobs = self._knowledge.priced_jobs
        for jobid in jobs:
            if jobid in self._so_manager.get_jobs_already_taken(): continue
            job = self._knowledge.priced_jobs[jobid]
            tasks = self._get_tasks_if_job_can_be_delivered(job)
            if tasks is not None and job.reward>best_reward: 
                best_reward = job.reward
                best_job = job
                priced_tasks=tasks
        return best_job, priced_tasks
            
    def get_auction_tasks(self):
        auction_tasks = None
        best_job = None
        best_reward = 0
        jobs = self._knowledge.auction_jobs
        for jobid in jobs:
            #print(self._knowledge.step, jobs[jobid].id, jobs[jobid].start, jobs[jobid].auction_time, jobs[jobid].end)
            if jobid in self._so_manager.get_jobs_already_taken(): continue
            job = self._knowledge.auction_jobs[jobid]
            tasks = self._get_tasks_if_job_can_be_delivered(job)
            if tasks is not None:
                if ((job.start + job.auction_time) >= self._knowledge.step):
                    stop_bidding_condition = self._condition_provider.stop_bidding_condition(jobid)
                    bidding_task = TaskProvider.best_bid_task(jobid, self._knowledge, stop_bidding_condition=stop_bidding_condition)
                    return job, [bidding_task]
                elif (jobid in self._knowledge._my_auctions) and (job.reward>best_reward): 
                    best_reward = job.reward
                    best_job = job
                    auction_tasks = tasks
        return best_job, auction_tasks
    
    def get_mission_tasks(self):
        mission_tasks = None
        best_job = None
        best_reward = 0
        jobs = self._knowledge.mission_jobs
        for jobid in jobs:
            if jobid in self._so_manager.get_jobs_already_taken(): continue
            job = self._knowledge.mission_jobs[jobid]
            tasks = self._get_tasks_if_job_can_be_delivered(job)
            if tasks is not None and job.reward>best_reward: 
                best_reward = job.reward
                best_job = job
                mission_tasks=tasks
        return best_job, mission_tasks
    
    def get_nb_active_missions(self):
        nb_missions = 0
        for jobid in self._knowledge.mission_jobs:
            job = self._knowledge.mission_jobs[jobid]
            if job.end <= self._knowledge.step: nb_missions += 1
        return nb_missions
    
    def _get_tasks_if_job_can_be_delivered(self, job):
        if job.end <= self._knowledge.step: return None
        if not self._enough_load(job): return None
        storages_by_resource = self._get_storages_for_items(job)
        if [] in storages_by_resource.values(): return None
        tasks = self._get_tasks(storages_by_resource, job)
        dist = sum(self._distance.steps_per_task(tasks))
        #print(self._knowledge.my_name, job.id, dist + self._knowledge.step, job.end)
        if (dist + self._knowledge.step + 10 * (np.ceil(dist/50))) > job.end: return None
        return tasks
        
    def _get_storages_for_items(self, job):
        needed_resources = job.resources
        available_resources_per_storage = self.get_available_items_per_storage()
        storages_by_resource = {res:[] for res in needed_resources}
        for res in storages_by_resource:
            for storage in available_resources_per_storage:
                if available_resources_per_storage[storage][res] >= needed_resources[res]:
                    ##print(res, needed_resources[res], available_resources_per_storage[storage][res], storage)
                    storages_by_resource[res].append(storage)
        return storages_by_resource

    def _emergency_retrieve_task(self, resource, amount):
        available_resources_per_storage = self.get_available_items_per_storage()
        for storage in available_resources_per_storage:
            if available_resources_per_storage[storage][resource] >= amount: break
        position_watcher = self._condition_provider.position_watcher(self._knowledge.facility_positions[storage])
        return TaskProvider.retrieve_task(position_watcher, [resource], [amount], name='retrieve_'+resource+'_'+str(amount)+'_'+storage)

    def _enough_load(self, job):
        max_volume = self._knowledge.my_info.load_max - self._knowledge.my_info.load
        needed_resources = job.resources
        resource_volume_for_items = sum([self._knowledge.product_types[nr].volume * needed_resources[nr] for nr in needed_resources])
        #print('max volume, load:',self._knowledge.my_info.load_max, self._knowledge.my_info.load, [self._knowledge.product_types[nr].volume * needed_resources[nr] for nr in needed_resources])
        return max_volume > resource_volume_for_items
        
    def _get_tasks(self, storages_by_resource, job):
        for item in storages_by_resource:
            for i,storage in enumerate(storages_by_resource[item]):
                storages_by_resource[item][i] = self._knowledge.facility_positions[storage]
        optimizer = self._optimization_provider.retrieve_items_optimizer(self._knowledge.my_position, storages_by_resource)
        optimizer.run(30)
        best_plan = optimizer.get_best_storage_indices()
        best_positions = optimizer.get_best_positions()
        tasks = []
        for i, (resource, index) in enumerate(best_plan):
            need_to_get_amount =  job.resources[resource]
            if need_to_get_amount <= 0: continue
            position_watcher = self._condition_provider.position_watcher(best_positions[i])
            storage_name = [facility for facility in self._knowledge.facility_positions if (self._knowledge.facility_positions[facility] == best_positions[i]).all()][0]
            tasks.append(TaskProvider.retrieve_task(position_watcher, [resource], [need_to_get_amount], name='retrieve_'+resource+'_'+str(need_to_get_amount)+'_'+storage_name))
        closest_distance = np.inf
        last_pos = job.goal_position
        for workshop in self._knowledge.workshops:
            dist = self._distance.steps_between(last_pos, self._knowledge.facility_positions[workshop])
            if dist< closest_distance:
                closest_distance = dist
                assemble_pos = self._knowledge.facility_positions[workshop]
        position_watcher = self._condition_provider.position_watcher(assemble_pos)
        for item in job.items:
            if job.items[item] < 1: continue
            roles_present_condition = self._condition_provider.roles_present_condition(item)
            tasks.append(TaskProvider.assemble_task(position_watcher, item, job.items[item], roles_present_condition))
        position_watcher = self._condition_provider.position_watcher(job.goal_position)
        tasks.append(TaskProvider.deliver_job_task(position_watcher, job.id))
        return tasks
    
    def get_available_items(self):
        all_items = {item:0 for item in self._knowledge.product_types}
        all_items_per_storage = self.get_avaliable_items_per_storage()
        for storage in all_items_per_storage:
            for item in all_items_per_storage[storage]:
                all_items[item] += all_items_per_storage[storage][item]
        return all_items
    
    def get_available_items_per_storage(self):
        all_items_per_storage = {storage:{item:0 for item in self._knowledge.product_types} for storage in self._knowledge.storages}
        for storage in all_items_per_storage:
            for item in self._knowledge.storages[storage].items:
                all_items_per_storage[storage][item.name] = item.stored
        for block in self._so_manager._resources._resources._buffer:
            if (all_items_per_storage[block.storage_name][block.item_name] - block.item_amount) >= 0:
                all_items_per_storage[block.storage_name][block.item_name] -= block.item_amount
        return all_items_per_storage
    
    
class SOManager:
    
    def __init__(self, knowledge):
        self._knowledge = knowledge
        my_name = knowledge.my_name
        my_role = knowledge.role.name
        while (len([name for name in knowledge.info_by_name]) != (knowledge._nb_agents - 1)):
            rospy.sleep(0.05)
        other_agent_names = [name for name in knowledge.info_by_name]
        self._my_name = my_name
        self._my_role = my_role
        self._job = SOJob(my_name)
        self._resources = SOResource(knowledge)
        self._meeting = SOMeeting(my_name, my_role)
        self._publisher = rospy.Publisher('~so_heartbeat', SOHeartBeat, queue_size=1, latch=False)
        for agent in [my_name] + other_agent_names:
            rospy.Subscriber('agent_node_'+agent+'/so_heartbeat', SOHeartBeat, self._update)
        
        self._so_heatbeat_steps = np.zeros(len(other_agent_names) + 1) + knowledge.step
        self._current_so_heartbeat = SOHeartBeat()
        self._current_so_heartbeat.agent_name = self._my_name
        self._current_so_heartbeat.simulation_step = self._knowledge.step

        
    def _update(self, msg):
        #if self._my_name == 'agentA1': print(msg)
        if msg.update_job:
            for job_application in msg.job:
                self._job._update_buffer(job_application)
        if msg.update_remove_job:
            for job_removal in msg.remove_job:
                self._job.remove_job(job_removal.job_id)
        if msg.update_request:
            for request in msg.requests:
                self._meeting._update_buffer_request(request)
        if msg.update_reply:
            for reply in msg.replies:
                self._meeting._update_buffer_reply(reply)
        if msg.update_remove:
            for meeting_removal in msg.remove:
                self._meeting._update_buffer_remove(meeting_removal)
        if msg.update_resources:
            for resource in msg.resources:
                self._resources._update_buffer(resource)
        agent_number = int(msg.agent_name[6:])
        if self._so_heatbeat_steps[agent_number-1] < msg.simulation_step:
            self._so_heatbeat_steps[agent_number-1] = msg.simulation_step
        #print(self._knowledge.my_name, agent_number, 'updated a heartbeat')
    
    def print_so_summary(self):
        print('\n\nSummary by '+self._knowledge.my_name+':\n')
        print('jobs already taken:\n', self.get_jobs_already_taken())
        print('number of entries in job buffer:', self._job._jobs._buffer)
        print('resources bocked:\n', self._get_all_blocks())
        print('all meeting requests and replies:\n', self._get_all_meetings())
        
    
    def sync(self):
        while (self._so_heatbeat_steps != self._knowledge.step - 1).any():
            rospy.sleep(0.05)
        self._current_so_heartbeat = SOHeartBeat()
        self._current_so_heartbeat.agent_name = self._my_name
        self._current_so_heartbeat.simulation_step = self._knowledge.step
        
    def send_heartbeat(self):
        self._publisher.publish(self._current_so_heartbeat)
    
    def apply_for_job(self, job_id):
        job = SOJobMessage()
        job.timestamp = int(rospy.get_rostime().nsecs)
        job.agent_name = self._my_name
        job.job_id = job_id
        self._current_so_heartbeat.job.append(job)
        self._current_so_heartbeat.update_job = True
        
    def remove_job(self, job_id):
        job_remove = SOJobRemove()
        job_remove.job_id = job_id
        self._current_so_heartbeat.remove_job.append(job_remove)
        self._current_so_heartbeat.update_remove_job = True
        
        
    def get_jobs_already_taken(self):
        return self._job.get_jobs_already_taken()
    
    def request_meeting(self, role, position, step):
        msg = SOMeetingRequest()
        msg.timestamp = int(rospy.get_rostime().nsecs)
        msg.agent_name = self._my_name
        msg.role = str(role)
        msg.id = abs(hash(role+str(position)+str(step)+str(msg.timestamp)))
        msg.position = position
        msg.step = step
        self._current_so_heartbeat.requests.append(msg)
        self._current_so_heartbeat.update_request = True
        return msg.id
        
    def remove_meeting(self, related_id):
        msg = SOMeetingRemove()
        msg.related_id = related_id
        self._current_so_heartbeat.remove.append(msg)
        self._current_so_heartbeat.update_remove = True
        
    def reply_meeting(self, arrival_step, related_request):
        msg = SOMeetingReply()
        msg.related_id = related_request
        msg.timestamp = int(rospy.get_rostime().nsecs)
        msg.agent_name = self._my_name
        msg.detour_steps = 0
        msg.arrival_step = arrival_step
        self._current_so_heartbeat.replies.append(msg)
        self._current_so_heartbeat.update_reply = True
        
    def get_open_meeting_requests(self):
        return self._meeting.get_open_meeting_requests()
    
    def block_resource_at_storage(self, storage_name, item_name, item_amount):
        msg = SOResourceMessage()
        msg.timestamp = int(rospy.get_rostime().nsecs)
        msg.agent_name = self._knowledge.my_name
        msg.block_release = True
        msg.storage_name = storage_name
        msg.item_name = item_name
        msg.item_amount = item_amount
        self._current_so_heartbeat.resources.append(msg)
        self._current_so_heartbeat.update_resources = True
        
    def release_resource_at_storage(self, storage_name, item_name, item_amount):
        msg = SOResourceMessage()
        msg.timestamp = int(rospy.get_rostime().nsecs)
        msg.agent_name = self._knowledge.my_name
        msg.block_release = False
        msg.storage_name = storage_name
        msg.item_name = item_name
        msg.item_amount = item_amount
        self._current_so_heartbeat.resources.append(msg)
        self._current_so_heartbeat.update_resources = True
    
    def _get_all_blocks(self):
        return [(block.item_name, block.item_amount, block.storage_name) for block in self._resources._resources._buffer]
            
    def _get_all_meetings(self):
        meetings = []
        for request in self._meeting._requests._buffer:
            meeting_id = request.id
            meetings.append((meeting_id, request.agent_name, request.role, request.step))
            for reply in self._meeting._replies._buffer:
                if reply.related_id == meeting_id:
                    meetings.append((reply.agent_name, reply.arrival_step))
        return meetings
    
    def get_first_job(self):
        try:
            return self._job._jobs._buffer[9]
        except IndexError:
            return None
    
    def get_my_next_meeting(self):
        agent_order = self.get_agent_order()
        if not agent_order: return None
        my_meetings = self.get_my_meetings()
        for agent_name in agent_order:
            for meeting in my_meetings:
                if meeting.agent_name == agent_name: return meeting
        return None
    
    def get_all_replies_to_meeting(self, meeting_id):
        return self._meeting._get_all_reqlies_for_id(meeting_id)
    
    def get_my_meetings(self):
        return self._meeting.get_my_meetings()
    
    def is_my_resource(self, storage_name, item_name, item_amount):
        return self._resources.is_my_resource(storage_name, item_name, item_amount)
        
    def is_my_job(self, job_id):
        return self._job.is_my_job(job_id)
    
    def get_agent_order(self):
        return self._job.get_agent_order()
    
    def get_nb_jobs_before_mine(self):
        return self._job.get_nb_jobs_before_mine()
