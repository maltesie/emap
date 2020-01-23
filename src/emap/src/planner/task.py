import numpy as np
from diagnostic_msgs.msg import KeyValue
    
valid_actions = np.array(['goto', 'give', 'receive', 'store', 'retrieve', 'retrieve_delivered', 
                        'assemble', 'assist_assemble', 'dump', 'charge', 'recharge', 'gather',
                        'build', 'dismantle', 'trade', 'continue', 'noAction', 'buy', 'resell', 
                        'resell_all', 'deliver_job', 'bid_for_job', 'abort', 'skip', 'post_job', 
                        'repair', 'upgrade'])

facility_actions = np.array(['store', 'retrieve', 'retrieve_delivered', 'dump', 'charge', 'gather',
                        'dismantle', 'trade', 'buy', 'resell', 'resell_all', 'deliver_job',])


class AllOfOneItemType:
    
    def __init__(self, knowledge):
        self._knowledge = knowledge
        self._resources = [item.name for item in knowledge.get_all_resources()]
        
    def evaluate(self):
        carried_items_amount = self._knowledge.get_my_items()
        for item in carried_items_amount:
            if (carried_items_amount[item] == 0) or (item not in self._resources): continue
            return [KeyValue('Item', item), KeyValue('Amount', str(carried_items_amount[item]))]

class BestWellType:
    
    def __init__(self, knowledge):
        self._knowledge = knowledge
        
    def evaluate(self):
        most_expensive, best_well_type = 0, None
        for well_type in self._knowledge.well_types:
            cost = self._knowledge.well_types[well_type].cost
            if (cost<self._knowledge.massium) and (cost>most_expensive): 
                most_expensive = cost
                best_well_type = well_type
                #print(best_well_type)
        return [KeyValue('Well', best_well_type)]
    
class BestBid:
    
    def __init__(self, job_id, knowledge):
        self._job_id = job_id
        self._knowledge = knowledge
        nb_items = len(knowledge.auction_jobs[job_id].items)
        average = knowledge.average_reward_by_item_amount[nb_items]
        self._bids =[int(2*average), int(1.5*average), int(0.8*average)]
        
    def evaluate(self):
        if self._job_id not in self._knowledge.auction_jobs: return [KeyValue('Job', self._job_id), KeyValue('Bid', str(int(self._bids[0])))]
        current_lowest_bid = self._knowledge.auction_jobs[self._job_id].lowest_bid
        if current_lowest_bid == -1: current_lowest_bid = np.inf
        for bid in self._bids:
            if current_lowest_bid>bid:break
        #print('Im bidding {} for {}'.format(bid, self._job_id))
        return [KeyValue('Job', self._job_id), KeyValue('Bid', str(int(bid)))]
            
class Action(object):
    
    def __init__(self, action_type, params=[], needs_evaluation=False, action_name='default'):
        
        self.action_type = action_type
        #assert self.action_type in valid_actions, 'Type of action not valid'
        
        self.name = action_name
        self.params = params  
        self._needs_evaluation = needs_evaluation


class ActionProvider(object):
    
    def goto_action(position):
        params=[KeyValue('latitude', str(position[0])), KeyValue('longitude', str(position[1]))]
        return Action(action_type='goto', params=params, action_name='goto_'+str(position[0])+'_'+str(position[1]))
    
    def goto_facility_action(facility_name):
        params = [KeyValue('Facility', facility_name)]
        return Action(action_type='goto', params=params, action_name='goto_facility_'+facility_name)
        
    def give_action(to_agent, item_name, amount=1):
        params = [KeyValue('Agent', to_agent), KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='give', params=params, action_name='give_'+item_name+'_'+to_agent)
    
    def receive_action():
        return Action(action_type='receive', action_name='receive')
    
    def store_action(item_name, amount=1):
        params = [KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='store', params=params, action_name='store_'+item_name)
    
    def store_evaluate_action(knowledge):
        params = AllOfOneItemType(knowledge)
        return Action(action_type='store', params=params, needs_evaluation=True, action_name='store_evaluate')
    
    def retrieve_action(item_name, amount=1):
        params = [KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='retrieve', params=params, action_name='retrieve_'+item_name)
    
    def retrieve_delivered_action(item_name, amount=1):
        params = [KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='retrieve_delivered', params=params, action_name='retrieve_delivered_'+item_name)
    
    def assemble_action(item_name):
        params = [KeyValue('Item', item_name)]
        return Action(action_type='assemble', params=params, action_name='assemble_'+item_name)
    
    def assist_assemble_action(to_agent):
        params = [KeyValue('Agent', to_agent)]
        return Action(action_type='assist_assemble', params=params, action_name='assist_assemble_'+to_agent)
    
    def deliver_job_action(job_name):
        params = [KeyValue('Job', job_name)]
        return Action(action_type='deliver_job', params=params, action_name='deliver_'+job_name)
    
    def bid_action(job_name, bid):
        params = [KeyValue('Job', job_name), KeyValue('Bid', str(bid))]
        return Action(action_type='bid_for_job', params=params, action_name='bid_for_'+job_name)
    
    def best_bid_evaluate_action(job_name, knowledge):
        params = BestBid(job_id=job_name, knowledge=knowledge)
        return Action(action_type='bid_for_job', params=params, needs_evaluation=True, action_name='best_bid_for_'+job_name)
    
    def dump_action(item_name, amount=1):
        params = [KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='dump', params=params, action_name='dump_'+item_name)
    
    def upgrade_load_action():
        params = [KeyValue('Type', 'load')]
        return Action(action_type='upgrade', params=params, action_name='upgrade_load')
    
    def charge_action():
        return Action(action_type='charge', action_name='charge')
    
    def recharge_action():
        return Action(action_type='recharge', action_name='recharge')
    
    def skip_action():
        return Action(action_type='skip', action_name='skip')
    
    def gather_action():
        return Action(action_type='gather', action_name='gather')
    
    def build_new_well_action(well_type):
        params=[KeyValue('Well', well_type)]
        return Action(action_type='build', params=params, action_name='build_new_well_'+well_type)
    
    def build_new_well_evaluate_action(knowledge):
        params = BestWellType(knowledge)
        return Action(action_type='build', params=params, needs_evaluation=True, action_name='build_best_well')
    
    def build_up_well_action():
        return Action(action_type='build', action_name='build_up_well')
    
    def dismantle_action():
        return Action(action_type='dismantle', action_name='dismantle')
    
    def trade_action(item_name, amount=1):
        params = [KeyValue('Item', item_name), KeyValue('Amount', str(amount))]
        return Action(action_type='trade', params=params, action_name='trade_'+item_name)
    
    def continue_action():
        return Action(action_type='continue', action_name='continue')


class Task(object):
    
    def __init__(self, actions, name='default'):
        
        self.name = name
        self.actions = actions
        self.nb_actions = len(actions)
        self.at = 0
        self._has_position = False
        
    def current_action(self):
        try: current_action = self.actions[self.at]
        except IndexError: current_action = None
        return current_action
        
    def step(self):
        self.at += 1
        
    def reset_last_action(self):
        self.at -= 1
        
    def is_done(self):
        return self.current_action() is None
        
class PositionTask(Task):
    
    def __init__(self, actions, goal_position, position_watcher, name='position_task'):
        super(PositionTask, self).__init__(actions, name)
        self.goal_position = goal_position
        self._position_watcher = position_watcher
        self._has_position = True
        
    def current_action(self):
        if not self._position_watcher.check(): self.at=0
        return super(PositionTask, self).current_action()
    
    #def step(self):
    #    if self._position_watcher.check(): super(PositionTask, self).step()
        
        
class RepeatTask(Task):
   
    def __init__(self, actions, condition, repeat_at=-1, name='repeat_task'):
        super(RepeatTask, self).__init__(actions, name)
        self.condition = condition
        self.repeat_index = repeat_at
    
    def current_action(self):
        if (self.at == self.repeat_index + 1) and not self.condition.check(): self.at = self.repeat_index
        return super(RepeatTask, self).current_action()
    
    
class ChoosePositionTask(PositionTask):
    
    def __init__(self, actions_true, actions_false, goal_position, position_watcher, condition, name='choose_position_task'):
        super(ChoosePositionTask, self).__init__(actions_false, goal_position, position_watcher, name)
        self.actions_alternative = actions_true
        self.condition = condition
        self._initialized = False
        
    def current_action(self):
        if not self._initialized:
            if self.condition.check(): self.actions = self.actions_alternative
            self._initialized = True
        return super(ChoosePositionTask, self).current_action()
        
        
class SkipPositionTask(PositionTask):
    
    def __init__(self, actions, goal_position, position_watcher, condition, skip_at=-1, name='skip_position_task'):
        super(SkipPositionTask, self).__init__(actions, goal_position, position_watcher, name)
        self.condition = condition
        self.skip_index = skip_at
        
    def current_action(self):
        if (self.at == self.skip_index) and self.condition.check(): self.at += 1
        return super(WaitPositionTask, self).current_action()
        

class WaitPositionTask(PositionTask):
    
    def __init__(self, actions, goal_position, position_watcher, condition, wait_at=-1, name='wait_position_task'):
        super(WaitPositionTask, self).__init__(actions, goal_position, position_watcher, name)
        self.condition = condition
        self.wait_index = wait_at
        
    def current_action(self):
        super(WaitPositionTask, self).current_action()
        if (self.at != self.wait_index) or self.condition.check(): return super(WaitPositionTask, self).current_action()
        else: return ActionProvider.continue_action()
    
    def step(self):
        if (self.at != self.wait_index) or self.condition.check(): super(WaitPositionTask, self).step()


class RepeatPositionTask(PositionTask):
    
    def __init__(self, actions, goal_position, position_watcher, condition, repeat_at=-1, name='repeat_position_task'):
        super(RepeatPositionTask, self).__init__(actions, goal_position, position_watcher, name)
        self.condition = condition
        self.repeat_index = repeat_at
        self.deactivated = False
        
    def current_action(self):
        if (self.at == self.repeat_index + 1) and not self.condition.check(): self.at = self.repeat_index
        return super(RepeatPositionTask, self).current_action()
    

class CancelPositionTask(PositionTask):
    
    def __init__(self, actions, goal_position, position_watcher, condition, cancel_at, name='cancel_position_task'):
        super(CancelPositionTask, self).__init__(actions, goal_position, position_watcher, name)
        self.condition = condition
        self.cancel_at = cancel_at
    
    def current_action(self):
        if self.condition.check() and (self.at==self.cancel_at): self.at = len(self.actions)
        return super(CancelPositionTask, self).current_action()
        
        
class CancelRepeatPositionTask(PositionTask):
    
    def __init__(self, actions, goal_position, position_watcher, repeat_condition, repeat_at, cancel_condition, cancel_at, name='cancel_repeat_position_task'):
        super(CancelRepeatPositionTask, self).__init__(actions, goal_position, position_watcher, name)
        self.cancel_condition = cancel_condition
        self.cancel_at = cancel_at
        self.repeat_condition = repeat_condition
        self.repeat_index = repeat_at
        
    def current_action(self):
        if self.cancel_condition.check() and (self.at==self.cancel_at): self.at = len(self.actions)
        elif (self.at == self.repeat_index + 1) and not self.repeat_condition.check(): self.at = self.repeat_index
        return super(CancelRepeatPositionTask, self).current_action()
        
        
class TaskProvider(object):
    
    def single_task(action):
        return Task([action])
    
    def skip_task():
        return Task([ActionProvider.continue_action()], name='continue')
    
    def bid_task(job_name, bid):
        return Task([ActionProvider.bid_action(job_name, bid)], name='bid_for_'+job_name)
    
    def best_bid_task(job_name, knowledge, stop_bidding_condition):
         return RepeatTask([ActionProvider.best_bid_evaluate_action(job_name, knowledge)], condition=stop_bidding_condition, repeat_at=0, name='best_bid_for_'+job_name+'_'+str(knowledge.auction_jobs[job_name].start + knowledge.auction_jobs[job_name].auction_time))
    
    def goto_task(position_watcher):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        return PositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher)
    
    def gather_task(position_watcher, n_items_condition, free_capacity_for_item_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.gather_action()]
        return CancelRepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, repeat_condition=n_items_condition, repeat_at=1, cancel_condition=free_capacity_for_item_condition, cancel_at=1, name='gather_'+n_items_condition._item_name)
    
    def store_task(position_watcher, item_names, item_amounts, store_is_full_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        for i, item_name in enumerate(item_names):
            actions += [ActionProvider.store_action(item_name, item_amounts[i])]
        return CancelPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=store_is_full_condition, cancel_at=1, name='store')
    
    def store_all_items_task(position_watcher, knowledge, no_items_present_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.store_evaluate_action(knowledge)]
        return CancelRepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, repeat_condition=no_items_present_condition, repeat_at=1, cancel_condition=no_items_present_condition, cancel_at=1, name='store_all')
    
    def retrieve_task(position_watcher, item_names, item_amounts, name='retrieve'):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        for i, item_name in enumerate(item_names):
            actions += [ActionProvider.retrieve_action(item_name, item_amounts[i])]
        return PositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, name=name)
    
    def dump_task(position_watcher, item_names, item_amounts):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        for i, item_name in enumerate(item_names):
            actions += [ActionProvider.dump_action(item_name, item_amounts[i])]
        return PositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, name='dump')
    
    def charge_task(position_watcher, charge_at_least_x_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.charge_action()]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=charge_at_least_x_condition, repeat_at=1, name='charge')
    
    def recharge_task(position_watcher, charge_at_least_x_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.recharge_action()]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=charge_at_least_x_condition, repeat_at=1, name='recharge')
        
    def recharge_now_task(can_reach_nearest_charging_station_condition):
        actions = [ActionProvider.recharge_action()]
        return RepeatTask(actions=actions, condition=can_reach_nearest_charging_station_condition, repeat_at=0, name='recharge_now')
    
    def deliver_job_task(position_watcher, job_name):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.deliver_job_action(job_name)]
        return PositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, name='deliver_job_'+job_name)
    
    def give_task(position_watcher, to_agent, item_names, item_amounts, other_agent_present_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        for i, item_name in enumerate(item_names):
            actions += [ActionProvider.give_action(to_agent, item_name, item_amounts[i])]
        return WaitPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=other_agent_present_condition, wait_at=1, name='give_to_'+to_agent)
    
    def upgrade_load_task(position_watcher, massium_lower_than_x_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.upgrade_load_action()]
        return CancelPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=massium_lower_than_x_condition, cancel_at=1, name='upgrade_load')
    
    def receive_task(position_watcher, other_agent_left_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.receive_action()]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=other_agent_left_condition, repeat_at=1, name='receive')
    
    def assemble_task(position_watcher, item_name, item_amount, roles_present_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)] + [ActionProvider.assemble_action(item_name) for i in range(item_amount)]
        return WaitPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=roles_present_condition, wait_at=1, name='assemble_'+item_name+'_'+str(item_amount))
    
    def assist_assemble_task(position_watcher, other_agent_left_condition, other_agent_name, name='assist_assemble'):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.assist_assemble_action(other_agent_name)]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=other_agent_left_condition, repeat_at=1, name=name)
    
    def build_new_well_task(position_watcher, well_type, massium_greater_than_x_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.build_new_well_action(well_type)]
        return CancelPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=massium_greater_than_x_condition, cancel_at=1, name='build_new_well')
    
    def build_best_well_task(position_watcher, knowledge, can_build_a_well_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.build_new_well_evaluate_action(knowledge)]
        return CancelPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=can_build_a_well_condition, cancel_at=1, name='build_new_well')
    
    def build_up_well_task(position_watcher, well_does_not_imporve_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.build_up_well_action()]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=well_does_not_imporve_condition, repeat_at=1, name='improve_well_'+well_does_not_imporve_condition._well_name)
    
    def dismantle_task(position_watcher, well_does_not_decrease_condition):
        actions = [ActionProvider.goto_action(position_watcher.goal_position), ActionProvider.dismantle_action()]
        return RepeatPositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, condition=well_does_not_decrease_condition, repeat_at=1, name='dismantle_well_'+well_does_not_decrease_condition._well_name)
    
    def trade_task(position_watcher, item_names, item_amounts):
        actions = [ActionProvider.goto_action(position_watcher.goal_position)]
        for i, item_name in enumerate(item_names):
            actions += [ActionProvider.trade_action(item_name, item_amounts[i])]
        return PositionTask(actions=actions, goal_position=position_watcher.goal_position, position_watcher=position_watcher, name='trade')