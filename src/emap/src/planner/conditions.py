from .distance import SamePosition
import numpy as np
        

class ConditionProvider:
    
    def __init__(self, knowledge, so_manager, distance_calc):
        self._knowledge = knowledge
        self._so_manager = so_manager
        self._distance = distance_calc
        
    def multi_condition(self, conditions):
        return MultiCondition(conditions=conditions)
        
    def roles_present_condition(self, item_to_assemble):
        return RolesPresent(item_to_assemble=item_to_assemble, knowledge=self._knowledge)
    
    def position_watcher(self, goal_position):
        return ReachedGoal(goal_position=goal_position, knowledge=self._knowledge)
    
    def n_items_present_condition(self, item_amount, item_type):
        return NItemsPresent(n=item_amount, item_name=item_type, knowledge=self._knowledge)
    
    def no_items_present_condition(self):
        return NoItemsPresent(knowledge=self._knowledge)
    
    def other_agent_present_condition(self, other_agent_name):
        return OtherAgentPresent(other_agent_name=other_agent_name, knowledge=self._knowledge)
    
    def other_agent_left_condition(self, other_agent_name):
        return OtherAgentLeft(other_agent_name=other_agent_name, knowledge=self._knowledge)
    
    def store_is_full_condition(self, storage_name, item_type, item_amount):
        return StorageIsFull(storage_name=storage_name, items_to_store=item_type, amount=item_amount)
    
    def massium_greater_than_x_condition(self, x):
        return MassiumGreaterThanX(x=x, knowledge=self._knowledge)
    
    def massium_lower_than_x_condition(self, x):
        return MassiumLowerThanX(x=x, knowledge=self._knowledge)
    
    def cant_build_a_well_condition(self):
        return CantBuildAWell(knowledge=self._knowledge)
    
    def no_capacity_for_item_condition(self, item_type):
        return NoCapacityForItem(item_name=item_type, knowledge=self._knowledge)
    
    def charge_at_least_x_condition(self, x):
        return ChargeAtLeastX(x=x, knowledge=self._knowledge)
    
    def can_reach_nearest_charging_station_condition(self):
        return CanReachNearestChargingStation(knowledge=self._knowledge, distance_calc=self._distance)

    def recharge_now_condition(self):
        return RechargeNow(knowledge=self._knowledge, distance_calc=self._distance)

    def all_resources_reserved_condition(self, so_resource, storages, items, amounts):
        return AllResourcesReserved(so_resource=so_resource, storages=storages, items=items, amounts=amounts)
    
    def stop_bidding_condition(self, job_id):
        return StopBidding(knowledge=self._knowledge, job_id=job_id)
    
    def meeting_removed_condition(self, meeting_id):
        return MeetingRemoved(meeting_id=meeting_id, so_manager=self._so_manager)
    
    def well_does_not_improve_condition(self, well_name):
        return WellDoesNotImprove(well_name=well_name, knowledge=self._knowledge)
    
    def well_does_not_decrease_condition(self, well_name):
        return WellDoesNotDecrease(well_name=well_name, knowledge=self._knowledge)


class MultiCondition:
    
    def __init__(self, conditions, combine_logic='and'):
        self._conditions = conditions
        self._combine_logic = combine_logic
    
    def check(self):
        conditions_evaluated = np.array([condition.check() for condition in self._conditions])
        if self._combine_logic == 'and':
            return conditions_evaluated.all()
        elif self._combine_logic == 'or':
            return conditions_evaluated.any()
        else: raise AssertionError('and or or')


class RolesPresent:
    
    def __init__(self, item_to_assemble, knowledge):
        self._knowledge = knowledge
        self._proximity = 10 ** (-1 * knowledge.proximity)
        self._needed_roles = knowledge.product_types[item_to_assemble].required_roles
        
    def check(self):
        #print('waiting for roles:', self._needed_roles)
        present_roles = [self._knowledge.role.name]
        names = [name for name in self._knowledge.position_by_name]
        a = np.array(self._knowledge.my_position)
        for other_agent in names:
            b = np.array(self._knowledge.position_by_name[other_agent])
            d = a-b
            if np.linalg.norm(d) <= self._proximity:
                present_roles.append(self._knowledge.info_by_name[other_agent].role)
        return all([role in present_roles for role in self._needed_roles])
      
        
class AllResourcesReserved:
    
    def __init__(self, so_resource, storages, items, amounts):
        self._so_resourcce = so_resource
        self._storages = storages
        self._items = items
        self._amounts = amounts
        
    def check(self):
        for i in range(len(self._storages)):
            if not self._so_resourcce.is_my_resource(self._storages[i], self._items[i], self._amounts[i]):
                return False
        return True
    
class ReachedGoal:
    
    def __init__(self, goal_position, knowledge):
        self.goal_position = goal_position
        self._distance_calc = SamePosition(knowledge)
        self._knowledge = knowledge
        
    def check(self):
        return self._distance_calc.check(self._knowledge.my_position, self.goal_position)

class NItemsPresent:
    
    def __init__(self, n, item_name, knowledge):
        self._knowledge = knowledge
        self._n = n
        self._item_name = item_name
        
    def check(self):
        for item in self._knowledge.my_info.items:
            if (item.name == self._item_name) and (item.amount >= self._n): return True
        return False
    
class NoItemsPresent:
    
    def __init__(self, knowledge):
        self._knowledge = knowledge
        self._resources = [item.name for item in knowledge.get_all_resources()]
        
    def check(self):
        carried_items = self._knowledge.get_my_items()
        carried_items_amount = sum([carried_items[item] for item in carried_items if item in self._resources])
        return (carried_items_amount == 0)
    
    
class StopBidding:
    
    def __init__(self, knowledge, job_id):
        self._knowledge = knowledge
        self._job_id = job_id
        nb_items = len(knowledge.auction_jobs[job_id].items)
        average = knowledge.average_reward_by_item_amount[nb_items]
        self._bids =[int(2*average), int(1.5*average), int(0.8*average)]
        self._my_lowest_bid = int(0.8*average)
        
    def check(self):
        if self._job_id not in self._knowledge.auction_jobs: return True
        current_lowest = self._knowledge.auction_jobs[self._job_id].lowest_bid
        lowest_check = (current_lowest < self._my_lowest_bid)
        if lowest_check: self._knowledge.add_failed_auction(self._job_id)
        my_bid_success_check = (int(current_lowest) in self._bids)
        if my_bid_success_check: self._knowledge.add_auction(self._job_id)
        check_job_active = self._job_id in self._knowledge.auction_jobs
        return lowest_check or my_bid_success_check or check_job_active


class OtherAgentPresent:
    
    def __init__(self, other_agent_name, knowledge):
        self._other_agent_name = other_agent_name
        self._knowledge = knowledge
        self._distance_calc = SamePosition(knowledge)
         
    def check(self):
        return self._distance_calc.check(self._knowledge.my_position, self._knowledge.position_by_name[self._other_agent_name])
 
    
class OtherAgentLeft:
    
    def __init__(self, other_agent_name, knowledge):
        self._other_agent_present_condition = OtherAgentPresent(other_agent_name, knowledge)
        self._other_agent_was_present = False
        self._checks_since_left = 0
         
    def check(self):
        if self._other_agent_present_condition.check(): 
            self._other_agent_was_present = True
        return self._other_agent_was_present and (not self._other_agent_present_condition.check())
 
    
class MeetingRemoved:
    
    def __init__(self, meeting_id, so_manager):
        self._meeting_id = meeting_id
        self._so_manager = so_manager
         
    def check(self):
        my_meeting = self._so_manager.get_my_next_meeting()
        if my_meeting is None: return True
        else: return my_meeting.id != self._meeting_id
  
    
class AllRepliedToMeetingRequest:
    
    def __init__(self, meeting_ids, so_manager):
        self._meeting_ids = meeting_ids
        self._so_manager = so_manager
        
    def check(self):
        for meeting_id in self._meeting_ids:
            replies = self._so_manager.get_all_replies_to_meeting(meeting_id)
            if not replies: return False
        return True
        
    
    
class StorageIsFull:
    
    def __init__(self, storage_name, items_to_store, amount, knowledge):
        self._storage_name = storage_name
        self._knowledge = knowledge
        self._needed_volume = sum([knowledge.product_types[item].volume * amount[i] for i,item in enumerate(items_to_store)])
        
    def check(self):
        free_volume = self._knowledge.storages[self._storage_name].total_capacity - self._knowledge.storages[self._storage_name].used_capacity
        return (free_volume <= self._needed_volume)
    
    
class WellDoesNotDecrease:
    def __init__(self, well_name, knowledge):
        self._well_name = well_name
        self._knowledge = knowledge
        self._last_well_integrity = np.inf
        
    def check(self):
        if self._well_name in self._knowledge.opponent_wells:
            decreased = (self._knowledge.opponent_wells[self._well_name].integrity < self._last_well_integrity)
            self._last_well_integrity = self._knowledge.opponent_wells[self._well_name].integrity
            return (not decreased)
        else: return True
    
    
class WellDoesNotImprove:
    
    def __init__(self, well_name, knowledge):
        self._well_name = well_name
        self._knowledge = knowledge
        self._last_well_integrity = 0
        
    def check(self):
        if self._well_name in self._knowledge.own_wells:
            improved = (self._knowledge.own_wells[self._well_name].integrity > self._last_well_integrity)
            self._last_well_integrity = self._knowledge.own_wells[self._well_name].integrity
            return (not improved)
        else: return True
        
    
class MassiumGreaterThanX:
    
    def __init__(self, x, knowledge):
        self._x = x
        self._knowledge = knowledge
        
    def check(self):
        return self._knowledge.massium > self._x
    
class MassiumLowerThanX:
    
    def __init__(self, x, knowledge):
        self._x = x
        self._knowledge = knowledge
        
    def check(self):
        return self._knowledge.massium < self._x
    
    
class CantBuildAWell:
    
    def __init__(self, knowledge):
        self._knowledge = knowledge
        
    def check(self):
        cheapest_well = min([self._knowledge.well_types[well].cost for well in self._knowledge.well_types])
        return (self._knowledge.massium < cheapest_well) or (self._knowledge.massium < 2000)
    
    
class ChargeAtLeastX:
    
    def __init__(self, x, knowledge):
        self._x = x
        self._knowledge = knowledge
        
    def check(self):
        return self._knowledge.my_info.charge >= self._x
    
    
class NoCapacityForItem:
    
    def __init__(self, item_name, knowledge):
        self.item_name = item_name
        self.item_volume = knowledge.product_types[item_name].volume
        self._knowledge = knowledge
        
    def check(self):
        return (self._knowledge.my_info.load_max - self._knowledge.my_info.load < self.item_volume)


class CanReachNearestChargingStation:
    
    def __init__(self, knowledge, distance_calc):
        self._knowledge = knowledge
        self._distance_calc = distance_calc
        self._charging_positions = np.array([self._knowledge.charging_station_positions[name] for name in self._knowledge.charging_station_positions])
        self._check_in_steps = 0
        
    def check(self):
        #t0 = time.time()
        self._check_in_steps -= 1
        own_position = self._knowledge.my_position
        max_steps = self._knowledge.my_info.charge - 1
        steps_needed = []
        if self._check_in_steps < 1:
            for charging_position in self._charging_positions:
                steps_needed.append(self._distance_calc.steps_between(own_position, charging_position))
            self._check_in_steps = int((max_steps - min(steps_needed)) / 2)
            #print('time spent in CanReachNearestChargingStation condition check', time.time() - t0)
            return (np.array(steps_needed) < max_steps).any()
        else: 
            #print('time spent in CanReachNearestChargingStation condition check', time.time() - t0)
            return True
    
class RechargeNow:
    
    def __init__(self, knowledge, distance_calc):
        self._knowledge = knowledge
        self._distance_calc = distance_calc
        self._charging_positions = np.array([self._knowledge.charging_station_positions[name] for name in self._knowledge.charging_station_positions])
        
    def check(self):
        own_position = self._knowledge.my_position
        max_steps = self._knowledge.my_info.charge + 1
        steps_needed = []
        for charging_position in self._charging_positions:
            steps_needed.append(self._distance_calc.steps_between(own_position, charging_position))
        return (np.array(steps_needed) < max_steps).any()