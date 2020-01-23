import numpy as np
from .task import TaskProvider
from scipy.spatial import cKDTree as tree

class GatherUpgradeStore():
    
    def __init__(self, knowledge, condition_provider, optimization_provider):
        
        self._knowledge = knowledge
        self._condition_provider = condition_provider
        self._positions_dict = {}
        self._known_resources = {}
        self._optimization_provider = optimization_provider
        for facility in self._knowledge.facility_positions:
            if facility.startswith('storage'):
                self._positions_dict[facility] = self._knowledge.facility_positions[facility]
        for resource in self._knowledge.resource_positions:
            self._positions_dict[resource] = self._knowledge.resource_positions[resource]
            self._known_resources[resource] = self._knowledge.resources[resource].item.name
        self._current_plan = None
        
    def _update_positions(self):
        for resource in self._knowledge.resource_positions:
            if resource not in self._positions_dict: 
                self._positions_dict[resource] = self._knowledge.resource_positions[resource]
                self._known_resources[resource] = self._knowledge.resources[resource].item.name
                
    def get_tasks(self):
        self._update_positions()
        optimizer = self._optimization_provider.gather_upgrade_store_optimizer(self._knowledge.my_position, self._positions_dict)
        optimizer.run(30)
        self._current_plan = optimizer.get_best_names()
        best_positions = optimizer.get_best_positions()
        tasks = []
        last_is_storage = False
        for i,goal in enumerate(self._current_plan):
            position_watcher = self._condition_provider.position_watcher(best_positions[i])
            if goal.startswith('node'):
                item_condition = self._condition_provider.n_items_present_condition(10, self._known_resources[goal])
                capacity_condition = self._condition_provider.no_capacity_for_item_condition(self._known_resources[goal])
                task = TaskProvider.gather_task(position_watcher, item_condition, capacity_condition)
                tasks.append(task)
                last_is_storage = False
            if goal.startswith('s'):
                item_condition = self._condition_provider.no_items_present_condition()
                task = TaskProvider.store_all_items_task(position_watcher, self._knowledge, item_condition)
                tasks.append(task)
                last_is_storage = True
        if not last_is_storage:
            last_position = tasks[-1].goal_position
            closest_dist = np.inf
            for facility in self._knowledge.facility_positions:
                if facility.startswith('storage'):
                    temp_dist = np.linalg.norm(np.array(last_position)-np.array(self._knowledge.facility_positions[facility]))
                    if temp_dist < closest_dist:
                        storage_pos = self._knowledge.facility_positions[facility]
                        closest_dist = temp_dist
            position_watcher = self._condition_provider.position_watcher(storage_pos)
            item_condition = self._condition_provider.no_items_present_condition()
            task = TaskProvider.store_all_items_task(position_watcher, self._knowledge, item_condition)
            tasks.append(task)
        return None, tasks
        
    
class ExploreBuildDismantle():
    
    def __init__(self, knowledge, condition_provider, optimization_provider, distance_calc, density_map_bins=10):
        self._knowledge = knowledge
        self._distance = distance_calc
        self._condition_provider = condition_provider
        self._optimization_provider = optimization_provider
        self.current_plan = None
        lat_pos, long_pos= np.meshgrid(np.linspace(self._knowledge.min_lat+10**(-1 * self._knowledge.proximity), \
                                                   self._knowledge.max_lat-10**(-1 * self._knowledge.proximity), density_map_bins), \
                                        np.linspace(self._knowledge.min_long+10**(-1 * self._knowledge.proximity), \
                                                    self._knowledge.max_long-10**(-1 * self._knowledge.proximity), density_map_bins))
        self._mean_dist = np.mean(((self._knowledge.max_lat - self._knowledge.min_lat)/density_map_bins, (self._knowledge.max_long - self._knowledge.min_long)/density_map_bins))
        self._grid = np.empty((density_map_bins**2, 2))
        self._grid[:,0] = lat_pos.flatten()
        self._grid[:,1] = long_pos.flatten()
        self._grid = np.round(self._grid, int(self._knowledge.proximity))
        self._grid_tree = tree(self._grid)
        self._density  = np.empty(density_map_bins**2)
        
    def _update_density_map(self):
        t_pos = tree(self._knowledge.position_cache.get_all_positions())
        self._density = np.array([len(neighbors) for neighbors in self._grid_tree.query_ball_tree(t_pos, self._mean_dist)])
        
    def get_tasks(self):
        self._update_density_map()
        optimizer = self._optimization_provider.explore_optimizer(self._grid, self._density)
        optimizer.run(20)
        self.current_plan = optimizer.get_best_positions()
        cant_build_a_well_condition = self._condition_provider.cant_build_a_well_condition()
        tasks = [TaskProvider.build_best_well_task(self._condition_provider.position_watcher(self.current_plan[0]), self._knowledge, cant_build_a_well_condition)]
        for pos in self.current_plan[1:]:
            tasks.append(TaskProvider.goto_task(self._condition_provider.position_watcher(pos)))
        for well in self._knowledge.opponent_wells:
            pos = self._knowledge.facility_positions[well]
            well_does_not_decrease_condition = self._condition_provider.well_does_not_decrease_condition(well)
            dismantle_task = TaskProvider.dismantle_task(self._condition_provider.position_watcher(pos), well_does_not_decrease_condition=well_does_not_decrease_condition)
            best_index = self._distance.shortest_detour_index(tasks, pos)
            tasks.insert(best_index, dismantle_task)
        #for well in self._knowledge.own_wells:
        #    pos = self._knowledge.facility_positions[well]
        #    well_does_not_improve_condition = self._condition_provider.well_does_not_improve_condition(well)
        #    build_up_task = TaskProvider.build_up_well_task(self._condition_provider.position_watcher(pos), well_does_not_imporve_condition=well_does_not_improve_condition)
        #    best_index = self._distance.shortest_detour_index(tasks, pos)
        #    tasks.insert(best_index, build_up_task)
        return None, tasks
                
                
class AssembleDeliver:
    
    def __init__(self, job_manager):
        self._job_manager = job_manager
        
    def get_tasks(self):
        job, mission_tasks = self._job_manager.get_mission_tasks()
        if mission_tasks is not None: return job, mission_tasks
        job, auction_tasks = self._job_manager.get_auction_tasks()
        if auction_tasks is not None: return job, auction_tasks
        job, priced_tasks = self._job_manager.get_priced_tasks()
        if priced_tasks is not None: return job, priced_tasks
        
        return None, []                                    