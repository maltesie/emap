import numpy as np
import rospy


class PositionCache:
    
    def __init__(self, length):
        self._max_cache_length = length
        self._positions_by_name = {}
        self._name_cache = []
        self.count = 0
        
    def add_position(self, agent_name, position):
        try: self._positions_by_name[agent_name].append(position)
        except KeyError: 
            self._positions_by_name[agent_name] = []
            self._positions_by_name[agent_name].append(position)
        self._name_cache.append(agent_name)
        self.count += 1
        if self.count > self._max_cache_length: self._remove_first_position()
        
    def _remove_first_position(self):
        name = self._name_cache[0]
        del self._name_cache[0]
        del self._positions_by_name[name][0]
        self.count -= 1
        
    def get_all_positions(self):
        return [pos for agent in self._positions_by_name for pos in self._positions_by_name[agent]]
    
    def get_positions_for_agent(self, agent_name):
        return [pos for pos in self._positions_by_name[agent_name]]


class MyJob(object):

    def __init__(self, job_info, job_type, product_types, facility_positions):
        self._product_types = product_types
        self.job_type = job_type
        self.max_bid = None
        self.lowest_bid = None
        self.auction_time = None
        if job_type == 'auction':
            self.max_bid = job_info.max_bid
            self.lowest_bid = job_info.lowest_bid
            self.auction_time = job_info.auction_time
            job_info = job_info.job
        self.destination_name = job_info.storage_name
        self.goal_position = facility_positions[job_info.storage_name]
        self.id = job_info.id
        self.start = job_info.start
        self.end = job_info.end
        self.fine = job_info.fine
        self.reward = job_info.reward
        items = {product_type:0 for product_type in product_types}
        for item in job_info.items:
            items[item.name] = item.amount
        self._needed_items = items
        self.items = {item:self._needed_items[item] for item in self._needed_items}
        resources = {product_type:0 for product_type in product_types}
        required_roles = []
        for needed_item in items:
            for consumed_item in product_types[needed_item].consumed_items:
                resources[consumed_item.name] += items[needed_item]
            for required_role in product_types[needed_item].required_roles:
                required_roles.append(required_role)
        self.resources = resources
        self.required_roles = list(np.unique(required_roles))
        
    def set_items_and_resources(self, my_items):
        resources = {product_type:0 for product_type in self._product_types}
        required_roles = []
        for needed_item in self._needed_items:
            self.items[needed_item] = max(0, self._needed_items[needed_item] - my_items[needed_item])
            if self.items[needed_item] == 0: continue
            for consumed_item in self._product_types[needed_item].consumed_items:
                resources[consumed_item.name] += max(0, self.items[needed_item])
            for required_role in self._product_types[needed_item].required_roles:
                required_roles.append(required_role)
        self.required_roles = list(np.unique(required_roles))
        

class Knowledge:
    
    def __init__(self, agent_name, nb_agents):
        
        self.charging_stations = {}
        self.storages = {}
        self.workshops = {}
        self.dumps = {}
        self.shops = {}
        self.facility_positions = {}
        self.charging_station_positions = {}
        self.product_types = {}
        self.well_types = {}
        self.upgrade_types = {}
        self.massium = 0
        self.score = 0
        self.cell_size = 0.01
        self.proximity = 0.01
        self.min_lat = None
        self.max_lat = None
        self.min_long = None
        self.max_long = None
        self.map_name = "paris"
        self.team = None
        self.role = None
        self.simulation_running = False
        self._sim_id = ''
        self.step = 0
        self.max_steps = -1
        self.my_name = agent_name
        self.info_by_name = {}
        self.position_by_name = {}
        self.position_cache = PositionCache(100)
        self.my_info = None
        self.my_position = None
        self.auction_jobs = {}
        self.mission_jobs = {}
        self.priced_jobs = {}
        self.average_reward_by_item_amount = {}
        self._all_rewards_by_item_amount = {i:[] for i in range(100)}
        self.own_wells = {}
        self._temp_own_wells = {}
        self.opponent_wells = {}
        self._temp_opponent_wells ={}
        self.resources = {}
        self.resource_positions = {}
        self._knowledge_steps = np.zeros(nb_agents + 1)
        self.received_first_percept = False
        self.last_action = None
        self.last_action_result = None
        self._all_job_types = {}
        self._my_auctions = []
        self._failed_auctions = []
        self._nb_agents = nb_agents
        self.wells_in_vision = {}
        
    
    def _initialize(self, sim_start):
        self._sim_id = sim_start.simulation_id
        self.max_steps = sim_start.steps
        
        self.proximity = sim_start.proximity
        self.cell_size = sim_start.cell_size
        
        self.min_lat = sim_start.min_lat
        self.max_lat = sim_start.max_lat
        self.min_long = sim_start.min_lon
        self.max_long = sim_start.max_lon

        self.team = sim_start.team
        self.role = sim_start.role
        
        for product in sim_start.products:
            self.product_types[product.name] = product
        for upgrade in sim_start.upgrades:
            self.upgrade_types[upgrade.name] = upgrade 
        for well_type in sim_start.wells:
            self.well_types[well_type.name] = well_type 
    
        self.simulation_running = True
     
        
    def _set_wells(self):
        self.own_wells = self._temp_own_wells
        self.opponent_wells = self._temp_opponent_wells
        self._temp_opponent_wells = {}
        self._temp_own_wells = {}
    
    
    def _end_simulation(self, sim_end):
        self.simulation_running = False
    

    def _update_agents(self, agent):
        position = [agent.pos.lat, agent.pos.long]
        self.position_cache.add_position(agent.name, position)
        agent_number = int(agent.name[6:])
        if self.my_name == agent.name: 
            self.my_info = agent
            self.my_position = position
            self.last_action = agent.last_action
            self.last_action_result = agent.last_action_result
        else:
            self.info_by_name[agent.name] = agent
            self.position_by_name[agent.name] = position
        for resource in agent.known_resources:
            self.resources[resource.name] = resource
            self.resource_positions[resource.name] = [resource.pos.lat, resource.pos.long]
        
        for well in agent.known_wells:
            if well.team == self.team:
                self.own_wells[well.name] = well
                self.facility_positions[well.name] = [well.pos.lat, well.pos.long]
            else:
                self.opponent_wells[well.name] = well
                self.facility_positions[well.name] = [well.pos.lat, well.pos.long]
            
        self._knowledge_steps[agent_number] = agent.simulation_step
        
    
    def update_world(self, msg):
        
        self.massium = msg.team.massium
        self.score = msg.team.score
        self.step = msg.simulation_step
        
        self.sync_agents_to_step(self.step)
        #self._set_wells()
        
        for workshop in msg.workshops:
            self.workshops[workshop.name] = workshop
            self.facility_positions[workshop.name] = [workshop.pos.lat, workshop.pos.long]
            
        for charging_station in msg.charging_stations:
            self.charging_stations[charging_station.name] = charging_station
            self.facility_positions[charging_station.name] = [charging_station.pos.lat, charging_station.pos.long]
            self.charging_station_positions[charging_station.name] = [charging_station.pos.lat, charging_station.pos.long]
        
        for shop in msg.shops:
            self.shops[shop.name] = shop
            self.facility_positions[shop.name] = [shop.pos.lat, shop.pos.long]
    
        for dump in msg.dumps:
            self.dumps[dump.name] = dump
            self.facility_positions[dump.name] = [dump.pos.lat, dump.pos.long]
    
        for storage in msg.storages:
            self.storages[storage.name] = storage
            self.facility_positions[storage.name] = [storage.pos.lat, storage.pos.long]
            
        for auction in msg.auction_jobs:
            self.auction_jobs = {}
            job = MyJob(auction, 'auction', self.product_types, self.facility_positions)
            job.set_items_and_resources(self.get_my_items())
            self.auction_jobs[auction.job.id] = job
            self._all_job_types[auction.job.id] = 'auction'
    
        for priced in msg.priced_jobs:
            self.priced_jobs = {}
            job = MyJob(priced, 'priced', self.product_types, self.facility_positions)
            job.set_items_and_resources(self.get_my_items())
            self.priced_jobs[priced.id] = job
            nb_items = len(self.priced_jobs[priced.id].items)
            self._all_rewards_by_item_amount[nb_items].append(self.priced_jobs[priced.id].reward)
            self.average_reward_by_item_amount[nb_items] = np.mean(self._all_rewards_by_item_amount[nb_items])
            self._all_job_types[priced.id] = 'priced'
    
        for mission in msg.mission_jobs:
            self.mission_jobs = {}
            job = MyJob(mission, 'mission', self.product_types, self.facility_positions)
            job.set_items_and_resources(self.get_my_items())
            self.mission_jobs[mission.id] = job
            self._all_job_types[mission.id] = 'mission'
            
        for resource in msg.resources:
            self.resources[resource.name] = resource
            self.resource_positions[resource.name] = [resource.pos.lat, resource.pos.long]
        
        self.wells_in_vision = {}
        for well in msg.wells:
            self.wells_in_vision[well.name] = well
            if well.team == self.team:
                self.own_wells[well.name] = well
                self.facility_positions[well.name] = [well.pos.lat, well.pos.long]
            else:
                self.opponent_wells[well.name] = well
                self.facility_positions[well.name] = [well.pos.lat, well.pos.long]
        
        self._knowledge_steps[0] = self.step
        self.received_first_percept = True
        
    def sync_agents_to_step(self, step):
        while (self._knowledge_steps[1:] != step).any():
            rospy.sleep(0.05)
    
    def get_all_positions(self):
        return [self.facility_positions[facility] for facility in self.facility_positions]
    
    def get_all_resources(self):
        return [self.product_types[typ] for typ in self.product_types if not self.product_types[typ].consumed_items]

    def get_my_items(self):
        my_items = {typ:0 for typ in self.product_types}
        for item in self.my_info.items:
            my_items[item.name]=item.amount
        return my_items
    
    def get_my_resources_count(self):
        my_items = self.get_my_items()
        resource_types = [res.name for res in self.get_all_resources()]
        count = 0
        for item in my_items:
            if item in resource_types: count += my_items[item]
        return count
    
    def get_other_agents_items(self, other_agent_name):
        return {item.name:item.amount for item in self.info_by_name[other_agent_name].items}

    def get_known_resources(self):
        return list(np.unique([resource for resource in self.resources]))
    
    def get_all_job_ids(self):
        job_ids = []
        for job_id in self.auction_jobs: job_ids.append(job_id)
        for job_id in self.mission_jobs: job_ids.append(job_id)
        for job_id in self.priced_jobs: job_ids.append(job_id)
        return job_ids

    def add_auction(self, auction_id):
        if auction_id not in self._my_auctions:
            self._my_auctions.append(auction_id)
    
    def add_failed_auction(self, auction_id):
        if auction_id not in self._failed_auctions:
            self._failed_auctions.append(auction_id)