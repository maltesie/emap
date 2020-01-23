#parts copied from TUBDAI

import urllib.request, urllib.error, urllib.parse
from urllib.error import URLError
import json
import socket
import numpy as np
from .help_functions import pairwise

GRAPHHOPPER_DEFAULT_PORT = 8989
GRAPHHOPPER_URL_REQUEST_TIMEOUT = 1.0

class Distance(object):
    
    def __init__(self, knowledge):
        
        self._role_name = knowledge.role.name
        self._cell_size = knowledge.cell_size
        self._knowledge = knowledge
        self._speed = self._knowledge.my_info.speed * self._cell_size
        self._cache = {}
        self._proximity = knowledge.proximity
        self._map = None
        self.graphhopper_port = GRAPHHOPPER_DEFAULT_PORT
        
    def _request_street_distance(self, a, b):
        
        request = 'http://localhost:{}/route?instructions=false&calc_points=false&' \
                  'points_encoded=false&point={},{}&point={},{}'.format(self.graphhopper_port, a[0], a[1], b[0], b[1])
        try:
            connection = urllib.request.urlopen(request, timeout=GRAPHHOPPER_URL_REQUEST_TIMEOUT)
            response = connection.read().decode()
            parsed = json.loads(response)
            connection.close()
            distance = parsed['paths'][0]['distance']
            if distance:
                self._cache[str(a)+str(b)] = distance
                return distance
            else:
                raise LookupError('Graphhopper: Route not available for:'+request)

        except URLError as e:
            raise Exception("Graphhopper: URL error message for "+request+" :: " + str(e))
        except socket.timeout:
            raise Exception("Graphhopper socket timeout for: " + str(request))
            
    def between(self, a, b):
        a, b = np.array(a), np.array(b)
        if self.are_same(a,b): return 0
        if self._role_name == 'drone': return np.linalg.norm((a-b)*np.array([111320.0, 71500]))
        key = str(a)+str(b)
        if key in self._cache: return self._cache[key]
        else:
            try: return self._request_street_distance(a,b)
            except: return -1
        
    def steps_between(self, a, b):
        self._speed = self._knowledge.my_info.speed * self._cell_size
        d = self.between(a,b)
        return np.ceil(d/self._speed)
    
    def are_same(self, a, b):
        d = np.array(a) - np.array(b)
        return np.linalg.norm(d) <= 10**(-1 * self._proximity)
    
    def steps_per_task(self, tasks):
        task_pos = self._get_task_positions(tasks)
        actions_per_task = np.array([task.nb_actions for task in tasks])
        return np.array([self.steps_between(*pair) for pair in pairwise(task_pos)]) + actions_per_task
    
    def shortest_detour_index(self, tasks, position):
        detour_vals = self.all_detour_values(tasks, position)
        best_index = np.argmin(detour_vals)
        return best_index
    
    def all_detour_values(self, tasks, position):
        task_pos = self._get_task_positions(tasks)
        detour_vals = []
        for i, (posa, posb) in enumerate(pairwise(task_pos)):
            steps_to, steps_from = self.steps_between(posa, position), self.steps_between(position, posb)
            steps_without = self.steps_between(posa, posb)
            detour = steps_to + steps_from - steps_without
            detour_vals.append(detour)
        return detour_vals
            
    def _get_task_positions(self, tasks):
        own_position = self._knowledge.my_position
        task_pos = [own_position]
        for task in tasks:
            if task._has_position: task_pos.append(task.goal_position)
            else: task_pos.append(task_pos[-1])
        return task_pos
    
    
class SamePosition:
    
    def __init__(self, simulation_knowledge):
        self._proximity = simulation_knowledge.proximity
        
    def check(self, a, b):
        d = np.array(a) - np.array(b)
        return np.linalg.norm(d) <= 10**(-1 * self._proximity)