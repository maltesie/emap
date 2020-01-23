import numpy as np
from .help_functions import pairwise
import networkx as nx


class OptimizationProvider:
    
    def __init__(self, random=False):
        self._random = random
        
    def gather_upgrade_store_optimizer(self, own_position, positions_dict, population_size=60, mating_fraction=0.4, mutation_probability=0.1, elitism=0.05):
        if self._random:
            return GatherRandom(positions_dict)
        else:
            return GatherEvolution(own_position=own_position, positions_dict=positions_dict, population_size=population_size, mating_fraction=mating_fraction, mutation=mutation_probability, elitism=elitism)
        
    def explore_optimizer(self, positions, density, plan_size=5, population_size=50, mating_fraction=0.5, mutation_probability=0.1, elitism=0.05):
        if self._random:
            return ExploreRandom(positions=positions, plan_size=plan_size)
        else:
            return ExploreEvolution(positions=positions, density=density, plan_size=plan_size, population_size=population_size, mating_fraction=mating_fraction, mutation=mutation_probability, elitism=elitism)
        
    def retrieve_items_optimizer(self, own_position, storage_positions_per_item, population_size=100, mating_fraction=0.5, mutation_probability=0.1, elitism=0.05):
        if self._random:
            return RetrieveRandom(storage_positions_per_item=storage_positions_per_item)
        else:
            return RetreiveEvolution(own_position=own_position, positions_per_item=storage_positions_per_item, population_size=population_size, mating_fraction=mating_fraction, mutation=mutation_probability, elitism=elitism)


class RetreiveEvolution:

    def __init__(self, own_position, positions_per_item, population_size=100, mating_fraction=0.5, mutation=0.05, elitism=0.05):
        self._cs = []
        self._elitism = elitism
        self._own_position = own_position
        self._plan_size = len(positions_per_item)
        self._positions_per_item = positions_per_item
        self._item_trans = {i:item for i, item in enumerate(positions_per_item)}
        self._s = population_size
        self._ma = mating_fraction
        mating_split = int(self._s * self._ma)
        if mating_split % 2 != 0: mating_split +=1
        self._possible_mates_indices = np.arange(mating_split)
        self._mu = mutation
        self.fitnes = np.zeros(self._s)
        self._number_of_items = [len(positions_per_item[item]) for item in positions_per_item]
        self._possible_mutation_indices = [i for i,nb in enumerate(self._number_of_items) if nb>1]
        
        for i in range(self._s):
            solution_1 = np.random.permutation(self._plan_size)
            solution_2 = [np.random.choice(self._number_of_items[item]) for item in solution_1]
            self._cs.append([solution_1, solution_2])
        
        self._ranking()
        #print('random plan average length over {} samples: {} pm {}'.format(self._s, np.mean(self.fitnes), np.std(self.fitnes)))

    def _compute_distance_air(self, solution):
        positions = np.array([self._own_position]+[self._positions_per_item[self._item_trans[ii]][solution[1][i]] for i,ii in enumerate(solution[0])])
        d = np.diff(positions, axis=0)
        return np.linalg.norm(d, axis=1).sum()
                        
    def _check_mutation(self):
        return np.random.uniform() < self._mu
    
    def _mutation(self):
        elite = int(self._elitism*self._s)
        for i,solution in enumerate(self._cs[elite:]):
            i += elite
            if self._check_mutation():
                if np.random.rand()>0.7:
                    replace_1, replace_2 = np.random.permutation(self._plan_size)[:2]
                    temp = solution[0][replace_1]
                    solution[0][replace_1] = solution[0][replace_2]
                    solution[0][replace_2] = temp
                    temp = solution[1][replace_1]
                    solution[1][replace_1] = solution[1][replace_2]
                    solution[1][replace_2] = temp
                    self._cs[i] = solution
                else:
                    replace_item_pos = np.random.choice(self._possible_mutation_indices)
                    replace_item = solution[0][replace_item_pos]
                    replace_with = np.random.choice(self._number_of_items[replace_item])
                    solution[1][replace_item_pos] = replace_with
                    self._cs[i] = solution
    
    def _crossing(self):
        mating_pairs = np.random.permutation(self._possible_mates_indices).reshape(-1,2)
        for parents in mating_pairs:
            a, b = parents
            split = int(self._plan_size/2)
            offspring_0 = np.empty(self._plan_size, dtype=np.int)
            offspring_0[:split], offspring_0[split:] = self._cs[a][0][:split], self._cs[b][0][split:]
            offspring = [offspring_0, self._cs[a][1][:split]+self._cs[b][1][split:]]
            if np.unique(offspring[0]).size != self._plan_size: continue
            offspring_fitnes = self._compute_distance_air(offspring)
            comp = offspring_fitnes < self.fitnes
            if comp.any():
                replace_at = np.argmax(self.fitnes)
                self._cs[replace_at] = offspring
    
    def _ranking(self):
        for i,solution in enumerate(self._cs):
            self.fitnes[i] = self._compute_distance_air(solution)
        ranking = np.argsort(self.fitnes)
        new_cs = [self._cs[rank] for rank in ranking]
        self._cs = new_cs
    
    def run(self, generations=10):
        if self._plan_size == 1: 
            dists = [self._compute_distance_air(solution) for solution in [[[0], [i]] for i in range(self._number_of_items[0])]]
            self._cs[0] = [[0],[np.argmin(dists)]]
            return
        for episode in range(generations):
            self._crossing()
            self._mutation()
            self._ranking()
            #print('best plan length: {}'.format(self._compute_distance_air(self.get_best_solution())))
    
    def get_best_solution(self):
        return self._cs[0]
    
    def get_best_positions(self):
        solution = self.get_best_solution()
        positions = np.array([self._positions_per_item[self._item_trans[ii]][solution[1][i]] for i,ii in enumerate(solution[0])])
        return positions
    
    def get_best_storage_indices(self):
        solution = self.get_best_solution()
        return [(self._item_trans[ii], solution[1][i]) for i,ii in enumerate(solution[0])]


class ExploreEvolution:

    def __init__(self, positions, density, plan_size=5, population_size=50, mating_fraction=0.5, mutation=0.05, elitism=0.05):
        self._cs = []
        self._elitism = elitism
        self._plan_size = plan_size
        self._positions = positions
        self._density = density
        self._s = population_size
        self._ma = mating_fraction
        mating_split = int(self._s * self._ma)
        if mating_split % 2 != 0: mating_split +=1
        self._possible_mates_indices = np.arange(mating_split)
        self._mu = mutation
        self._fitnes = np.zeros((self._s, 2))
        self._ranks = np.zeros((self._s, 2))
        
        for i in range(self._s):
            solution = np.random.permutation(len(positions))[:self._plan_size]
            self._cs.append(solution)
        
        self._ranking()
        #print('random plan average length over {} samples: {} pm {}'.format(self._s, np.mean(self._fitnes, axis=0), np.std(self._fitnes, axis=0)))
        
    def _compute_distance_air(self, solution):
        positions = self._positions[solution]
        d = np.diff(positions, axis=0)
        return np.linalg.norm(d, axis=1).sum()
    
    def _compute_objectives(self, solution):
        return (self._compute_distance_air(solution), self._density[solution].sum())
    
    def _random_split_value(self):
        t = int(np.random.normal(loc=self._plan_size/2, scale=self._plan_size/4))
        if t<0 : t=0
        elif t>self._plan_size: t=self._plan_size
        return t
    
    def _check_mutation(self):
        return np.random.uniform() < self._mu
    
    def _mutation(self):
        elite = int(self._elitism*self._s)
        for i,solution in enumerate(self._cs[elite:]):
            i += elite
            if self._check_mutation():
                replace_at = np.random.choice(self._plan_size)
                replace_with = np.random.choice(len(self._positions))
                if replace_with not in solution:
                    solution[replace_at] = replace_with
                    self._cs[i] = solution
    
    def _crossing(self):
        mating_pairs = np.random.permutation(self._possible_mates_indices).reshape(-1,2)
        for parents in mating_pairs:
            a, b = parents
            split = self._random_split_value()
            offspring = np.empty(self._plan_size, dtype=np.int)
            offspring[:split], offspring[split:] = self._cs[a][:split], self._cs[b][split:]
            if np.unique(offspring).size != self._plan_size: continue
            offspring_fitnes = self._compute_objectives(offspring)
            comp_dens = offspring_fitnes[1] < self._fitnes[:,1]
            comp_length = offspring_fitnes[0] > self._fitnes[:,0]
            comp = comp_dens & comp_length
            if comp.any():
                replace_at = np.argmax(self._ranks.sum(1))
                self._cs[replace_at] = offspring
    
    def _ranking(self):
        for i,solution in enumerate(self._cs):
            self._fitnes[i] = self._compute_objectives(solution)
        self._ranks[:,0][np.flip(np.argsort(self._fitnes[:,0]))] = np.arange(self._s)
        self._ranks[:,1][np.argsort(self._fitnes[:,1])] = np.arange(self._s)
        ranking = np.argsort(self._ranks.sum(1))
        new_cs = [self._cs[ranking[i]] for i in range(ranking.size)]
        self._cs = new_cs
    
    def run(self, generations=10):
        for episode in range(generations):
            self._crossing()
            self._mutation()
            self._ranking()
            #print('best plan fitnes:', self._compute_objectives(self.get_best_solution()))
    
    def get_best_solution(self):
        return self._cs[0]
    
    def get_best_positions(self):
        return self._positions[self.get_best_solution()]
 
    
class GatherEvolution:

    def __init__(self, own_position, positions_dict, population_size=100, mating_fraction=0.3, mutation=0.05, elitism=0.02):
        self._cs = []
        self._elitism = elitism
        self._own_position = np.array(own_position)
        self._plan_size = len(positions_dict)
        self._positions_names = np.array([key if not key.startswith('storage') else 's' for key in positions_dict])
        self._positions = np.array([positions_dict[position] for position in positions_dict])
        self._s = population_size
        self._ma = mating_fraction
        mating_split = int(self._s * self._ma)
        if mating_split % 2 != 0: mating_split +=1
        self._possible_mates_indices = np.arange(mating_split)
        self._mu = mutation
        self.fitnes = np.zeros(self._s)
        
        for i in range(self._s):
            solution = np.random.permutation(self._plan_size)
            self._cs.append(solution)
        
        self._ranking()
        #print('random plan average length over {} samples: {} pm {}'.format(self._s, np.mean(self.fitnes), np.std(self.fitnes)))

    def _compute_distance_air(self, solution):
        positions = np.vstack((self._own_position, self._positions[solution]))
        d = np.diff(positions, axis=0)
        return np.linalg.norm(d, axis=1).sum()
                        
    def _check_mutation(self):
        return np.random.uniform() < self._mu
    
    def _mutation(self):
        elite = int(self._elitism*self._s)
        for i,solution in enumerate(self._cs[elite:]):
            i += elite
            if self._check_mutation():
                replace_1, replace_2 = np.random.permutation(self._plan_size)[:2]
                temp = solution[replace_1]
                solution[replace_1] = solution[replace_2]
                solution[replace_2] = temp
                self._cs[i] = solution

    def _crossing(self):
        mating_pairs = np.random.permutation(self._possible_mates_indices).reshape(-1,2)
        for parents in mating_pairs:
            a, b = parents
            edges = [pair for pair in pairwise(self._cs[a])] + [pair for pair in pairwise(self._cs[b])]
            g = nx.Graph()
            g.add_edges_from(edges)
            current_node = self._cs[a][0]
            offspring = [current_node]
            while g.number_of_nodes() > 0:
                if g.degree[current_node]<1: 
                    possible_nodes = [n for n in g.nodes() if n!=current_node]
                    if not possible_nodes: break
                    next_node = np.random.choice(possible_nodes)
                else:
                    lowest = np.inf
                    for node in g[current_node]:
                        if g.degree[node]<lowest:
                            next_node = node
                            lowest = g.degree[node]
                g.remove_node(current_node)
                current_node = next_node
                offspring.append(current_node)
            offspring_fitnes = self._compute_distance_air(offspring)
            comp = offspring_fitnes < self.fitnes
            if comp.any():
                replace_at = np.argmax(self.fitnes)
                self._cs[replace_at] = offspring
    
    def _ranking(self):
        for i,solution in enumerate(self._cs):
            self.fitnes[i] = self._compute_distance_air(solution)
            storage_check = (self._positions_names[np.array([pair for pair in pairwise(solution)])] == 's').all(1)
            self.fitnes[i] += self.fitnes[i]*storage_check.sum()/self._plan_size
        ranking = np.argsort(self.fitnes)
        new_cs = [self._cs[rank] for rank in ranking]
        self._cs = new_cs
    
    def run(self, generations=10):
        for episode in range(generations):
            self._crossing()
            self._mutation()
            self._ranking()
            #print('best plan length: {}'.format(self.fitnes[0]))
            
    def get_best_solution(self):
        return self._cs[0]
    
    def get_best_names(self):
        return self._positions_names[self.get_best_solution()]
    
    def get_best_positions(self):
        solution = self.get_best_solution()
        positions = self._positions[solution]
        return positions
    
    
class GatherRandom:
    
    def __init__(self, positions_dict):
        self._positions = np.array([positions_dict[pos] for pos in positions_dict])
        self._positions_names = np.array([pos for pos in positions_dict])
        
    def run(self, generations=10):
        self.solution = np.random.permutation(len(self._positions))
    
    def get_best_solution(self):
        return self.solution
    
    def get_best_positions(self):
        return self._positions[self.solution]
    
    def get_best_names(self):
        return self._positions_names[self.solution]
    
    
class ExploreRandom:
    
    def __init__(self, positions, plan_size):
        self._positions = positions
        self._plan_size = plan_size
        
    def run(self, generations=10):
        self.solution = np.random.permutation(len(self._positions))[:self._plan_size]
        
    def get_best_solution(self):
        return self.solution
    
    def get_best_positions(self):
        return self._positions[self.solution]
        
    
class RetrieveRandom:
    
    def __init__(self, storage_positions_per_item):
        self._storage_positions_per_item = storage_positions_per_item
        
    def run(self, generations=10):
        pass
    
    def get_best_storage_indices(self):
        return [(item_name, 0) for item_name in self._storage_positions_per_item]
    
    def get_best_positions(self):
        return np.array([self._storage_positions_per_item[item][index] for item, index in self.get_best_storage_indices()])