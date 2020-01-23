import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

class SimulationResults:
    
    
    def __init__(self):
        self._results_per_agent = {}
        self._nb_sims = 0
        self._sim_length = None
        self._steps = None
    
    def add_new_sim(self, agent):
        try:
            self._results_per_agent[agent].append([])
        except:
            self._results_per_agent[agent] = [[]]
        if not 'B' in agent: self._nb_sims += 1
    
    def add_step_to_current_sim(self, line):
        agent, step, failed_jobs, success_jobs, massium, score, own_wells, \
        failed_auctions, success_auctions, bid_on_auctions, failed_missions, \
        success_missions, res1, res2, res3, res4, res5 = line.split(' ')
        self._results_per_agent[agent][-1].append([int(step), int(massium),\
                               int(score), int(own_wells), int(failed_jobs),\
                               int(failed_auctions), int(failed_missions), \
                               int(success_jobs), int(success_auctions), int(success_missions), \
                               int(bid_on_auctions), int(res1), int(res2), int(res3), \
                               int(res4), int(res5)])
        
    def _get_shortest_sim_length(self):
        shortest = np.inf
        for agent in self._results_per_agent:
            for res in self._results_per_agent[agent]:
                if len(res)<shortest: shortest = len(res)
        return shortest
    
    def _get_nb_sims(self):
        return int(self._nb_sims / 6)
    
    def switch_AB(self):
        temp = {agent:self._results_per_agent[agent] for agent in self._results_per_agent}
        self._results_per_agent = self._results_per_agent_opponent
        self._results_per_agent_opponent = temp
    
    def finalize(self):
        length = self._get_shortest_sim_length()
        for agent in self._results_per_agent:
            for i,sim in enumerate(self._results_per_agent[agent]):
                self._results_per_agent[agent][i] = np.array(sim)[:length]
        self._sim_length = length
        for agent in self._results_per_agent:
            steps = self._results_per_agent[agent][0][:,0]
            break
        self._steps = steps
        self._results_per_agent_opponent = {}
        remove = []
        for agent in self._results_per_agent:
            if 'B' in agent: 
                self._results_per_agent_opponent[agent] = self._results_per_agent[agent]
                remove.append(agent)
        for agent in remove:
            del self._results_per_agent[agent]
            
    def draw_massium(self, fname):
        for agent in self._results_per_agent:
            massium = np.empty((self._get_nb_sims(), len(self._steps)), dtype = np.int)
            for i,sim in enumerate(self._results_per_agent[agent]):
                massium[i]=sim[:,1]
            massium[:,1:] = np.diff(massium, axis=1)
            massium[massium<0] = 0
            massium = np.cumsum(massium, axis=1)
            massium_std = np.round(massium.std(0), 1)
            massium = np.round(massium.mean(0), 1)
            break
        fig, ax = plt.subplots()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.xlabel('Simulation Step' , fontsize = 16)
        plt.ylabel('Massium' , fontsize = 16)
        plt.plot(self._steps, massium)
        print('max massium: {} $\pm$ {}'.format(massium[-1], massium_std[-1]))
        plt.tight_layout()
        plt.savefig(fname)
        plt.close()

        
    def draw_score(self, fname):
        for agent in self._results_per_agent:
            score = np.empty((self._get_nb_sims(), len(self._steps)), dtype = np.int)
            for i,sim in enumerate(self._results_per_agent[agent]):
                score[i]=sim[:,2]
            score_std = np.round(score.std(0), 1)
            score = np.round(score.mean(0), 1)
            break
        fig, ax = plt.subplots()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.xlabel('Simulation Step' , fontsize = 16)
        plt.ylabel('Score' , fontsize = 16)
        plt.plot(self._steps, score)
        print('max score: {} $\pm$ {}'.format(score[-1], score_std[-1]))
        plt.tight_layout()
        plt.savefig(fname)
        plt.close()

        
    def draw_resources_gatherer(self, fname):
        resources = np.zeros((self._get_nb_sims(), len(self._steps), 6), dtype = np.int)
        for agent in self._results_per_agent:
            if agent in ['agentA1', 'agentA5', 'agentA6', 'agentB1', 'agentB5', 'agentB6']: continue
            for i,sim in enumerate(self._results_per_agent[agent]):
                resources[i,:,:5] += sim[:,11:16]
                resources[i,:,5] += sim[:,11:16].sum(1)
        resources_std = np.round(resources.std(0), 1)
        resources = np.round(resources.mean(0), 1)
        fig, ax = plt.subplots()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.xlabel('Simulation Step' , fontsize = 16)
        plt.ylabel('Resources Amount' , fontsize = 16)
        labels = ['res 1', 'res 2', 'res 3', 'res 4', 'res 5', 'total']
        for i in range(resources.shape[1]):
            plt.plot(self._steps, resources[:,i], label=labels[i])
        plt.legend()
        print_numbers = np.hstack((resources[-1].reshape((-1,1)), resources_std[-1].reshape((-1,1)))).flatten()
        print('max gather resources: {} $\pm$ {},  {} $\pm$ {},  {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {},  {} $\pm$ {}'.format(*print_numbers))
        plt.tight_layout()
        plt.savefig(fname)
        plt.close()

     
    def draw_resources_assembly(self, fname):
        resources = np.zeros((self._get_nb_sims(), len(self._steps), 6), dtype = np.int)
        for agent in self._results_per_agent:
            if agent in ['agentA2', 'agentA3', 'agentA4', 'agentB2', 'agentB3', 'agentB4']: continue
            for i,sim in enumerate(self._results_per_agent[agent]):
                resources[i,:,:5] += sim[:,11:16]
                resources[i,:,5] += sim[:,11:16].sum(1)
        resources_std = np.round(resources.std(0), 1)
        resources = np.round(resources.mean(0), 1)
        fig, ax = plt.subplots()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.xlabel('Simulation Step' , fontsize = 16)
        plt.ylabel('Resources Amount' , fontsize = 16)
        labels = ['res 1', 'res 2', 'res 3', 'res 4', 'res 5', 'total']
        for i in range(resources.shape[1]):
            plt.plot(self._steps, resources[:,i], label=labels[i])
        plt.legend()
        print_numbers = np.hstack((resources[-1].reshape((-1,1)), resources_std[-1].reshape((-1,1)))).flatten()
        print('max assembly resources: {} $\pm$ {},  {} $\pm$ {},  {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {},  {} $\pm$ {}'.format(*print_numbers))
        plt.tight_layout()
        plt.savefig(fname)
        plt.close()

        
    def compute_jobs(self):
        success_jobs = np.zeros((self._get_nb_sims(), 4))
        failed_jobs = np.zeros((self._get_nb_sims(), 4))
        bid_on_auctions = np.zeros(self._get_nb_sims())
        wells = np.zeros(self._get_nb_sims())
        for agent in self._results_per_agent:
            if agent in ['agentA2', 'agentA3', 'agentA4', 'agentA1', 'agentB2', 'agentB3', 'agentB4', 'agentB1']: continue
            for i, sim in enumerate(self._results_per_agent[agent]):
                t = np.diff(sim[:, 3])
                t[t<0] = 0
                wells[i] = np.cumsum(t)[-1]
                print(i,agent,wells[i])
                success_jobs[i][0] += sim[-1][7]
                success_jobs[i][1] += sim[-1][7] - (sim[-1][8] + sim[-1][9])
                success_jobs[i][2] += sim[-1][8]
                success_jobs[i][3] += sim[-1][9]
                failed_jobs[i][0] += sim[-1][4]
                failed_jobs[i][1] += sim[-1][4] - (sim[-1][5] + sim[-1][6])
                failed_jobs[i][2] += sim[-1][5]
                failed_jobs[i][3] += sim[-1][6]
                bid_on_auctions[i] += sim[-1][10]
        wells_std = np.round(wells.std(), 1)
        wells = np.round(wells.mean(), 1)
        success_jobs_std = np.round(success_jobs.std(0), 1)
        success_jobs = np.round(success_jobs.mean(0),1)
        failed_jobs_std = np.round(failed_jobs.std(0), 1)
        failed_jobs = np.round(failed_jobs.mean(0), 1)
        bid_on_auctions_std = np.round(bid_on_auctions.std(), 1)
        bid_on_auctions = np.round(bid_on_auctions.mean(),1)
        print('jobs: {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}, {} $\pm$ {}'.format(\
              success_jobs[0],success_jobs_std[0],success_jobs[1],success_jobs_std[1],success_jobs[2],success_jobs_std[2],success_jobs[3],success_jobs_std[3],\
              failed_jobs[0],failed_jobs_std[0],failed_jobs[1],failed_jobs_std[1],failed_jobs[2],failed_jobs_std[2],failed_jobs[3],failed_jobs_std[3],\
              bid_on_auctions, bid_on_auctions_std, wells, wells_std))

        
log_file = 'random_2.log'
simres = SimulationResults()
with open(log_file, 'r') as fp:
   line = fp.readline()
   cnt = 1
   while line:
       if line.endswith('new simulation\n'):
           agent = line.split(' ')[0]
           simres.add_new_sim(agent)
       else: 
           simres.add_step_to_current_sim(line)
       line = fp.readline()
       cnt += 1
       
simres.finalize()
simres.draw_massium('./plots/random_massium.png')
simres.draw_score('./plots/random_score.png')
simres.draw_resources_assembly('./plots/random_res_assemble.png')
simres.draw_resources_gatherer('./plots/random_res_gather.png')
simres.compute_jobs()

log_file = 'emap_2.log'
simres = SimulationResults()
with open(log_file, 'r') as fp:
   line = fp.readline()
   cnt = 1
   while line:
       if line.endswith('new simulation\n'):
           agent = line.split(' ')[0]
           simres.add_new_sim(agent)
       else: 
           simres.add_step_to_current_sim(line)
       line = fp.readline()
       cnt += 1
       
simres.finalize()
simres.draw_massium('./plots/emap_massium.png')
simres.draw_score('./plots/emap_score.png')
simres.draw_resources_assembly('./plots/emap_res_assemble.png')
simres.draw_resources_gatherer('./plots/emap_res_gather.png')
simres.compute_jobs()

log_file = 'vs_2.log'
simres = SimulationResults()
with open(log_file, 'r') as fp:
   line = fp.readline()
   cnt = 1
   while line:
       if line.endswith('new simulation\n'):
           agent = line.split(' ')[0]
           simres.add_new_sim(agent)
       else: 
           simres.add_step_to_current_sim(line)
       line = fp.readline()
       cnt += 1
       
simres.finalize()
simres.draw_massium('./plots/vs_emap_massium.png')
simres.draw_score('./plots/vs_emap_score.png')
simres.draw_resources_assembly('./plots/vs_emap_res_assemble.png')
simres.draw_resources_gatherer('./plots/vs_emap_res_gather.png')
simres.compute_jobs()
simres.switch_AB()
simres.draw_massium('./plots/vs_random_massium.png')
simres.draw_score('./plots/vs_random_score.png')
simres.draw_resources_assembly('./plots/vs_random_res_assemble.png')
simres.draw_resources_gatherer('./plots/vs_random_res_gather.png')
simres.compute_jobs()
