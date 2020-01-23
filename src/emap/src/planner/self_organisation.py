from .help_functions import remove_multiple
import numpy as np


class TimeStampBuffer:
    
    def __init__(self, buffer_len=3000):
        self._buffer = []
        self._buffer_len = buffer_len
        
    def add(self, message):
        for i, m in enumerate(self._buffer):
            if m.timestamp > message.timestamp:
                self._buffer.insert(i, message)
                return
        self._buffer.append(message)
        if len(self._buffer) > self._buffer_len:
            diff = len(self._buffer) - self._buffer_len
            self._buffer = self._buffer[diff:]
            

class SOJob:
    
    def __init__(self, my_name):
        self._my_name = my_name
        self._jobs = TimeStampBuffer(buffer_len=1000)
        
    def _update_buffer(self, message):
        self._jobs.add(message)
        
    def remove_job(self, job_id):
        remove = []
        #print('actiually removing job ' +job_id)
        for i,job_application in enumerate(self._jobs._buffer):
            if job_application.job_id == job_id: remove.append(i)
        remove_multiple(self._jobs._buffer, remove)
        
    def is_my_job(self, job_id):
        for job_application in self._jobs._buffer:
            if job_application.job_id == job_id:
                if job_application.agent_name == self._my_name: 
                    return True
                else:
                    return False
        return False
    
    def get_jobs_already_taken(self):
        already_taken = []
        for job_application in self._jobs._buffer:
            if job_application.job_id not in already_taken: already_taken.append(job_application.job_id)
        return already_taken
    
    def get_agent_order(self):
        order = []
        for job_application in self._jobs._buffer:
            if job_application.agent_name not in order: order.append(job_application.agent_name)
        #print(order)
        return order
    
    def get_nb_jobs_before_mine(self):
        already_taken =  self.get_jobs_already_taken()
        nb_jobs_before_mine = 0
        for job_id in already_taken:
            if not self.is_my_job(job_id): nb_jobs_before_mine +=1
            else: break
        #print(self._agent_knowledge.my_name + ' has ' +str(nb_jobs_before_mine) + ' jobs before it')
        return nb_jobs_before_mine
    
    
class SOResource:
    
    def __init__(self, knowledge):
        self._my_name = knowledge. my_name
        self._knowledge = knowledge
        self._resources = TimeStampBuffer()
    
    def _update_buffer(self, message):
        if message.block_release:
            self._resources.add(message)
        else:
            self._remove_block_from_buffer(message.agent_name, message.storage_name, message.item_name, message.item_amount)
        
    def _remove_block_from_buffer(self, agent_name, storage_name, item_name, item_amount):
        for i,resource_block in enumerate(self._resources._buffer):
            if (resource_block.storage_name == storage_name) and (resource_block.item_name == item_name) and \
            (resource_block.item_amount == item_amount) and (resource_block.agent_name == agent_name):
                del self._resources._buffer[i]
                break
            
    def is_my_resource(self, storage_name, item_name, item_amount):
        already_blocked_amount = 0
        block_found = False
        for i,resource_block in enumerate(self._resources._buffer):
            #print(resource_block.storage_name,  resource_block.item_name, resource_block.item_amount, resource_block.agent_name)
            if (resource_block.storage_name == storage_name) and (resource_block.item_name == item_name) and \
            (resource_block.item_amount == item_amount) and resource_block.agent_name == self._my_name: 
                block_found = True
                break
            if (resource_block.storage_name == storage_name) and (resource_block.item_name == item_name):
                already_blocked_amount += resource_block.item_amount   
        if not block_found: return False
        for item in self._knowledge.storages[storage_name].items:
            if item.name == item_name: break
        #print(item_name, item.stored, already_blocked_amount, item_amount, (item.stored > (already_blocked_amount + item_amount)))
        return (item.stored >= (already_blocked_amount + item_amount))
    
    
class SOMeeting:
    
    def __init__(self, my_name, my_role):
        self._my_name = my_name
        self._my_role = my_role
        self._requests = TimeStampBuffer()
        self._replies = TimeStampBuffer()
     
    def _update_buffer_remove(self, message):
        for i in range(len(self._requests._buffer)-1,-1,-1):
            if self._requests._buffer[i].id == message.related_id: del self._requests._buffer[i]
        for i in range(len(self._replies._buffer)-1,-1,-1):
            if self._replies._buffer[i].related_id == message.related_id: del self._replies._buffer[i]
            
    def _update_buffer_request(self, message):
        self._requests.add(message)
        
    def _update_buffer_reply(self, message):
        self._replies.add(message)
        
    def _get_all_my_reply_ids(self):
        return {reply.related_id for reply in self._replies._buffer if reply.agent_name == self._my_name}
    
    def _get_all_requests_with_replies_ids(self):
        return {reply.related_id for reply in self._replies._buffer}
    
    def _get_all_requests_for_me_ids(self):
        return {request.id for request in self._requests._buffer if ((request.agent_name != self._my_name) and (request.role == self._my_role))}
    
    def _get_all_reqlies_for_id(self, related_id):
        return [reply for reply in self._replies._buffer if reply.related_id == related_id]
    
    def _get_request(self, related_id):
        for request in self._requests._buffer:
            if request.id == related_id: return request
    
    def _get_best_reply_for_id(self, related_id):
        related_request = self._get_request(related_id)
        replies = self._get_all_reqlies_for_id(related_id)
        if not replies: return None
        waiting_times = [abs(related_request.step - reply.arrival_step) for reply in replies]
        best_id = np.argmin(waiting_times)
        return replies[best_id]
    
    def get_open_meeting_requests(self):
        #print(self._my_name, self._get_all_requests_for_me_ids(), self._get_all_my_reply_ids())
        open_ids = self._get_all_requests_for_me_ids() - self._get_all_requests_with_replies_ids()
        requests = []
        for request in self._requests._buffer:
            if request.id in open_ids: requests.append(request)
        return requests
    
    def get_my_meetings(self):
        my_meetings = []
        for request in self._requests._buffer:
            best_reply = self._get_best_reply_for_id(request.id)
            if (best_reply is not None) and (best_reply.agent_name == self._my_name): my_meetings.append(request)
        return my_meetings
