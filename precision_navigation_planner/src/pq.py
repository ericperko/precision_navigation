#Priority Queue yoinked right from the heapq docs

import itertools
import heapq

class PriorityQueue:
    def __init__(self):
        self.pq = []                         # the priority queue list
        self.counter = itertools.count(1)    # unique sequence count
        self.state_finder = {}                # mapping of states to entries
        self.INVALID = 0                     # mark an entry as deleted

    def contains_state(self, state):
        return state in self.state_finder

    def empty(self):
        return (len(self.state_finder) == 0)

    def add_state(self, priority, state, count=None):
        if count is None:
            count = next(self.counter)
        entry = [priority, count, state]
        self.state_finder[state] = entry
        heapq.heappush(self.pq, entry)

    def get_top_priority(self):
        while True:
            priority, count, state = heapq.heappop(self.pq)
            del self.state_finder[state]
            if count is not self.INVALID:
                return state

    def delete_state(self,state):
        entry = self.state_finder[state]
        entry[1] = self.INVALID

    def reprioritize(self,priority, state):
        entry = self.state_finder[state]
        self.add_state(priority, state, entry[1])
        entry[1] = self.INVALID
