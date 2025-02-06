import os
import numpy as np
import queue
def list_all_files(data_dir, ext):
    return [f"{data_dir}/{f}" for f in os.listdir(data_dir) if f.endswith(ext)]

def list_all_npz_files(data_dir):
    return list_all_files(data_dir, ".npz")

def load_state(filename):
    if not os.path.exists(filename):
        return None
    state = np.load(filename)
    return state

def save_state(state, filename):
    # save dict state
    np.savez(filename, **state)


class MyStates:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.action_queue = queue.Queue()
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        self.sync_dir()

    def sync_dir(self):
        self.npz_files = list_all_npz_files(self.data_dir)
        print("sync_dir:", self.npz_files)
        self.order_list = []
        if len(self.npz_files) > 0:
            for f in self.npz_files:
                base_name = os.path.basename(f)
                order = int(base_name.split(".")[0])
                self.order_list.append(order)
            self.order_list.sort()
            self.next_order = self.order_list[-1] + 1
        else:
            self.next_order = 0

    def save_state(self, state):
        order = self.next_order
        filename = os.path.join(self.data_dir, f"{order}.npz")
        save_state(state, filename)
        print("saving state to", filename)
        self.order_list.append(order)
        self.next_order += 1
        return filename
    
    
    def read_by_order(self, order):
        filename = os.path.join(self.data_dir, f"{order}.npz")
        return load_state(filename)

    def find_next_order(self, i, loop=False):
        for order in self.order_list:
            if order > i:
                return order
        if loop:
            return self.order_list[0]
        return None
    
    def empty(self):
        return len(self.order_list) == 0
    
    def sync_to_action_queue(self):
        for i in self.order_list:
            self.action_queue.put(i)
        return self.action_queue
    
    def popfront(self):
        if self.action_queue.empty():
            return None
        order =  self.action_queue.get()
        print(f"popfront: {order}")
        return self.read_by_order(order)
    
if __name__ == "__main__":
    home_dir = "./data_states/test1"
    states = MyStates(home_dir)
    
    # for i in range(5):
    #     state = dict(a=i, b=i*2)
    #     states.pushback(state)

    # states.sync_dir()
    order = 0

    action_queue = states.sync_to_action_queue()
    # print the queue
    while not action_queue.empty():
        state = states.popfront()
        print(f"order {order}:")
        print(state['a'])
        print(state['b'])

    # for i in range(10):
    #     order = states.find_next_order(order, loop=True)#order_list:
    #     state = states.read_by_order(order)
    #     print(f"order {order}:")
    #     print(state['a'])
    #     print(state['b'])
    #     action_queue.put(state)
    
