

class CircularQueue:
    def __init__(self, capacity):
        self.capacity = capacity
        self.queue = [None] * self.capacity
        self.head = 0
        self.tail = 0
        self.size = 0

    def enqueue(self, item):
        self.queue[self.tail] = item
        self.tail = (self.tail + 1) % self.capacity
        self.size += 1

    def dequeue(self):
        if self.is_empty():
            return None
        item = self.queue[self.head]
        for i in range(self.capacity):
            self.queue[self.head] = None
            self.head = (self.head + 1) % self.capacity
            i += 1
            self.size -= 1
        return item

    def is_empty(self):
        return self.size == 0

    def is_full(self):
        return self.size == self.capacity

    def get_size(self):
        return self.size
    
    def print_data(self):
        return self.queue


