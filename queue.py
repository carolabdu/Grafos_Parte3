class Queue():
    
    def __init__(self):
        self.items =[]
        
    def add(self, x): #adds the element x on the first position
        self.items.insert(0,x)
        
    def pop(self): # remove and returns the last item on the queue
        return self.items.pop()

    def is_empty(self): #checks if the queue is empty
        return len(self.items) == 0
    
    def size(self): #returns how many items are in the queue
        return len(self)
