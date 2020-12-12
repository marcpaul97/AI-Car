import torch
import torch.nn as nn
import torch.nn.functional as F
import random
import time
import torch.optim as optim
from torch.autograd import Variable
from servo_ultrasonic_avoid import Car
import csv
from multiprocessing import Process, SimpleQueue
import multiprocessing




    
    
class Net(nn.Module): #Creation of the NN
    def __init__(self,inputSize, actionSize):
        super(Net, self).__init__() #extending the neural network class
        self.inputSize = inputSize #number of inputs, for me it is 8
        self.actionSize = actionSize #number of possible actions
        self.fc = nn.Linear(inputSize, 30) #creating the connection between the input and the hidden layer
        self.fc2 = nn.Linear(30, actionSize) #creating the connection between the hidden layer to the output
        
    def forward(self, state): #Getting Q values from hidden layer to output layer and returning them
        hiddenLayer = F.relu(self.fc(state)) #creating weights between the input and the hidden layer
        qValues = self.fc2(hiddenLayer) #creating weights between the hidden layer and the output or the Q Values
        return qValues
    
class ReplayMemory(): #Creating memory and deciding on memory size
    def __init__(self, capacity, memory):
        self.capacity = capacity
        self.memory = memory 
        
    def push(self, event): #adding items into the memory 
        self.memory.append(event)
        if(len(self.memory) > self.capacity): # if memory is full we delete the last item in it
            del self.memory[0]
    
    def sample(self,batchSize): #getting a sample of our data
        self.batchSize = batchSize
        samples = zip(*random.sample(self.memory, batchSize)) #zipping it so we can save it has a tuple
        return map(lambda x: Variable(torch.cat(x, axis = 0,)), samples) #retuning the data as a map so we can access it easily
    
class Dqn(): #Deep Q Learning

    def __init__(self, inputSize, action, gamma, rewardWindow, lastState, memory, lastAction, lastScore, model, optimizer):
        self.gamma = gamma
        self.rewardWindow = rewardWindow
        self.model = model #model is our NN
        self.memory = memory #making our memory large --> allows more states to be added into memory
        self.optimzer = optimizer # setting the learning rate
        self.lastState = lastState
        self.lastState = torch.Tensor(inputSize).unsqueeze(0) #making our last state the last data from our input
        self.lastAction = lastAction 
        self.lastScore = lastScore
            
    def selectAction(self, state): #Soft max
        with torch.no_grad(): #disables the gradiant calulation so we are able to manipulate and use the data
            probs = F.softmax(self.model(state) , dim = 1) * 100 #getting the probabilty by using the softmax, and the model is the NN
            action = probs.multinomial(num_samples = 1) #we create a multinomial to find the "best" action --> largest number means best action
            return action.data[0,0]

        
    def doLearn(self, batchState, nextBatchState, batchReward, batchAction): #making the NN learn via backward propegation
        outputs = self.model(batchState).gather(1, batchAction.unsqueeze(1)).squeeze(1) #getting the outputs from our sample data
        nextOutputs = self.model(nextBatchState).detach().max(1)[0] #max in q values is at [0]
        target = self.gamma* nextOutputs + batchReward #calculating how far away we are from our goal
        td_loss = F.smooth_l1_loss(outputs, target) #calculating the difference between our goal and our actual result
        self.optimizer.zero_grad()
        td_loss.backward() 
        self.optimizer.step()
            
    def update(self, reward, newSignal, lastAction, lastState, lastScore, rewardWindow): #This updates the agent and provides a new State with an action that needs to be selected
        newState = torch.tensor(newSignal, requires_grad = True).float().unsqueeze(0) #getting our new state from the newSignal
        self.memory.push((lastState, newState, torch.LongTensor([int(lastAction)]), torch.Tensor([self.lastScore]))) #adding last action and last score to memory
        action = self.selectAction(newState) #new action is the new state after the action has been chosen
        if len(self.memory.memory) > 100: #looking at the memory as long as it's below our memory cap
            batch_state, batch_next_state, batch_action, batch_reward = self.memory.sample(100) 
            self.learn(batch_state, batch_next_state, batch_reward, batch_action)
        lastAction = action #setting our last action to current action
        lastState = newState #setting our last state to our current state
        lastScore = reward #setting our last score as our current score
        rewardWindow.append(reward) #adding reward to our reward window
        if len(rewardWindow) > 1000:
            del self.reward_window[0]
        return action 
    
    def score(self,rewardWindow):
        return sum(rewardWindow)
            
class main():  #main function
    def __init__(self, rewardWindow, lastState):
        self.rewardWindow = rewardWindow #a list of all the previous rewards
        self.lastState = lastState #the last state BeepBoop was in
        self.lastAction = 0 #the last action beepBoop took
        self.lastScore = 0 #the score from the last action
        self.memory = ReplayMemory(100, []) #our memory, containing the last 100 scores, states, and actions
        self.model = Net(8, 4) #intializing our NN with 8 input and 4 possible actions
        self.optimizer = optim.Adam(self.model.parameters(), lr = 1) #creates a learning rate, learning rate of 1 is very small
        self.deepQ = Dqn(8, 4, .9, self.rewardWindow, self.lastState, self.memory, self.lastAction, self.lastScore, self.model, self.optimizer) #NN
        self.beepBoop = Car(0) #intializing our car
        self.lastX = 0
        self.lastY = 0
        self.actionCount = 0
        self.totalScoreAdded = 0
        self.totalScores = []
        self.goalScore = 100 #our target goal
        self.angles = [0, 90, 180, -90] #our four angles for the motion sensor
        self.trackerStates = [False, False, False, False] #array of booleans for the tracker sensors
        self.possibleStates = [0, 0, 0, 0] #0 degrees is left, 90 degrees is straight, 180 is right, -90 is backwards

        
    def scanEast(self): #scans to see how far beepboop is to an object on the east side
        q = multiprocessing.SimpleQueue() #creates a simple queue which can be exchanged between processes
        p1 = Process(target = self.beepBoop.servo_appointed_detection(0,q), args = ()) #sensing for the left
        p1.start() #starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p1 = Process(target = self.beepBoop.Distance_test(q), args = ())
        p1.start() #starts the process on a separate process then the process which is running the program
        east = q.get()
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        return east    
            
    def scanNorth(self): #scans to see how far beepboop is to an object on the north side
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.servo_appointed_detection(67,q), args = ()) #sensing for straight forward
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p1 = Process(target = self.beepBoop.Distance_test(q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        north = q.get()
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        return north
    
    def scanWest(self): #scans to see how far beepboop is to an object on the west side
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.servo_appointed_detection(150,q), args = ()) #sensing for the right
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p1 = Process(target = self.beepBoop.Distance_test(q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        west = q.get()
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        return west
        
    def scanSouth(self): #scans to see how far beepboop is to an object on the south side
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.spin_right(72, 72,q), args = ()) 
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        p3 = Process(target = self.beepBoop.brake(q), args = ())
        p3.start()#starts the process on a separate process then the process which is running the program
        p3.join()#starts the process on a separate process then the process which is running the program
        p4 = Process(target = self.beepBoop.spin_right(70, 70, q), args = ())
        p4.start()#starts the process on a separate process then the process which is running the program
        p4.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p6 = Process(target = self.beepBoop.brake(q), args = ())
        p6.start()#starts the process on a separate process then the process which is running the program
        p6.join()#starts the process on a separate process then the process which is running the program
        p7 = Process(target = self.beepBoop.servo_appointed_detection(67,q), args = ())
        p7.start()#starts the process on a separate process then the process which is running the program
        p7.join()#starts the process on a separate process then the process which is running the program
        p8 = Process(target = self.beepBoop.Distance_test(q), args = ())
        p8.start()#starts the process on a separate process then the process which is running the program
        south = q.get()
        p8.join()
        time.sleep(.5)
        return south
    
    def moveBackFromSouth(self): #South is 180 degrees, to keep beepBoop accurate we turn another 180 degrees to face forward
        q = multiprocessing.SimpleQueue()
        p14 = Process(target = self.beepBoop.spin_right(69, 69,q), args = ()) 
        p14.start()#starts the process on a separate process then the process which is running the program
        p14.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p16 = Process(target = self.beepBoop.brake(q), args = ())
        p16.start()#starts the process on a separate process then the process which is running the program
        p16.join()#starts the process on a separate process then the process which is running the program
        
        
    def scan(self): #Does the four scans and puts into possible States array
        east = self.scanEast()
        print("East distance is " + str(east))
        time.sleep(1)
        
        north = self.scanNorth()
        print("North distance is " + str(north))
        time.sleep(1)
        
        west = self.scanWest()
        print("West distance is " + str(west)) 
        time.sleep(1)
              
        south = self.scanSouth()
        print("South distance is " + str(south))
        self.moveBackFromSouth()
        time.sleep(1)     
        
        self.possibleStates[0] = east #adding the four possible states into our array
        self.possibleStates[1] = north
        self.possibleStates[2] = west
        self.possibleStates[3] = south
        return
        
    def doTrack(self): #activates the tracker to tell if beepBoop is on a roacd or not, True = beepBoop is, false = beepBoop is not
        q = multiprocessing.Queue()
        p1 = Process(target = self.beepBoop.doTrack(self.trackerStates, q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        
    def moveSouth(self): #moves south and turns forward
        self.lastY = self.lastY - 1 #-1 since it moved backward
        self.actionCount = self.actionCount + 1
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.spin_right(72, 72,q), args = ()) 
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        p3 = Process(target = self.beepBoop.brake(q), args = ())
        p3.start()#starts the process on a separate process then the process which is running the program
        p3.join()#starts the process on a separate process then the process which is running the program
        p4 = Process(target = self.beepBoop.spin_right(70, 70, q), args = ())
        p4.start()#starts the process on a separate process then the process which is running the program
        p4.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p6 = Process(target = self.beepBoop.brake(q), args = ())
        p6.start()#starts the process on a separate process then the process which is running the program
        p6.join()#starts the process on a separate process then the process which is running the program
        p10 = Process(target = self.beepBoop.run(50, 50, q), args = ())
        p10.start()#starts the process on a separate process then the process which is running the program
        p10.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p13 = Process(target = self.beepBoop.brake(q), args = ())
        p13.start()#starts the process on a separate process then the process which is running the program
        p13.join()#starts the process on a separate process then the process which is running the program
        p14 = Process(target = self.beepBoop.spin_right(69, 69,q), args = ()) 
        p14.start()#starts the process on a separate process then the process which is running the program
        p14.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p16 = Process(target = self.beepBoop.brake(q), args = ())
        p16.start()#starts the process on a separate process then the process which is running the program
        p16.join()#starts the process on a separate process then the process which is running the program

        
        
    def moveEast(self): #moves east and turns forward
        self.lastX = self.lastX + 1 #+1 since it moved right
        self.actionCount = self.actionCount + 1
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.right(65, 65,q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p3 = Process(target = self.beepBoop.run(50, 50,q), args = ())
        p3.start()#starts the process on a separate process then the process which is running the program
        p3.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p5 = Process(target = self.beepBoop.brake(q), args = ())
        p5.start()#starts the process on a separate process then the process which is running the program
        p5.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p1 = Process(target = self.beepBoop.spin_left(50, 50,q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p7 = Process(target = self.beepBoop.brake(q), args = ())
        p7.start()#starts the process on a separate process then the process which is running the program
        p7.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        
    def moveNorth(self):
        q = multiprocessing.SimpleQueue()
        self.lastY = self.lastY + 1 #+1 since it moved forward
        self.actionCount = self.actionCount + 1
        p1 = Process(target = self.beepBoop.run(50, 50,q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p5 = Process(target = self.beepBoop.brake(q), args = ())
        p5.start()#starts the process on a separate process then the process which is running the program
        p5.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        
    def moveWest(self): #moves north already facing forward
        self.lastX = self.lastX - 1 #+1 since it moved left
        self.actionCount = self.actionCount + 1
        q = multiprocessing.SimpleQueue()
        p1 = Process(target = self.beepBoop.left(97, 97,q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p3 = Process(target = self.beepBoop.run(50, 50,q), args = ())
        p3.start()#starts the process on a separate process then the process which is running the program
        p3.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p5 = Process(target = self.beepBoop.brake(q), args = ())
        p5.start()#starts the process on a separate process then the process which is running the program
        p5.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p1 = Process(target = self.beepBoop.spin_right(43, 43,q), args = ())
        p1.start()#starts the process on a separate process then the process which is running the program
        p1.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        p7 = Process(target = self.beepBoop.brake(q), args = ())
        p7.start()#starts the process on a separate process then the process which is running the program
        p7.join()#starts the process on a separate process then the process which is running the program
        time.sleep(1)
        
        
    def move(self, direction): #the car actually moving once an action has been selected
        time.sleep(1)
        if(direction == 0):
            print("BeepBoop is moving East")
            self.moveEast()
            time.sleep(.25)
            
        elif(direction == 90):
            print("BeepBoop is moving North")
            self.moveNorth()
            time.sleep(.25)

        elif(direction == 180):
            print("BeepBoop is moving West")
            self.moveWest()
            time.sleep(.25)

        else:
            print("BeepBoop is moving South")
            self.moveSouth()
            time.sleep(.25)


            
            
    def lastSignalUpdater(self): #Converts the tracker boolean into 0's for False and 1's for true and adds them into the possible state array
        temp = [0, 0, 0, 0, 0, 0, 0, 0]
        i = 0
        while(i < 4):
            temp[i] = self.possibleStates[i]
            i = i + 1
        if(self.trackerStates[0] == True):
            temp[4] = 1
        if(self.trackerStates[0] == False):
            temp[4] = 0
        if(self.trackerStates[1] == True):
            temp[5] = 1
        if(self.trackerStates[1] == False):
            temp[5] = 0
        if(self.trackerStates[2] == True):
            temp[6] = 1
        if(self.trackerStates[2] == False):
            temp[6] = 0
        if(self.trackerStates[3] == True):
            temp[7] = 1
        if(self.trackerStates[3] == False):
            temp[7] = 0
        time.sleep(1)
        return temp
    
    
    def nnUpdater(self): #The action of updating the neural network
        q = multiprocessing.SimpleQueue()
        while(self.totalScoreAdded < 100):
            time.sleep(1)
            self.scan()
            self.doTrack()
            lastSignals = self.lastSignalUpdater() #list of all the possible next states
            self.lastScore = self.getReward() #last score
            time.sleep(1)
            action = self.deepQ.update(self.lastScore, lastSignals, self.lastAction, self.lastState, self.lastScore, self.rewardWindow) #picks the best next state
            self.totalScoreAdded = self.deepQ.score(self.rewardWindow) #adds the score that was just calculated
            print("The total score is " + str(self.totalScoreAdded))
            print((action.item()))
            foundAction = self.angles[action] #chooses between 0-3 for the next state
            #print("The found action is " + str(foundAction))
            self.move(foundAction) #the car moves to the next state
            time.sleep(.25)

    
    def getReward(self): #The "rules" for beepBoop
        for directions in self.possibleStates:
            if (directions <= 5): #if any of the inputs are less than 5 reward = -1
                self.lastScore = -1
                break
            else: #if they're all greater than 5 than reward = +1
                self.lastScore = 1
        for tracker in self.trackerStates:
            if(tracker == True):
                self.lastScore = self.lastScore + 0.25
                break
        if(self.trackerStates[0] == False and self.trackerStates[1] == False and self.trackerStates[2] == False and self.trackerStates[3] == False):
            self.lastScore = self.lastScore - 0.25
        return self.lastScore
                

    def sendToCSV(self): #sends data to csv file
        with open('data.txt', mode = 'w') as file:
            fieldnames = ["Memory Size", "Number of Actions","Rewards"]
            writeData = csv.writer(file, fieldnames = fieldnames) 
            writeData.writerow(len(self.nnUpdater.Dqn.self.memory), self.actionCount, self.rewardWindow)
     
     
if __name__ == '__main__':
    main([], 0).nnUpdater()
    
    
#obj = main([], 0)
#obj.nnUpdater()
#obj.sentToCSV()
        
        
        
         
            
            
            
            
            