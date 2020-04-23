import cv2
from matplotlib import pyplot as plt
import numpy as np
import time
import torch
import torch.nn as nn
import torch.nn.functional as F

def load_dataset(train_percentage=0.8):
    IMG_SIZE = 28
    TOTAL_IMGS = 1242
    X = np.zeros((TOTAL_IMGS, 1, IMG_SIZE, IMG_SIZE))
    y = np.zeros((TOTAL_IMGS, 3))
    for k in range(TOTAL_IMGS):
        
        img = cv2.imread("./ds/image{}.png".format(k), cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, dsize=(IMG_SIZE, IMG_SIZE))/256.
        X[k, :, :, :] = img
        X[k, :, 0:4, 0:32] = 1.
        
    f = open('./ds/data.txt')
    content = f.read()
    y=np.array([np.array([float(v.strip()) for v in values.split(',')]) for values in content.split('\n')])
    
    final_idx = int(TOTAL_IMGS * train_percentage)
    
    X_train = X[0:final_idx]
    y_train = y[0:final_idx]
    
    X_test = X[final_idx:]
    y_test = y[final_idx:]
    
    return (X_train, y_train), (X_test, y_test)

# The neural networks class
class CNN(nn.Module):

    # define our network structure
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 6 , 5)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(16*16, 120)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 3)

    # define one forward pass through the network
    def forward(self, x):
        x = F.max_pool2d(F.relu(self.conv1(x)), (2, 2))
        x = F.max_pool2d(F.relu(self.conv2(x)), (2, 2))
        x = x.view(-1, self.num_flat_features(x))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
    
    # Thelper function to understand the dimensions
    def num_flat_features(self, x):
        size = x.size()[1:] # all dimensions except the batch dimension
        return np.prod(size)

# T: number of epochs
# B: minibatch size, 
# gamma: step size,
# rho: momentum.
device = 'cuda' if torch.cuda.is_available() else 'cpu'
errors = []
errors_test = []
def backprop_deep(xtrain, ltrain, xtest, ltest, net, T, B=64, gamma=.00001, rho=.9):
    N = xtrain.size()[0]        # Training set size
    NB = N//B   # Number of minibatches #!Complete
    criterion = nn.MSELoss()
    optimizer = torch.optim.SGD(net.parameters(), lr=gamma, momentum=rho) #!Complete 
    for epoch in range(T):
        running_loss = 0.0
        shuffled_indices = np.random.permutation(range(N))
        for k in range(NB):
            # Extract k-th minibatch from xtrain and ltrain
            minibatch_indices = shuffled_indices[B*k:min(B*(k+1), N)]
            #print(minibatch_indices)
            inputs = xtrain[minibatch_indices].type(torch.FloatTensor).to(device) #!Complete
            labels = ltrain[minibatch_indices].type(torch.FloatTensor).to(device)#!Complete

            # Initialize the gradients to zero
            optimizer.zero_grad()

            # Forward propogation
            outputs = net.forward(inputs) 

            # Error evaluation
            loss = criterion(outputs, labels)

            # Back propogation
            loss.backward()

            # Optimize step
            optimizer.step()

            # Compute and print statistics
            with torch.no_grad():
                running_loss += loss
                
            errors.append(loss.to('cpu'))
            
            with torch.no_grad():
                yinit = net.forward(xtest)
                
            errors_test.append(F.mse_loss(yinit, ltest).to('cpu'))

            if k == 0 and epoch % 20 == 0:
                print('[%d] loss: %.3f' %(epoch + 1, running_loss / 100))
                
            #running_loss = 0.0

(X_train, y_train), (X_test, y_test) = load_dataset()

xtrain = torch.from_numpy(X_train)
ltrain = torch.from_numpy(y_train)
xtest = torch.from_numpy(X_test).type(torch.FloatTensor)
ltest = torch.from_numpy(y_test).type(torch.FloatTensor)

net = CNN().to(device)
xtrain = xtrain.to(device)
ltrain = ltrain.to(device)
xtest = xtest.to(device)
ltest = ltest.to(device)
t = time.time()
backprop_deep(xtrain, ltrain, xtest, ltest, net, T=300)
print(time.time() - t)

training_loss = np.array(errors)
testing_loss = np.array(errors_test)

import matplotlib.pyplot as plt
plt.plot(training_loss)
plt.plot(testing_loss)
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['training loss', 'testing loss'])
plt.show()