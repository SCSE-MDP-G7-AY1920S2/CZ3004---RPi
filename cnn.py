import torch
import torch.nn as nn
import torch.nn.functional as F

class CNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool2d(2)

        self.conv2 = nn.Conv2d(6, 16,5)
        self.relu2 = nn.ReLU()
        self.pool2 = nn.MaxPool2d(2)

        self.fcnl1 = nn.Linear(400, 32)
        self.relu3 = nn.ReLU()
        
        self.fcnl2 = nn.Linear(32, 15)
        self.relu4 = nn.ReLU()

        self.fcnl3 = nn.Linear(15, 15)
        self.relu5 = nn.ReLU()
        #self.soft1 = nn.Softmax()
    def forward(self, img): 
        img = self.conv1(img)
        img = self.relu1(img)
        img = self.pool1(img)

        img = self.conv2(img)
        img = self.relu2(img)
        img = self.pool2(img)

        img = img.view(-1, 400)

        img = self.fcnl1(img)
        img = self.relu3(img)

        img = self.fcnl2(img)
        img = self.relu4(img)

        img = self.fcnl3(img)
        img = self.relu5(img)
        
        img = F.softmax(img, dim=1)
        return img
    
    def loadModel(self, model_path):
        self.load_state_dict(torch.load(model_path))

    def saveModel(self, model_path):
        torch.save(self.state_dict(), model_path)

