import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import os

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.relu = nn.ReLU()
        
        self.flatten = 128 * 16 * 16
        self.fc1 = nn.Linear(self.flatten, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 3)

    def forward(self, x):
        x = self.pool(self.relu(self.conv1(x)))
        x = self.pool(self.relu(self.conv2(x)))
        x = self.pool(self.relu(self.conv3(x)))
        x = x.view(x.size(0), -1)
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# 设置设备
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# 图片处理（和训练时保持一致：128x128）
IMG_SIZE = 128
test_transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# 加载两个测试集
test1_dataset = datasets.ImageFolder(root='dataset/test1', transform=test_transform)
test2_dataset = datasets.ImageFolder(root='dataset/test2', transform=test_transform)

test1_loader = DataLoader(test1_dataset, batch_size=32, shuffle=False)
test2_loader = DataLoader(test2_dataset, batch_size=32, shuffle=False)

# 加载训练的模型
model = Net().to(DEVICE)
model.load_state_dict(torch.load('best_model.pth'))
model.eval()

# 验证函数
def validate(loader, name):
    correct = 0
    total = 0
    class_correct = [0, 0, 0]
    class_total = [0, 0, 0]
    class_names = ['blue', 'red', 'yellow']
    
    with torch.no_grad():
        for imgs, labels in loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            outputs = model(imgs)
            _, predicted = torch.max(outputs, 1)
            
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
            
            # 统计每一类
            for i in range(labels.size(0)):
                label = labels[i].item()
                class_total[label] += 1
                if predicted[i].item() == label:
                    class_correct[label] += 1
    
    print(f"\n=== {name} 测试集验证结果 ===")
    print(f"整体准确率: {100 * correct / total:.2f}%")
    for i in range(3):
        if class_total[i] > 0:
            acc = 100 * class_correct[i] / class_total[i]
            print(f"{class_names[i]} 类准确率: {acc:.2f}% (样本数: {class_total[i]})")

# 开始验证
validate(test1_loader, "Test1")
validate(test2_loader, "Test2")
print("\n验证完成！")