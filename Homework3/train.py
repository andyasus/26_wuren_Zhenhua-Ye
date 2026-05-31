import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
import os

# 基础设置
BATCH_SIZE = 16
EPOCHS = 15
LR = 0.001
NUM_CLASSES = 3
DATA_PATH = './dataset'
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

print(f"用到的是: {DEVICE}")

# 数据增强和转换
img_size = 128
train_transform = transforms.Compose([
    transforms.Resize((img_size, img_size)),
    transforms.RandomHorizontalFlip(),  # 翻转一下，增加样本
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])
test_transform = transforms.Compose([
    transforms.Resize((img_size, img_size)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# 加载数据
train_data = datasets.ImageFolder(os.path.join(DATA_PATH, 'train'), train_transform)
test1_data = datasets.ImageFolder(os.path.join(DATA_PATH, 'test1'), test_transform)
test2_data = datasets.ImageFolder(os.path.join(DATA_PATH, 'test2'), test_transform)

print("类别对应:", train_data.class_to_idx)  # 打印出来确认一下

train_loader = DataLoader(train_data, batch_size=BATCH_SIZE, shuffle=True)
test1_loader = DataLoader(test1_data, batch_size=BATCH_SIZE, shuffle=False)
test2_loader = DataLoader(test2_data, batch_size=BATCH_SIZE, shuffle=False)

# 网络定义
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        # 卷积层 (不算激活)
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.relu = nn.ReLU()
        
        # 全连接 (不超过3层)
        self.flatten = 128 * 16 * 16  # 128x16x16 = 32768
        self.fc1 = nn.Linear(self.flatten, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 3)  # 3个类别

    def forward(self, x):
        # 输入: (batch, 3, 128, 128)
        x = self.pool(self.relu(self.conv1(x)))   # -> (batch, 32, 64, 64)
        x = self.pool(self.relu(self.conv2(x)))   # -> (batch, 64, 32, 32)
        x = self.pool(self.relu(self.conv3(x)))   # -> (batch, 128, 16, 16)
        x = x.view(x.size(0), -1)                 # -> (batch, 128*16*16=32768)
        x = self.relu(self.fc1(x))                # -> (batch, 256)
        x = self.relu(self.fc2(x))                # -> (batch, 128)
        x = self.fc3(x)                           # -> (batch, 3)
        return x

model = Net().to(DEVICE)
loss_fn = nn.CrossEntropyLoss()
opt = optim.Adam(model.parameters(), lr=LR)

# 训练
print("开始训练 ...")
for epoch in range(EPOCHS):
    model.train()
    total_loss = 0.0
    correct = 0
    total = 0
    
    for imgs, labels in train_loader:
        imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
        
        opt.zero_grad()
        outs = model(imgs)
        loss = loss_fn(outs, labels)
        loss.backward()
        opt.step()
        
        total_loss += loss.item()
        _, preds = torch.max(outs, 1)
        total += labels.size(0)
        correct += (preds == labels).sum().item()
    
    acc = 100 * correct / total
    print(f"Epoch {epoch+1}/{EPOCHS}  Loss: {total_loss/len(train_loader):.4f}  Acc: {acc:.2f}%")

torch.save(model.state_dict(), 'best_model.pth')
print("模型已经保存了~")

# 验证函数
def test_model(loader, name):
    model.eval()
    correct = 0
    total = 0
    class_correct = [0] * 3
    class_total = [0] * 3
    
    with torch.no_grad():
        for imgs, labels in loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            outs = model(imgs)
            _, preds = torch.max(outs, 1)
            total += labels.size(0)
            correct += (preds == labels).sum().item()
            
            # 统计每一类的
            for i in range(labels.size(0)):
                label = labels[i].item()
                class_total[label] += 1
                if preds[i].item() == label:
                    class_correct[label] += 1
                    
    overall = 100 * correct / total
    print(f"\n--- {name} 测试集 ---")
    print(f"总准确率: {overall:.2f}%")
    
    cls_names = ['blue', 'red', 'yellow']
    for i in range(3):
        if class_total[i] > 0:
            acc = 100 * class_correct[i] / class_total[i]
            print(f"{cls_names[i]}: {acc:.2f}% (样本数 {class_total[i]})")
    print("-" * 30)

# 验证
model.load_state_dict(torch.load('best_model.pth'))
test_model(test1_loader, "Test1")
test_model(test2_loader, "Test2")

print("拿下！")