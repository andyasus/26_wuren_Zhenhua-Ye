import torch
from torch import nn
from torchvision import transforms,datasets
from torch.utils.data.dataloader import DataLoader
import torch.optim as optim
import torch.nn.functional as F
from torchinfo import summary
import os

class mixed_net(nn.Module):
    def __init__(self):
        super(mixed_net,self).__init__()

    
    def forward(self, x):
        '''
        公式： W = (W + 2padding - kernel_w) / stride + 1

        '''

        return x

if __name__ == "__main__":
    #图像转换
    transforms = transforms.Compose(
        [
            transforms.Resize([64, 64]),
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ]
    )
    
    #超参数设置
    BATCH_SIZE = 1024
    EPOCH = 200

    #加载数据
    trainset = datasets.ImageFolder(root=r'dataset/train',transform=transforms)
    testset1 = datasets.ImageFolder(root=r'dataset/test1',transform=transforms)
    testset2 = datasets.ImageFolder(root=r'dataset/test2',transform=transforms)

    print(f"训练集图片数量: {len(trainset)}")
    print(f"测试集1图片数量: {len(testset1)}")
    print(f"测试集2图片数量: {len(testset2)}")
    
    train_loader = DataLoader(trainset, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
    test_loader1 = DataLoader(testset1, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
    test_loader2 = DataLoader(testset2, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)

    #创建网络
    device = torch.device("mps" if torch.cuda.is_available() else "cpu")
    net = mixed_net().to(device)
    
    #打印网络信息
    summary(net, input_size=(1, 3, 64, 64), device=device)
    print(f'标签对应的ID: {trainset.class_to_idx}')

    #设置优化器、损失函数
    criterion = nn.CrossEntropyLoss()
    optimizer =optim.SGD(net.parameters(), lr=0.01, momentum=0.9)
    # optimizer = optim.Adam(net.parameters(), lr=0.001, weight_decay=1e-4)

    #开始训练

    print("Start")
    for epoch in range(EPOCH):
        train_loss = 0.0
        #print(epoch)
        
        for batch_id, (datas, labels) in enumerate(train_loader):
            datas, labels = datas.to(device), labels.to(device)

            optimizer.zero_grad()

            outputs = net(datas)

            loss = criterion(outputs, labels)

            loss.backward()

            optimizer.step()

            train_loss += loss.item()

            if epoch > 50 and (epoch + 1) % 10 == 0:
                os.makedirs("pth", exist_ok=True)
                PATH = "pth/modeltemp.pth"
                torch.save(net.state_dict(), PATH)
                model = mixed_net()
                model.load_state_dict(torch.load(PATH))
                model.eval()
                model.to(device)

                #限定保存条件
                max_correct = 99
                correct1 = 0
                correct2 = 0
                total1 = 0
                total2 = 0

                #分别测试两个数据集
                with torch.no_grad():
                    for i ,(datas1, labels1) in enumerate(test_loader1):
                        datas1, labels1 = datas1.to(device), labels1.to(device)
                        output_test1 = model(datas1)
                        _, predicted1 = torch.max(output_test1.data, dim=1)
                        total1 += predicted1.size(0)
                        correct1 += (predicted1 == labels1).sum()

                    for i ,(datas2, labels2) in enumerate(test_loader2):
                        datas2, labels2 = datas2.to(device), labels2.to(device)
                        output_test2 = model(datas2)
                        _, predicted2 = torch.max(output_test2.data, dim=1)
                        total2 += predicted2.size(0)
                        correct2 += (predicted2 == labels2).sum()

                    #打印消息
                    c1 = 0
                    c2 = 0
                    c2 = correct2 / total2 * 100
                    c1 = correct1 / total1 * 100
                    print(
                        f"epoch:{epoch + 1}\tbatch_id:{batch_id + 1}\taverage_loss:{(train_loss / len(train_loader.dataset)):.5f}\t"
                        f"correct1:{c1:.2f}%\tcorrect2:{c2:.2f}%"
                    )
                    if (c1 > max_correct):
                        max_correct = c1
                        MAX_PATH = f"pth/model_best_{max_correct}.pth"
                        print(f"save {MAX_PATH}")
                        torch.save(net.state_dict(),MAX_PATH)

