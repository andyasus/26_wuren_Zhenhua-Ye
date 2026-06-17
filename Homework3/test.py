import torch
from torchvision import transforms,datasets
from torch.utils.data.dataloader import DataLoader
from net import mixed_net

def test_model(model_path, test_loader):
    #加载模型
    model = mixed_net()
    model.load_state_dict(torch.load(model_path))
    model.eval()  

    #初始化计数器
    class_correct = list(0. for i in range(3))
    class_total = list(0. for i in range(3))

    #开始测试
    with torch.no_grad():
        for i ,(datas, labels) in enumerate(test_loader):
            output_test = model(datas)
            _, predicted1 = torch.max(output_test.data, dim=1)

            matches = (predicted1 == labels)
            for i in range(len(labels)):
                label = labels[i]
                class_correct[label] += matches[i].item()
                class_total[label] += 1

    #计算总体正确率
    total_correct = sum(class_correct)
    total = sum(class_total)
    accuracy = 100.0 * total_correct / total
    print(f'Overall Accuracy: {accuracy:.2f}%')

    #输出每个类别的正确率
    for i in range(3):
        print(f'Accuracy of Class {i}: {100 * class_correct[i] / class_total[i]:.2f}%')
    

#图像转换
transforms = transforms.Compose([
    transforms.Resize([64, 64]),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

#加载数据
BATCH_SIZE = 1024
testset1 = datasets.ImageFolder(root=r'dataset/test1',transform=transforms)
testset2 = datasets.ImageFolder(root=r'dataset/test2',transform=transforms)
test_loader1 = DataLoader(testset1, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
test_loader2 = DataLoader(testset2, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)

#调用测试函数
model_path = r"pth/model_best_100.0.pth" 
test_model(model_path, test_loader1)
test_model(model_path, test_loader2)