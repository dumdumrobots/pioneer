import torch
import torch.nn as nn
import torch.optim as optim
from torchvision.datasets import SVHN
import torchvision.transforms as transforms
from torch.utils.data import DataLoader

from pioneer import Pioneer

# Load your pre-trained Pioneer model
model_path = 'pioneer_checkpoint_v2.pth'
model = Pioneer()
checkpoint = torch.load(model_path)
model.load_state_dict(checkpoint['model_state_dict'])

# Replace the classification head for SVHN
model.fc2 = nn.Linear(in_features=128, out_features=10)  # Adjust output features for SVHN

# Load SVHN dataset
transform = transforms.Compose([
    transforms.Grayscale(),  # Convert to grayscale
    transforms.Resize((28, 28)),  # Resize to match Pioneer input size
    transforms.ToTensor(),  # Convert to tensor
    transforms.Normalize((0.5,), (0.5,))  # Normalize
])

train_dataset = SVHN(root='svhn_data/', split='train', download=True, transform=transform)
test_dataset = SVHN(root='svhn_data/', split='test', download=True, transform=transform)

train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
test_loader = DataLoader(test_dataset, batch_size=64, shuffle=False)

# Define optimizer and loss function
optimizer = optim.Adam(model.parameters(), lr=0.001)
criterion = nn.CrossEntropyLoss()

# Fine-tuning
model.train()

for epoch in range(10):
    for inputs, labels in train_loader:
        inputs, labels = inputs, labels
        
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

# Evaluation
model.eval()
total_correct = 0
total_samples = 0
with torch.no_grad():
    for inputs, labels in test_loader:
        inputs, labels = inputs, labels
        outputs = model(inputs)
        _, predicted = torch.max(outputs, 1)
        total_samples += labels.size(0)
        total_correct += (predicted == labels).sum().item()

accuracy = total_correct / total_samples
print(f'Accuracy on SVHN test set: {accuracy * 100:.2f}%')

torch.save(model.state_dict(), 'fine_tuned_pioneer_svhn.pth')
