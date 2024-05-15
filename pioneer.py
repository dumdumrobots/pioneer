import torch
import torch.nn as nn
import torch.nn.functional as F

class Pioneer(nn.Module):
    def __init__(self):
        super(Pioneer, self).__init__()
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3)
        self.conv2 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3)
        
        # Fully connected layers
        self.fc1 = nn.Linear(in_features=64*5*5, out_features=128)
        self.fc2 = nn.Linear(in_features=128, out_features=10)

    def forward(self, x):
        # Convolutional layers with max pooling
        x = F.relu(F.max_pool2d(self.conv1(x), kernel_size=2))
        x = F.relu(F.max_pool2d(self.conv2(x), kernel_size=2))
        
        # Flatten the output for fully connected layers
        x = x.view(-1, 64*5*5)
        
        # Fully connected layers with ReLU activation
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        
        return F.log_softmax(x, dim=1)
    
class PioneerTrainer:
    def __init__(self, model, train_loader, eval_loader, criterion, optimizer, checkpoint_path):
        self.model = model
        self.train_loader = train_loader
        self.eval_loader = eval_loader
        self.criterion = criterion
        self.optimizer = optimizer
        self.checkpoint_path = checkpoint_path

    def train(self, epochs=5):
        print("Training started...")
        for epoch in range(1, epochs + 1):
            running_loss = 0.0
            for i, (inputs, labels) in enumerate(self.train_loader, 1):
                self.optimizer.zero_grad()
                outputs = self.model(inputs)
                loss = self.criterion(outputs, labels)
                loss.backward()
                self.optimizer.step()
                running_loss += loss.item()

                if i % 100 == 0:
                    print(f"Epoch [{epoch}/{epochs}], Step [{i}/{len(self.train_loader)}], Loss: {running_loss / i:.4f}")

            print(f"Epoch [{epoch}/{epochs}], Loss: {running_loss / len(self.train_loader):.4f}")

            # Save checkpoint
            self.save_checkpoint(epoch)

        print("Training finished.")

    def evaluate(self):
        print("Evaluation started...")
        correct = 0
        total = 0
        with torch.no_grad():
            for images, labels in self.eval_loader:
                outputs = self.model(images)
                _, predicted = torch.max(outputs.data, 1)
                total += labels.size(0)
                correct += (predicted == labels).sum().item()

        accuracy = 100 * correct / total
        print(f"Accuracy of the network on the {total} test images: {accuracy:.2f}%")
        print("Evaluation finished.")

    def save_checkpoint(self, epoch):
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'loss': self.criterion
        }
        torch.save(checkpoint, self.checkpoint_path)
        print("Checkpoint saved.")