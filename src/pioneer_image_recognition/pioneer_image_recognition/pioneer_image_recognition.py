import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
# from geometry_msgs.msg import Pose

from cv_bridge import CvBridge

import cv2
import torch
import torchvision.transforms as transforms
<<<<<<< HEAD
=======

>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)
import torch.nn as nn
import torch.nn.functional as F


class Number:
    def __init__(self, number, size, location):
        self.number = number
        self.size = size
        # self.location = location


class Colour:
    def __init__(self, colour, size, location):
        self.colour = colour
        self.size = size
        self.location = location
<<<<<<< HEAD
        
=======

>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)

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
<<<<<<< HEAD
 
=======


>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)

class DigitRecogniser:
    def __init__(self, model_path='pioneer_checkpoint_v2.pth'):
        self.model = self.load_model(model_path)

    def load_model(self, model_path):
        # Load the checkpoint
        checkpoint = torch.load(model_path)

        # Initialize the model and load its state
        model = Pioneer()
        model.load_state_dict(checkpoint['model_state_dict'])

        # Set the model to evaluation mode
        model.eval()
        return model

    def preprocess_image(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Resize and invert the image
        img_resized = cv2.resize(gray, (28, 28), interpolation=cv2.INTER_AREA)
        img_resized = cv2.bitwise_not(img_resized)

        # Convert to tensor and normalize
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,))
        ])
        img_tensor = transform(img_resized)
        return img_tensor

    def predict_digit(self, img_tensor):
        with torch.no_grad():
            outputs = self.model(img_tensor.unsqueeze(0))  
            probabilities = torch.nn.functional.softmax(outputs, dim=1)
            confidence, predicted = torch.max(probabilities, 1)
        return int(predicted[0]), float(confidence * 100)

    def recognise_digit(self, image):
        digit_ROIs = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive thresholding to handle varying lighting conditions
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)
        
        # Perform morphological operations to enhance the digit contours
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
<<<<<<< HEAD
            if area > 1000: 
=======
            if area > 5000: 
>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Expand the bounding box
                x -= 40  # Increase expansion in x-direction
                y -= 40  # Increase expansion in y-direction
                w += 80  # Increase expansion in width
                h += 80  # Increase expansion in height
                
                # Ensure the bounding box stays within the frame boundaries
                x = max(x, 0)
                y = max(y, 0)
                w = min(w, image.shape[1] - x)
                h = min(h, image.shape[0] - y)
                
                # Check if the contour is on a white A4 paper
                mean_color = cv2.mean(image[y:y+h, x:x+w])
<<<<<<< HEAD
                if mean_color[0] > 160 and mean_color[1] > 160 and mean_color[2] > 160:
=======
                if mean_color[0] > 150 and mean_color[1] > 150 and mean_color[2] > 150:
>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)
                    digit_ROIs.append((x, y, w, h))
        return digit_ROIs

    def process_frame(self, frame):
        digits = []
        digit_ROIs = self.recognise_digit(frame)

        for x, y, w, h in digit_ROIs:
            ROI = frame[y:y+h, x:x+w]
            predicted_digit, confidence = self.predict_digit(self.preprocess_image(ROI))
<<<<<<< HEAD
            if confidence > MIN_CONFIDENCE:
=======
            if confidence > 80:
>>>>>>> 40ac2264 (Added venv with pre installed packages for image processing)
                digits.append((predicted_digit, w * h))
                
        return digits


class Pioneer_Image_Recognition(Node):

    def __init__(self):
        # self.pose = Pose()
        super().__init__('pioneer_image_recognition')
        self.subscription = self.create_subscription(Image, '/oak/stereo/image_raw', self.image_cb, 10)
        # self.subscription = self.create_subscription(Pose, '/odom', self.pose_cb, 10)
        self.publisher_ = self.create_publisher(Number, '/number_recog', 10)
        # self.publisher_ = self.create_publisher(Colour, '/colour_recog', 10)


    def image_cb(self, msg):
        self.image = msg.data

        digit_recogniser = DigitRecogniser()
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')

        digits = digit_recogniser.process_frame(cv_image)

        for digit in digits:
            number = Number(digit[0], digit[1]) # Number, Size, Pose
            self.publisher_.publish(number)


    # def pose_cb(self, msg):
    #     self.pose = msg.position


def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = Pioneer_Image_Recognition()

    # Spin the node to receive messages and call the joy_callback function for each message.

    rclpy.spin(node)
    
    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()