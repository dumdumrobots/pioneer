import cv2
import torch
import torchvision.transforms as transforms
from matplotlib import pyplot as plt
from pioneer import Pioneer  

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

    def preprocess_image(self, image_path):
        # Read the image in grayscale
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        # Resize and invert the image
        img_resized = cv2.resize(image, (28, 28), interpolation=cv2.INTER_AREA)
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

    def visualise_image(self, image_path):
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        plt.imshow(image, cmap='gray')
        plt.show()
        
    def visualise_preprocessed_image(self, image_path):
        img_tensor = self.preprocess_image(image_path)
        img = img_tensor.squeeze().numpy()
        plt.imshow(img, cmap='gray')
        plt.show()

    def recognise_digit(self, image_path):
        img_tensor = self.preprocess_image(image_path)
        predicted_digit, confidence = self.predict_digit(img_tensor)
        return predicted_digit, confidence

if __name__ == "__main__":
    digit_recogniser = DigitRecogniser()
    image_path = r'/Users/nathan/Desktop/digit_recognition/images/close/6_closer.JPG'
    predicted_digit, confidence = digit_recogniser.recognise_digit(image_path)
    print(f"Predicted digit: {predicted_digit}, Confidence: {confidence:.2f}%")
