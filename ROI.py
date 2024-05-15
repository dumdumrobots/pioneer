import cv2
import torch
import torchvision.transforms as transforms
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
            if area > 1000: 
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
                if mean_color[0] > 160 and mean_color[1] > 160 and mean_color[2] > 160:
                    digit_ROIs.append((x, y, w, h))
        return digit_ROIs

    def process_frame(self, frame):
        digit_ROIs = self.recognise_digit(frame)
        for x, y, w, h in digit_ROIs:
            ROI = frame[y:y+h, x:x+w]
            predicted_digit, confidence = self.predict_digit(self.preprocess_image(ROI))
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, f"Digit: {predicted_digit}, Confidence: {confidence:.2f}%", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        return frame

if __name__ == "__main__":
    digit_recogniser = DigitRecogniser()
    cap = cv2.VideoCapture(0)  

    while True:
        ret, frame = cap.read()  # Read frame from webcam
        if not ret:
            break

        processed_frame = digit_recogniser.process_frame(frame)
        cv2.imshow('Webcam', processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
