import os
import wandb
wandb.login()

from ultralytics import YOLO
import cv2


def load_model_scratch():
    # Create a new YOLO model from scratch
    scratch_model = YOLO('yolov8n.yaml')
    return scratch_model


def load_pretrained_model(name):
    # Load a pretrained YOLO model (recommended for training)
    print("Loading pretrained model...")
    pretrained_model = YOLO(name)
    return pretrained_model


def train_model(model):
    # Train the model using the dataset
    print("Training now...")
    results = model.train(data='source/data.yaml',
                          save_json = False, project='Project_Name',
                          epochs=2, val = True)
    
    # Evaluate the model's performance on the validation set
    print("Validating now...")
    results = model.val()
    return model


def validation_model(model):
    # Evaluate the model's performance on the validation set
    print("Validating now...")
    results = model.val(data='source/data.yaml',
                        imgsz = 640, batch = 32, iou = 0.1, split='val',save_txt=True, save = True)


def export_OpenVINO(model):
    # Export the model to ONXX and OpenVINO format
    print("Exporting OpenVINO format now...")
    model.export(format='openvino', half=False)


def load_OpenVINO_model(name):
    # Load the exported OpenVINO model
    print("Loading OpenVINO model...")
    ov_model = YOLO(name, task = 'detect')
    return ov_model


def single_detection(model, file):
    # Perform object detection on an image using the model
    print("Detecting now...")
    results = model(file, show = True, save = True, imgsz = 640)


def webcam_detection(model):
    print("Detecting now...")
    source = 0

    cap = cv2.VideoCapture(source)
    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 inference on the frame
            results = model(frame, iou = 0.1, verbose = True, show_labels = False, imgsz = 640)
            
            #for r in results:
                #print(r.speed)

            # Display the visualized results on the frame
            cv2.imshow("YOLOv8 Inference", results[0].plot())

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break

    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()



def main():
    # Model names:
    #'yolov8n.pt'
    #'best.pt' after training

    # Load our inicial model
    yolo_model = load_pretrained_model('yolov8n.pt')

    # Train our model
    yolo_model = train_model(yolo_model)

    # Validate our model
    validation_model(yolo_model)

    # Export our model
    export_OpenVINO(yolo_model)

    # Load OpenVINO model
    yolo_model = load_OpenVINO_model('best_openvino_model/')

    validation_model(yolo_model)

    #single_detection(yolo_model, 'file')
    webcam_detection(yolo_model)
    

if __name__ == '__main__':
    main()
