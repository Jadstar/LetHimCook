import argparse
import time
from pathlib import Path
from PIL import Image as im
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized,TracedModel
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QObject

'''
DEFINITIONS FOR PARAMETERS FOR DETECT:
self.source: The image that is being analysed (path of img/video)
nosave:  (Boolean: 1 if you dont want to save)
self.weights: The Machine Learning self.weights of the data (best.pt)
self.view_img: Displays image (Boolean:1 if you want to view)
self.save_txt: Saves the results 
self.imgsz:
self.trace:
'''
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QProgressBar
        

class BoundaryBox(QLabel):
    def __init__(self, x, y, width, height):
        super().__init__()
        self._x = x
        self._y = y
        self._width = width
        self.height = height
        self.mousePressEvent = self.BoxClicked
        self.setFixedSize(width,height)
        self.setText("I WILL DO MY BEST")

    def BoxClicked(self, event):
        if event.button() == Qt.LeftButton:
            print("clicked box")
            
    def get_position(self) -> tuple:
        return self._position


class DetectThread(QThread):
    progress_update = pyqtSignal(int)

    def __init__(self,source, save_img, weights, view_img, save_txt, imgsz, trace, name, augment, save_conf, iou, confThresh):
        super().__init__()
        self.running = True
        self.source = source
        self.save_img = save_img
        self.weights = weights
        self.view_img = view_img
        self.save_txt = save_txt
        self.imgsz = imgsz
        self.trace = trace
        self.name = name
        self.augment = augment
        self.save_conf = save_conf
        self.iou = iou
        self.confThresh = confThresh
        # outputs 
        self._outputimg = self.source
        self.croppedimg = []
        self.errorFlag = 0
        
        self.dimensions = []
    def returnError(self):
        '''
        Returns Error code if it doesnt work
        '''
        return self.errorFlag
    def getCroppedImgs(self):
        '''
        return: dictionary of the found components path [path:] which were scanned and the boundary box [box:]
        '''
        return self.croppedimg
    def getOutputImg(self):
        '''
        return: The output image that was scanned
        '''
        return self._outputimg
    def run(self):
        # Your existing detect() function code goes here
        # Replace the print statements with self.progress_update.emit(percent_complete)
        # where percent_complete is an integer between 0 and 100 that indicates the progress
        # of the detect() function. For example, if detect() is 50% complete, you would call
        # self.progress_update.emit(50)
        # self.errorFlag = 0
        
        while self.running:
            try:
                self.progress_update.emit(0)
                over = 0

                # webcam = self.source.isnumeric() or self.source.endswith('.txt') or self.source.lower().startswith(
                    # ('rtsp://', 'rtmp://', 'http://', 'https://'))

                # Directories
                save_dir = Path(increment_path(Path('GUI\Database') / self.name))  # increment run
                (save_dir / 'labels' if self.save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

                # Initialize
                set_logging()
                device = select_device('')
                half = device.type != 'cpu'  # half precision only supported on CUDA

                # Load model
                model = attempt_load(self.weights, map_location=device)  # load FP32 model
                stride = int(model.stride.max())  # model stride
                self.imgsz = check_img_size(self.imgsz, s=stride)  # check img_size

                if self.trace:
                    model = TracedModel(model, device,self.imgsz)
                self.progress_update.emit(10)

                # Second-stage classifier
                classify = False
                if classify:
                    modelc = load_classifier(name='resnet101', n=2)  # initialize
                    modelc.load_state_dict(torch.load('self.weights/resnet101.pt', map_location=device)['model']).to(device).eval()

                # Set Dataloader
                vid_path, vid_writer = None, None
                # if webcam:
                #     self.view_img = check_imshow()
                #     cudnn.benchmark = True  # set True to speed up constant image size inference
                #     dataset = LoadStreams(self.source, img_size=self.imgsz, stride=stride)
                # else:
                dataset = LoadImages(self.source, img_size=self.imgsz, stride=stride)

                # Get names and colors
                names = model.module.names if hasattr(model, 'module') else model.names
                colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

                # Run inference
                if device.type != 'cpu':
                    model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(device).type_as(next(model.parameters())))  # run once
                old_img_w = old_img_h = self.imgsz
                old_img_b = 1

                t0 = time.time()
                for path, img, im0s, vid_cap in dataset:
                    img = torch.from_numpy(img).to(device)
                    img = img.half() if half else img.float()  # uint8 to fp16/32
                    img /= 255.0  # 0 - 255 to 0.0 - 1.0
                    if img.ndimension() == 3:
                        img = img.unsqueeze(0)

                    # Warmup
                    if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
                        old_img_b = img.shape[0]
                        old_img_h = img.shape[2]
                        old_img_w = img.shape[3]
                        for i in range(3):
                            model(img, self.augment)[0]

                    # Inference
                    t1 = time_synchronized()
                    with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
                        pred = model(img, self.augment)[0]
                    t2 = time_synchronized()

                    # Apply NMS
                    pred = non_max_suppression(pred, self.confThresh, self.iou, agnostic=1)
                    t3 = time_synchronized()

                    # Apply Classifier
                    if classify:
                        pred = apply_classifier(pred, modelc, img, im0s)

                    # Process detections
                    self.progress_update.emit(20)

                    for i, det in enumerate(pred):  # detections per image
                        
                        # if webcam:  # batch_size >= 1
                        #     p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
                        # else:
                        p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

                        p = Path(p)  # to Path
                        save_path = str(save_dir / p.name)  # img.jpg
                        txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
                        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                        if len(det):
                            bounding_box = []
                            # Rescale boxes from img_size to im0 size
                            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                            # Print results
                            for c in det[:, -1].unique():
                                n = (det[:, -1] == c).sum()  # detections per class
                                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                            # Write results
                            for *xyxy, conf, cls in reversed(det):
                                
                                if self.save_txt:  # Write to file
                                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                                    line = (cls, *xywh, conf) if self.save_conf else (cls, *xywh)  # label format
                                    with open(txt_path + '.txt', 'a') as f:
                                        f.write(('%g ' * len(line)).rstrip() % line + '\n')

                                if self.save_img or self.view_img:  # Add bbox to image
                                    label = f'{names[int(cls)]} {conf:.2f}'
                                    
                                    x1 = int(xyxy[0].item())
                                    y1 = int(xyxy[1].item())
                                    x2 = int(xyxy[2].item())
                                    y2 = int(xyxy[3].item())

                                    self.dimensions.append([x1,x2,y1,y2])
                                    confidence_score = conf
                                    class_index = cls
                                    object_name = names[int(cls)]

                                    original_img = im0
                                    cropped_img = im0[y1:y2, x1:x2]
                                    if(object_name=='Broodje'):
                                        resistor_path = f'GUI\Database\croppedImgs\{object_name}_{over}.png'
                                        if cropped_img.shape[0] > cropped_img.shape[1]:
                                            # Rotate the image by 90 degrees
                                            cropped_img = cv2.rotate(cropped_img, cv2.ROTATE_90_CLOCKWISE)
                                        cv2.imwrite(resistor_path, cropped_img)
                                        # Create an instance of BoundingBoxItem
                                        bounding_box = {
                                            'x': x1,
                                            'y': y1,
                                            'width': x2 - x1,
                                            'height': y2 - y1
                                        }

                                        # Create an instance of BoundingBoxItem
                                        # Assuming you have the bounding box coordinates relative to the image
                                        x, y, width, height = bounding_box['x'], bounding_box['y'], bounding_box['width'], bounding_box['height']

                                        # Assuming you have the image label widget object named `imglabel`
                                        imglabel_x = 40  # X-coordinate of the image label
                                        imglabel_y = 30  # Y-coordinate of the image label

                                        # Calculate the adjusted bounding box coordinates
                                        adjusted_x = imglabel_x + x
                                        adjusted_y = imglabel_y + y

                                        # Create the BoundingBoxItem with adjusted coordinates
                                        bounding_box_item = {'x':adjusted_x,
                                                            'y': adjusted_y,
                                                            'width': width,
                                                            'height': height
                                                            }
                
                                        self.croppedimg.append({'path': resistor_path, 'box': bounding_box_item})
                                        over = over+1
                                        print(self.croppedimg)
                                    plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)

                                    # Get the boundaries of the boxes to crop the output image with all items in it
                                    

                        #Find the Min and max dimensions of all the components so we can crop out wasted space
                        min_x = float('inf')
                        max_x = float('-inf')
                        min_y = float('inf')
                        max_y = float('-inf')
                        for d in self.dimensions:
                            x1, x2, y1, y2 = d
                            if x1 < min_x:
                                min_x = x1
                            if x2 > max_x:
                                max_x = x2
                            if y1 < min_y:
                                min_y = y1
                            if y2 > max_y:
                                max_y = y2
                        print(min_y)
                        print(max_y)
                        print(min_x)
                        print(max_x)
                        # Calculate the coordinates for the bounding box encompassing all bounding boxes
                        
                        # Add padding
                        padding = 100
                        min_x = max(0, min_x - padding)
                        max_x = min(im0.shape[1], max_x + padding)
                        min_y = max(0, min_y - padding)
                        max_y = min(im0.shape[0], max_y + padding)

                        # Calculate the aspect ratio of the original image
                        original_width = im0.shape[1]
                        original_height = im0.shape[0]
                        aspect_ratio = original_width / original_height

                        # Calculate the dimensions of the cropped image
                        crop_width = max_x - min_x
                        crop_height = max_y - min_y

                        # If the cropped image does not have the same aspect ratio as the original image, adjust its dimensions
                        if crop_width / crop_height != aspect_ratio:
                            if crop_width / crop_height < aspect_ratio:
                                # The cropped image is too tall and narrow, adjust its width
                                adjust_width = int(crop_height * aspect_ratio)
                                # Add half of the adjustment to the left and half to the right
                                min_x = max(0, min_x - (adjust_width - crop_width) // 2)
                                max_x = min(im0.shape[1], max_x + (adjust_width - crop_width) // 2)
                            else:
                                # The cropped image is too short and wide, adjust its height
                                adjust_height = int(crop_width / aspect_ratio)
                                # Add half of the adjustment to the top and half to the bottom
                                min_y = max(0, min_y - (adjust_height - crop_height) // 2)
                                max_y = min(im0.shape[0], max_y + (adjust_height - crop_height) // 2)

                        # Crop the image
                        cropped_img = im0[min_y:max_y, min_x:max_x]

                        # Assign the cropped image to self._outputimg
                        self._outputimg = cropped_img

                        # self._outputimg = croppedOutput
                        # Print time (inference + NMS)
                        print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
                        self.progress_update.emit(75)

                        # Stream results
                        if self.view_img:
                            # cv2.imshow(str(p), im0)
                            cv2.waitKey(1)  # 1 millisecond

                        # Save results (image with detections)
                        if self.save_img:
                            if dataset.mode == 'image':
                                cv2.imwrite(save_path, im0)
                                print(f" The image with the result is saved in: {save_path}")
                                self.progress_update.emit(95)
                            else:  # 'video' or 'stream'
                                if vid_path != save_path:  # new video
                                    vid_path = save_path
                                    if isinstance(vid_writer, cv2.VideoWriter):
                                        vid_writer.release()  # release prevself.ious video writer
                                    if vid_cap:  # video
                                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                                    else:  # stream
                                        fps, w, h = 30, im0.shape[1], im0.shape[0]
                                        save_path += '.mp4'
                                    vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                                vid_writer.write(im0)

                if self.save_txt or self.save_img:
                    s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if self.save_txt else ''
                    #print(f"Results saved to {save_dir}{s}")

                print(f'Done. ({time.time() - t0:.3f}s)')
                self.progress_update.emit(100)
                self.running = False
                
            except:
                self.errorFlag = 1
                # self.running = False
                print("error")
            

'''
def detect(source,save_img,weights,view_img,save_txt,imgsz,trace,name,augment,save_conf,iou,confThresh):
    # save_img = not opt.nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    save_dir = Path(increment_path(Path('GUI\Database') / name))  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    device = select_device('')
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    if trace:
        model = TracedModel(model, device,imgsz)

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    old_img_w = old_img_h = imgsz
    old_img_b = 1

    t0 = time.time()
    for path, img, im0s, vid_cap in dataset:
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, confThresh, iou, agnostic=1)
        t3 = time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
           
            if webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
            else:
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # img.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                over = 0
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    over = over+1
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or view_img:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}'
                        
                        x1 = int(xyxy[0].item())
                        y1 = int(xyxy[1].item())
                        x2 = int(xyxy[2].item())
                        y2 = int(xyxy[3].item())

                        confidence_score = conf
                        class_index = cls
                        object_name = names[int(cls)]

                        original_img = im0
                        cropped_img = im0[y1:y2, x1:x2]
                        if(object_name=='resistor'):
                            cv2.imwrite(f'{object_name}_{over}.jpg', cropped_img)

                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)


            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Stream results
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                    print(f" The image with the result is saved in: {save_path}")
                else:  # 'video' or 'stream'
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path += '.mp4'
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer.write(im0)

    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        #print(f"Results saved to {save_dir}{s}")

    print(f'Done. ({time.time() - t0:.3f}s)')
    return original_img
'''


# CommandLine Stuff, no need
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--weights', nargs='+', type=str, default='yolov7.pt', help='model.pt path(s)')
#     parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
#     parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
#     parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
#     parser.add_argument('--iou-thres', type=float, default=0.45, help='iou threshold for NMS')
#     parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
#     parser.add_argument('--view-img', action='store_true', help='display results')
#     parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
#     parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
#     parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
#     parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
#     parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
#     parser.add_argument('--augment', action='store_true', help='augmented inference')
#     parser.add_argument('--update', action='store_true', help='update all models')
#     parser.add_argument('--project', default='runs/detect', help='save results to project/name')
#     parser.add_argument('--name', default='exp', help='save results to project/name')
#     parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
#     parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
#     opt = parser.parse_args()
#     print(opt)
#     #check_requirements(exclude=('pycocotools', 'thop'))

#     with torch.no_grad():
#         if opt.update:  # update all models (to fix sourceChangeWarning)
#             for opt.weights in ['yolov7.pt']:
#                 detect()
#                 strip_optimizer(opt.weights)
#         else:
#             detect()