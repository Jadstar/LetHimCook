from PyQt5.QtGui import QImage, QPixmap
from PyQt5.uic import loadUi
from PyQt5.QtCore import Qt,QSize,pyqtSignal
from PyQt5.QtWidgets import QGridLayout,QLabel,QListWidget,QDialog,QFileDialog,QMessageBox,QVBoxLayout,QListWidgetItem
from componentimg import imagingComponents,Camera
# from PIL.ImageQt import ImageQt
from  detect import DetectThread,BoundaryBox
import os
import tempfile
import numpy as np
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import Qt

class Ui_OutputDialog(QDialog):

    _imaging = imagingComponents
    layout = QGridLayout()
    patties_detected_signal = pyqtSignal(list)  # Signal that emits a list

    tempImgpath = ""                #temporary path that will be used in case photo is taken
    def __init__(self):
        super(Ui_OutputDialog, self).__init__()
        loadUi("out_window.ui", self)
        defaultImg = QPixmap(self._imaging.getImg(self))
        self.imglabel.setPixmap(defaultImg)     #setting default images 
        self.imglabel.setScaledContents(True)
        self._imaging = imagingComponents
        # Check Cameras
        self._imaging.setAvailableCams(self)
        
        if len(self._imaging.getAvailableCams(self)) >0:
            
            self.selectCam.setText("Selected Camera: " + str(self._imaging.getCam(self)[0]))
        
        self.selectCam.clicked.connect(self.show_available_webcams)
        
        self.UploadImageButton.clicked.connect(self.openUploadImg)
        self.beginButton.clicked.connect(self.startCamera)
       
        self.stopButton.setEnabled(False)
        self.stopButton.clicked.connect(self.stopCamera)

        self.photoButton.setEnabled(False)
        self.photoButton.clicked.connect(self.takePhoto)

        self.progress_bar.setVisible(False)

        self.ComponentValue.setEnabled(False)
        self.ComponentValue.clicked.connect(self.return_patties)

        self.AddSimulatorButton.clicked.connect(self.AddToSim)
        self.AddSimulatorButton.setEnabled(False)
        #Elec sim portion
        # self._new_window = MainSpace()


    def return_patties(self):
        cropped_imgs = self.detect_thread.getCroppedImgs()
        self.patties_detected_signal.emit(cropped_imgs)  # Emit the signal with the cropped images
        self.close()  # Close the dialog
    def show_available_webcams(self):
        '''
        Shows the available webcams connected to the device
        '''
        msg_box = QMessageBox(self)
        list_widget = QListWidget(self)
        print("clicked show webcam")
        camlist = self._imaging.getAvailableCams(self)
        currentcam =self._imaging.getCam(self)
        for i, camera in enumerate(camlist):
            item = QListWidgetItem(f"Webcam: {camera[0]}", list_widget)
            item.setFlags(item.flags())
            list_widget.addItem(item)
            if currentcam[1] == camera[1]:
                print("the default camera is" + str(currentcam[0]))
                list_widget.setCurrentRow(i)


        msg_box.setWindowTitle("Select Webcam")
        msg_box.setText("Available webcams:")
        msg_box.adjustSize()
        msg_box.layout().addWidget(list_widget)

        list_widget.itemDoubleClicked.connect(lambda item: msg_box.done(QMessageBox.Ok))

        result = msg_box.exec_()
        if result == QMessageBox.Ok:
            selected_index = list_widget.currentRow()
            print("setting camera " + str(selected_index))
            self._imaging.setCam(self,camlist[selected_index])
            self.selectCam.setText("Selected Camera: " + str(self._imaging.getCam(self)[0]))

        else:
            return None



    def ConfirmScan(self,img):
        '''
        An additional popup which will appear after a photo is taken/image is uploaded
        to confirm the scanning process
        '''
        confirmPopup = QMessageBox(self)
        confirmPopup.setWindowTitle("Would you like to scan the following image?")
        confirmLayout = QVBoxLayout()
        confirmImg = QPixmap(img).scaled(700, 500, Qt.KeepAspectRatio)
        confirmPopup.setIconPixmap(confirmImg)
        confirmPopup.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        confirmPopup.setDefaultButton(QMessageBox.Yes)
        
        result = confirmPopup.exec_()

        if result == QMessageBox.Yes:
            print('Yes button clicked')
            self.EM_flag = False        #Used so that the message error box will only appear once 
            return 1 
        else:
            print('No button clicked')
            return 0

    @pyqtSlot()
    def AddToSim(self):
        '''
        Takes current component and add it into the electrical simulation interface
        '''

        print("Adding to simulation")

        # #getting resistor data
        # value = self.Value.text()
        # tolerance = self.ToleranceValue.text()
        # resistor =  ElecC.Resistor((30,30),value)

        # # Adding the new resistor to the list of components
        # self._new_window.components.append(resistor)

        # # Adding all components to the new window
        # for component in self._new_window.components:
        #     self._new_window.AddComponent(component)
        # #Opening simulation window
        # self._new_window.show()

    def update_progress(self, percent_complete):
        '''
        Updates the Loading Bar progress when detect.py is running
        '''
        if self.EM_flag == False:
            self.progress_bar.setValue(percent_complete)

        #Checking for any errors
        print( self.detect_thread.returnError())
        
        if self.detect_thread.returnError() and self.EM_flag ==False:

            self.EM_flag = True        #Used so that the message error box will only appear once 
            self.detect_thread.running = False
            self.progress_bar.setValue(0)
            errorPopup = QMessageBox(self)
            errorPopup.setWindowTitle("Scanning Error")
            errorPopup.setText("We have not found any Patties in this image")
            self.Contours.setText("No Patties found")

            errorPopup.exec_()
        #After YoloV7 is complete 
        if percent_complete == 100:
            self.progress_bar.setVisible(False)
            self.Contours.setText("Scan Complete, we found " + str(len(self.detect_thread.getCroppedImgs()))+ " Patties")
            self.Contours.adjustSize()
            # self.close()  # Close the PyQt window
            # return self.detect_thread.getCroppedImgs()
            self._imaging.setImg(self,self.detect_thread.getOutputImg())
            # Since the output image is a cropped version of the original, we must show that on GUI
        
            img_array = self._imaging.getImg(self)  # Assuming this returns a numpy array
        
           # Ensure the array is in contiguous memory
            img_array = np.ascontiguousarray(img_array)

            # Convert the array to QImage
            height, width, channel = img_array.shape
            bytesPerLine = 3 * width
            qimage = QImage(img_array.data.tobytes(), width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
            
            # Convert QImage to QPixmap
            qpixmap = QPixmap.fromImage(qimage)
            self.imglabel.setPixmap(qpixmap)
            self.imglabel.show()
            # Clean up: remove the temporary file
            if self.tempImgpath != "":
                os.remove(self.tempImgpath)


            self.ComponentValue.setEnabled(True)
                # self.layout.addWidget(bbox,bbox.x(),bbox.y())
                
          

    def ImageIdentify(self):
        '''
        This will run the yolov7 algorithm on the image to detect the 
        components within it (found in detect.py with the weights in best.pt)
        '''

                #MACHINE LEARNING FUNCTION IN DETECT.PY
        source = self._imaging.getImg(self)
        save_img = 1
        try:
            print("PATH BELOW")
            weights = os.path.abspath(r"best.pt")
            print(weights)
            # weights = r'C:\Users\Jaden\Github\the-rock\GUI\best.pt'
        except:
            print("CANNOT FIND PATH")

        # TELL EVERYTHING THAT IT IS CURRENTLY LOADING 
        self.Contours.setText("Scanning Image...")
        self.progress_bar.setVisible(True)
        
        view_img =1
        save_txt = 0
        imgsz = 640
        trace = 1
        name ='exp'
        augment = 1
        confThresh = 0.15
        iou = 0.45
        save_conf = 1
        self.detect_thread = DetectThread(source,save_img,weights,view_img,save_txt,imgsz,trace,name,augment,save_conf,iou,confThresh)
        self.detect_thread.progress_update.connect(self.update_progress)
        self.detect_thread.start()
        # self.detect_thread.wait()


    def takePhoto(self):
        '''
        Gets the current frame from the Camera
        (works when camera start and stopped)
        '''

        photo = self.cameralabel.pixmap()
        if photo is not None: 
            # Convert QPixmap to QImage
            qimage = QPixmap.toImage(photo)
            # Create a temporary file to save the QImage
            temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".png")
            self.tempImgpath = temp_file.name
            # Save QImage to the temporary file
            qimage.save(self.tempImgpath)
            
            if self.ConfirmScan(self.tempImgpath):
                self._imaging.setImg(self, self.tempImgpath)
                # Load the image from the file path
                image_path = self._imaging.getImg(self)
                pixmap = QPixmap(image_path)
                # Set the pixmap for the label
                self.imglabel.setPixmap(pixmap)
                # self.imglabel.setScaledContents(True)
                self.ImageIdentify()

            

    def stopCamera(self):
        '''
        Stops the Camera via the stop button 
        '''
        print("Clicked Stop Camera")
        self.beginButton.setEnabled(True)
        self.beginButton.setText("Start Camera")
        self.beginButton.adjustSize()
        # resetting image and text

        self.stopButton.setEnabled(False)
        self.thread.stop()
        
        # need to wait otherwise you wont be able to turn camera back on lol
        self.thread.wait()
        # Set transparent pixmap to clear image        
        self.cameralabel.setText("Click the 'Start Camera' Button to Begin Video")


        
    def startCamera(self):
        '''
        Starts the Camera from the button 
        '''
        print("Clicked Start Camera")
        self.beginButton.setEnabled(False)
        self.beginButton.setText("Camera Running")
        self.beginButton.adjustSize()

        self.cameralabel.setText("Loading Camera")
        self.stopButton.setEnabled(True)
        self.photoButton.setEnabled(True)

        # Loading Camera

        # Start Thread
        self.thread = Camera()
        self.thread.image_signal.connect(self.update_camera)
        self.thread.start()
        
    def openUploadImg(self):
        '''
        File explorer opens up when 'Upload Image' is clicked,
        allowing you to choose an image
        '''
        options = QFileDialog.Options()
        # options |= QFileDialog.DontUseNativeDialog
        filePath, _ = QFileDialog.getOpenFileName(self,"Select a Image To Scan", "","Image Files (*.png *.jpg *.bmp)", options=options)
        if filePath:
            # Load selected image file and set as pixmap for label
            if self.ConfirmScan(filePath):
                self._imaging.setImg(self,filePath)
                pixmap = QPixmap(self._imaging.getImg(self))
                self.imglabel.setPixmap(pixmap)
                self.imglabel.setScaledContents(True)
                # Run the Machine Learning on code
                self.ImageIdentify()

    def UpdateData(self):
       
        # updating data
        # self.ImageIdentify()

        #Setting Data is n/a, until its populated


        # Contours =ImageQt(self._imaging.getContours(self))
        # readimg = ImageQt(self._imaging.findColours(self))
        # histPixmap = QPixmap.fromImage(readimg).scaled(500,80,aspectRatioMode=Qt.KeepAspectRatio)
        self.Value.setText("N/A")
        # self.ToleranceValue.setText("N/A")
        # labels = [self.c1_value, self.c2_value, self.c3_value, self.c4_value, self.c5_value]
        # for i in labels:
        #     i.setText("N/A")

        # numbands = len(self._imaging.getColourBands(self))

        # print(numbands)
        # if numbands >= 3:

        #     self.Contours.setPixmap(histPixmap)
        #     self.Value.setText(self._imaging.getResistanceValue(self) + "Ω") 
        #     self.ToleranceValue.setText(self._imaging.getTolerance(self))

        #     # updating colour bands (we have to check how many bands)
            
        #     colour_bands = self._imaging.getColourBands(self)
        #     print(colour_bands)
        #     labels = [self.c1_value, self.c2_value, self.c3_value, self.c4_value, self.c5_value]
        #     print(labels)
        #     for i in range(numbands):
        #         labels[i].setText(colour_bands[i])
        # else:
        #     self.Contours.setText("Error finding value of resistor")
        # #If there is data, allow adding to simulation
        # if self.Value.text() != "N/A" or self.Value.text() != "-": 
        #     self.AddSimulatorButton.setEnabled(True)


    def update_camera(self, frame):
        '''
        Runs the Camera thread and should start when start running is pressed and stop when stop is pressed
        '''

        if frame is not None:
            self.cameralabel.setPixmap(QPixmap.fromImage(frame))
            self.cameralabel.setScaledContents(True)

    
    def closeCamera(self, event):
        """Stops the video feed thread when the GUI is closed"""
        self.thread.stop()
        event.accept()