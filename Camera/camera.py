from picamera import PiCamera
from time import sleep
import cv2


def reconocedor_imagen(direccion):
    image = cv2.imread(direccion)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cv2.imshow("Image", thresh)
    cv2.waitKey(0)

camera = PiCamera()
camera.rotation = 180
camera.start_preview()
#Esperamos 2 segundos a que la imagen se ajuste a la luz
#sleep(2)
i = 0
while i < 20:
    #Tomamos la foto
    #camera.start_preview()
    sleep(20)
    camera.capture('/home/pi/Documents/Larc 2018-2019/Robot A/Camera/Images/image%s.jpg' % i)
    #camera.stop_preview()
    #Empezamos a reconocer la imagen
    #reconocedor_imagen('/home/pi/Documents/Larc 2018-2019/Robot A/Camera/Images/image.jpg')

    i += 1
#camera.stop_preview()
