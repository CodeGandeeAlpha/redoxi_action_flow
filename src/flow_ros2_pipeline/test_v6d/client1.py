from time import sleep
import vineyard
import cv2

img = cv2.imread('/mnt/chengxiao/code/MOTRv2/0.jpg')

client = vineyard.connect('/var/run/vineyard.sock')
object_id = client.put(img)

print(object_id)

sleep(1000000)