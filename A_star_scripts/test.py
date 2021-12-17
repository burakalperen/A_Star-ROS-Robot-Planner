from PIL import Image
import cv2
import numpy as np


def pixel_to_world(x,y):

    world_x = ((x - 1000.0) / 2000.0) * 125.0

    world_y = ((350 - y) / 700) * 43.75

    return world_x,world_y


"""
1000 + ((w_x / 125) * 2000)  = p_x
w_x = ((p_x - 1000) / 2000) * 125


350 - ((w_y / 43.75) * 700) = p_y
w_y = ((350 - p_y) / 700) * 43.75

"""

# map de start 136,230
# map de finish 1095 151


def heapify_heappush_heappop():
    import heapq


    li = [5,7,9,1,3]
    print("Initial list: ",li)

    # using heapify to convert list into heap
    
    heapq.heapify(li)
    print("After heapify: ",li)

    heapq.heappush(li,10)
    print("After heappush: ",li)

    h = []
    heapq.heappush(h, (5, 'write code'))
    print(h)
    heapq.heappush(h, (7, 'release product'))
    print(h)
    heapq.heappush(h, (1, 'write spec'))
    print(h)
    heapq.heappush(h, (3, 'create tests'))
    print(h)
    print(heapq.heappop(h)[1])
    print(h)
    




if __name__ == "__main__":
    
    #img = cv2.imread("./Map.png")
    # # print(img.shape)
 
    #heapify_heappush_heappop()

    a = 2
    b = 30
    c = 0



    # x,y = pixel_to_world(136,230)
    # print("x: ",x, "y: ",y)


    # cv2.imshow("img",img)
    # cv2.waitKey(0)
