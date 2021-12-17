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

if __name__ == "__main__":
    
    img = cv2.imread("./Map.png")
    # # print(img.shape)
    
    # # map_image = Image.open("Map.png")
    # # width,height = map_image.size
    # # print("w: ",width, "h: ",height)


    x,y = pixel_to_world(136,230)
    print("x: ",x, "y: ",y)


    cv2.imshow("img",img)
    cv2.waitKey(0)


    """
    map_image = Image.open("Map.png")
    width, height = map_image.size
    pixels = map_image.load()
    grid_map = []
    for x in range(width):
        # x: 0 - 2000
        
        row = []
        for y in range(height):
            
            if pixels[x,y] == 0:
                row.append(False)
            else:
                print(x,y)
                row.append(True)
            
        grid_map.append(row)

    print(pixels[1095,151])
    print(grid_map[1095][151])
    print(pixels[572,341])
    print(grid_map[572][341])

   print(len(grid_map[0][:]))
   """