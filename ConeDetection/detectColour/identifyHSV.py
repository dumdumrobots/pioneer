import cv2

def coords_mouse_disp(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK: #left mouse double click 
         #print("Orginal BGR:",img[y,x])
         print("HSV values:", HSV[y,x])

path = "queryCones\IMG_20240503_154832333.jpg"
while True:
    img = cv2.imread(path)
    img = cv2.resize(img, None, fx=0.2, fy=0.2)
    HSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    cv2.imshow("Original",img)
    cv2.imshow("HSV", HSV)
    #left mouse click event
    cv2.setMouseCallback("HSV", coords_mouse_disp)
    cv2.setMouseCallback("Original", coords_mouse_disp)

    if cv2.waitKey(1) &0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
