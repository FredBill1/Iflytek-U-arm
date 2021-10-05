import cv2 as cv
import numpy as np

def template_demo():
    src = cv.imread("./data/test.jpg")
    tpl = cv.imread("./data/muban.png")
    cv.imshow("input", src)
    cv.imshow("tpl", tpl)
    th, tw = tpl.shape[:2]
    result = cv.matchTemplate(src, tpl, cv.TM_CCORR_NORMED)

    t = 0.98
    loc = np.where(result > t)

    for pt in zip(*loc[::-1]):
        cv.rectangle(src, pt, (pt[0] + tw, pt[1] + th), (0, 0, 255), 1, 8, 0)
    cv.imshow("match-demo", src)
    cv.imwrite("./data/match_result.png", src)

if __name__ == '__main__':
    try:
		template_demo()
		cv.waitKey(0)
    except KeyboardInterrupt:
        cv.destroyAllWindows()
