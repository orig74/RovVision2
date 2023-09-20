import cv2
import numpy as np
import random
import math
import time


GRAY_THRESHOLD_MUL = 0.55  # * mean gray value
PCA_LENGTH_THRESH_MUL = 0.1  # * img width
PCA_W2H_RATIO = 2.0
ERROSION_SCALE_MUL = 0.01
VERT_THRESHHOLD_MUL = 0.125


def pca(points):
    u = np.mean(points, axis=0)
    cov_mat = np.cov(points - u, rowvar=False)
    eig_vals, eig_vecs = np.linalg.eigh(cov_mat)
    return eig_vecs[:, ::-1], eig_vals[::-1], u

def detect(img,px=None,annotate=False):
    img_rs = img.copy() # cv2.resize(img, (2000, 1600))
    pca_scale_thresh = img_rs.shape[0] * PCA_LENGTH_THRESH_MUL
    k_size = int(ERROSION_SCALE_MUL * img_rs.shape[0])
    k_size += 1 if k_size % 2 == 0 else 0
    k_size = max(k_size, 3)
    verts_thresh = int(VERT_THRESHHOLD_MUL * img_rs.shape[0])

    img_grey = cv2.cvtColor(img_rs, cv2.COLOR_BGR2GRAY)
    thresh = int(img_grey.mean() * GRAY_THRESHOLD_MUL)
    ret, img_mask = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((k_size, k_size), np.uint8)
    morph_final = cv2.dilate(img_mask, kernel, iterations=1)

    contours, _ = cv2.findContours(morph_final, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    angles = []
    centers = []
    for i in range(len(contours)):
        if len(contours[i]) > verts_thresh:
            contours_np = np.array(contours[i])[:, 0]
            ds_step = contours_np.shape[0] // 20
            contours_np = contours_np[::ds_step]
            pc_vecs, pc_lengths, center_pnt = pca(contours_np)
            if True or pc_lengths[0] > pca_scale_thresh and pc_lengths[0] / pc_lengths[1] > PCA_W2H_RATIO:
                centers.append(center_pnt)
                y_val = np.sign(pc_vecs[0, 0]) * pc_vecs[1, 0]
                angle = round(math.degrees(math.acos(y_val / np.linalg.norm(pc_vecs[:, 0]))))
                if angle > 90:
                    angle = angle - 180
                angles.append(angle)
                if annotate:
                    color = tuple([random.randint(0, 256) for i in range(3)])
                    cv2.drawContours(img_rs, contours, i, color, 3)
                    cv2.putText(img_rs, str(angle), tuple(center_pnt.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)

    closest_idx=None
    if px is not None:
        distances = np.linalg.norm(np.array(centers) - px, axis=1)
        closest_idx = np.argmin(distances)
    return {'img':img_rs if annotate else None,
            'centers':centers,
            'angles':angles,
            'closest_idx':closest_idx
            }


if __name__ == "__main__":
    img = cv2.imread("/home/tim/Dropbox/Pictures/mussels_GB.png")

    img_rs = img # cv2.resize(img, (2000, 1600))
    pca_scale_thresh = img_rs.shape[0] * PCA_LENGTH_THRESH_MUL
    k_size = int(ERROSION_SCALE_MUL * img_rs.shape[0])
    k_size += 1 if k_size % 2 == 0 else 0
    k_size = max(k_size, 3)
    verts_thresh = int(VERT_THRESHHOLD_MUL * img_rs.shape[0])

    img_grey = cv2.cvtColor(img_rs, cv2.COLOR_BGR2GRAY)
    thresh = int(img_grey.mean() * GRAY_THRESHOLD_MUL)
    ret, img_mask = cv2.threshold(img_grey, thresh, 255, cv2.THRESH_BINARY_INV)

    kernel = np.ones((k_size, k_size), np.uint8)
    morph_final = cv2.dilate(img_mask, kernel, iterations=1)

    contours, _ = cv2.findContours(morph_final, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    angles = []
    centers = []
    for i in range(len(contours)):
        if len(contours[i]) > verts_thresh:
            contours_np = np.array(contours[i])[:, 0]
            ds_step = contours_np.shape[0] // 20
            contours_np = contours_np[::ds_step]
            pc_vecs, pc_lengths, center_pnt = pca(contours_np)
            if True or pc_lengths[0] > pca_scale_thresh and pc_lengths[0] / pc_lengths[1] > PCA_W2H_RATIO:
                centers.append(center_pnt)
                y_val = np.sign(pc_vecs[0, 0]) * pc_vecs[1, 0]
                angle = round(math.degrees(math.acos(y_val / np.linalg.norm(pc_vecs[:, 0]))))
                if angle > 90:
                    angle = angle - 180
                angles.append(angle)
                color = tuple([random.randint(0, 256) for i in range(3)])
                cv2.drawContours(img_rs, contours, i, color, 3)
                cv2.putText(img_rs, str(angle), tuple(center_pnt.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)

    px = [100, 200]
    distances = np.linalg.norm(np.array(centers) - px, axis=1)
    closest_idx = np.argmin(distances)

    cv2.imshow("Img original", img_rs)
    cv2.imshow("Img mask", morph_final)

    cv2.waitKey()
