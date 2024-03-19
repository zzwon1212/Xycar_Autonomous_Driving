import numpy as np
import cv2

img = cv2.imread("../images/depth.png")

img_points = np.array(
    [
        # [167, 218], [148, 218], [254, 216],
        [157, 221], [210, 222], [263, 221],
        [143, 228], [210, 227], [279, 228],
        [113, 241], [210, 240], [307, 240],
        [41, 273], [210, 271], [379, 271]
    ],
    dtype = np.float32
    )
world_points = np.array(
    [
        # for getting real homography
        # # [180, 90, 1], [270, 90, 1], [360, 90, 1],
        [180, 180, 1], [270, 180, 1], [360, 180, 1],
        [180, 270, 1], [270, 270, 1], [360, 270, 1],
        [180, 360, 1], [270, 360, 1], [360, 360, 1],
        [180, 450, 1], [270, 450, 1], [360, 450, 1]

        # for just drawing grid image
        # [180, 90, 1], [270, 90, 1], [360, 90, 1],
        # [90, 90, 1], [135, 90, 1], [180, 90, 1],
        # [90, 135, 1], [135, 135, 1], [180, 135, 1],
        # [90, 180, 1], [135, 180, 1], [180, 180, 1],
        # [90, 225, 1], [135, 225, 1], [180, 225, 1]
    ],
    dtype = np.float32
    )

H_mat, _ = cv2.findHomography(img_points, world_points)
print(H_mat)

img_points = np.hstack((img_points, np.ones((img_points.shape[0], 1), dtype=np.float32)))

estimated_world_points = np.dot(H_mat, img_points.T).T
estimated_world_points = estimated_world_points / (estimated_world_points[:, 2][:, None] + 1e-6)

def draw_grid(world_points, window_name="Estimation", color=(0, 0, 255)):
    grid_img = np.ones((273, 273, 3), dtype=np.uint8) * 255

    for i in range(grid_img.shape[0] // 45):
        grid_img[45 * i, :, :] = [0, 0, 0]
        grid_img[:, 45 * i, :] = [0, 0, 0]

    x_points, y_points = world_points[:, 0], world_points[:, 1]
    distances = np.round(np.sqrt((x_points - 135)**2 + (y_points - 270)**2)).astype(int)
    x_points, y_points = np.round(x_points).astype(int), np.round(y_points).astype(int)

    cv2.circle(grid_img, center=(135, 270), radius=4, color=(255, 180, 0), thickness=-1)
    cv2.putText(grid_img, text=f"My Car", org=(135, 270), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 0))
    for x, y, d in zip(x_points, y_points, distances):
        cv2.circle(grid_img, center=(x, y), radius=4, color=color, thickness=-1)
        cv2.putText(grid_img, text=f"{d}", org=(x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 0))
        print(x, y, d)

    cv2.imshow(f"{window_name}", grid_img)

draw_grid(world_points[:, :2], window_name="Ground Truth", color=(255, 0, 0))
draw_grid(estimated_world_points)
cv2.waitKey()
