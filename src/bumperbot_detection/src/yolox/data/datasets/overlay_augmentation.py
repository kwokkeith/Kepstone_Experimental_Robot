import cv2
import numpy as np
import random
import matplotlib.pyplot as plt
import math


def is_overlapping(box1, box2):
    """
    Check if two bounding boxes overlap.

    Parameters:
    box1 (numpy array): Bounding box represented as [x1, y1, x2, y2]
    box2 (numpy array): Bounding box represented as [x1, y1, x2, y2]

    Returns:
    bool: True if the bounding boxes overlap, False otherwise
    """
    # Extract coordinates
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2

    # Check for no overlap conditions
    if x1_max < x2_min or x2_max < x1_min:
        return False
    if y1_max < y2_min or y2_max < y1_min:
        return False

    return True


def augment_object_detection(image, object_img, mask, sqrt_area_min, sqrt_area_max, affine_ratio, obj_alpha_min):
    H, W = image.shape[:2]
    H_o, W_o = object_img.shape[:2]

    # Random rotation
    angle = random.uniform(0, 360)
    M = cv2.getRotationMatrix2D((W_o/2, H_o/2), angle, 1)
    rotated_object = cv2.warpAffine(object_img, M, (W_o, H_o))
    rotated_mask = cv2.warpAffine(mask, M, (W_o, H_o))

    # Find bounding box of the mask
    x, y, w, h = cv2.boundingRect(rotated_mask)

    # Extract the rotated object and mask within the bounding box
    rotated_object_cropped = rotated_object[y:y+h, x:x+w]
    rotated_mask_cropped = rotated_mask[y:y+h, x:x+w]

    # Random scaling
    scale_factor = random.uniform(sqrt_area_min, sqrt_area_max) / np.sqrt(h * w + 1e-5)
    if h*w == 0:
        return None, None, None
    new_h, new_w = int(h * scale_factor), int(w * scale_factor)
    resized_object = cv2.resize(rotated_object_cropped, (new_w, new_h))
    resized_mask = cv2.resize(rotated_mask_cropped, (new_w, new_h))

    # Apply random affine transformation
    pts1 = np.float32([[0, 0], [new_w, 0], [0, new_h]])
    pts2 = np.float32([[random.uniform(-affine_ratio, affine_ratio) * new_w, random.uniform(-affine_ratio, affine_ratio) * new_h],
                       [new_w + random.uniform(-affine_ratio, affine_ratio) * new_w, random.uniform(-affine_ratio, affine_ratio) * new_h],
                       [random.uniform(-affine_ratio, affine_ratio) * new_w, new_h + random.uniform(-affine_ratio, affine_ratio) * new_h]])

    affine_matrix = cv2.getAffineTransform(pts1, pts2)
    transformed_object = cv2.warpAffine(resized_object, affine_matrix, (new_w, new_h))
    transformed_mask = cv2.warpAffine(resized_mask, affine_matrix, (new_w, new_h))

    # Find new bounding box of the transformed mask
    x, y, w, h = cv2.boundingRect(transformed_mask)
    if h*w == 0:
        return None, None, None

    # Extract the transformed object and mask within the new bounding box
    transformed_object_cropped = transformed_object[y:y + h, x:x + w]
    transformed_mask_cropped = transformed_mask[y:y + h, x:x + w]

    # Random position
    x1 = random.randint(0, W - w)
    y1 = random.randint(0, H - h)
    x2, y2 = x1 + w, y1 + h

    # Create a copy of the image to place the object
    augmented_image = image.copy()
    overlay_mask = np.zeros((augmented_image.shape[0], augmented_image.shape[1]), dtype=np.bool_)

    # Overlay the object onto the image using the mask
    obj_alpha = random.uniform(obj_alpha_min, 1.0)

    # affine overlay
    aug_img_hsv = cv2.cvtColor(augmented_image, cv2.COLOR_BGR2HSV).astype(np.float64)
    obj_img_hsv = cv2.cvtColor(transformed_object_cropped, cv2.COLOR_BGR2HSV).astype(np.float64)
    aug_img_mean = np.mean(aug_img_hsv.reshape((-1, 3)), axis=0)
    obj_img_mean = np.mean(obj_img_hsv.reshape((-1, 3)), axis=0)
    obj_img_hsv[:, :, 1] *= (aug_img_mean[1] / obj_img_mean[1])
    obj_img_hsv[:, :, 2] *= (aug_img_mean[2] / obj_img_mean[2])
    obj_img_hsv = np.clip(obj_img_hsv, a_min=0, a_max=255).astype(np.uint8)
    transformed_object_cropped = cv2.cvtColor(obj_img_hsv, cv2.COLOR_HSV2BGR)

    for c in range(3):  # Assuming image has 3 channels (RGB)
        augmented_image[y1:y2, x1:x2, c] = np.where(
            transformed_mask_cropped > 0,
            transformed_object_cropped[:, :, c] * obj_alpha + (1 - obj_alpha) * augmented_image[y1:y2, x1:x2, c],
            augmented_image[y1:y2, x1:x2, c]
        )
    overlay_mask[y1:y2, x1:x2] = np.logical_or(overlay_mask[y1:y2, x1:x2], transformed_mask_cropped)
    # print("np.mean(overlay_mask[y1:y2, x1:x2]) = ", np.mean(overlay_mask[y1:y2, x1:x2]))
    # print("np.mean(overlay_mask) = ", np.mean(overlay_mask))

    return augmented_image, [x1, y1, x2, y2], overlay_mask


if __name__ == "__main__":
    # Example usage
    image = cv2.imread('/mnt/ssd1/data/mtl_garbage_sewage/changi-sample-images/img_1685327231.jpg')
    object_img = cv2.imread('/mnt/ssd1/data/mtl_garbage_sewage/changi-sample-images/img_1685505411.jpg')
    # mask = cv2.imread('path_to_object_mask.png', cv2.IMREAD_GRAYSCALE)
    print(object_img.shape)
    H, W = object_img.shape[0], object_img.shape[1]
    mask = np.zeros((H, W), dtype=np.uint8)
    mask[H//4:H//4*3, W//4:W//4*3] = 1
    print(mask.shape)

    # min - 16, max - 100
    affine_ratio = 0.1
    sqrt_area_min = 16 * math.sqrt(H*W) / math.sqrt(480*640)  # Minimum area of the bounding box
    sqrt_area_max = 100 * math.sqrt(H*W) / math.sqrt(480*640)  # Maximum area of the bounding box
    obj_alpha_min = 0.5

    augmented_image, bbox, overlay_mask = augment_object_detection(
        image, object_img, mask,
        sqrt_area_min, sqrt_area_max, affine_ratio, obj_alpha_min
    )

    # Save or display the augmented image and bounding box
    cv2.imwrite('augmented_image.jpg', augmented_image)
    print('Bounding Box:', bbox)

    plt.figure()
    plt.imshow(augmented_image)
    plt.show()