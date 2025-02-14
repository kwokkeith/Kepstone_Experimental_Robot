#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Copyright (c) Megvii, Inc. and its affiliates.
"""
Data augmentation functionality. Passed as callable transformations to
Dataset classes.

The data augmentation procedures were interpreted from @weiliu89's SSD paper
http://arxiv.org/abs/1512.02325
"""
import os
import time

import math
import random
import json
import cv2
import numpy as np

from yolox.utils import xyxy2cxcywh
from yolox.data.datasets.overlay_augmentation import augment_object_detection, is_overlapping


def augment_hsv(img, hgain=5, sgain=30, vgain=30):
    hsv_augs = np.random.uniform(-1, 1, 3) * [hgain, sgain, vgain]  # random gains
    hsv_augs *= np.random.randint(0, 2, 3)  # random selection of h, s, v
    hsv_augs = hsv_augs.astype(np.int16)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)

    img_hsv[..., 0] = (img_hsv[..., 0] + hsv_augs[0]) % 180
    img_hsv[..., 1] = np.clip(img_hsv[..., 1] + hsv_augs[1], 0, 255)
    img_hsv[..., 2] = np.clip(img_hsv[..., 2] + hsv_augs[2], 0, 255)

    cv2.cvtColor(img_hsv.astype(img.dtype), cv2.COLOR_HSV2BGR, dst=img)  # no return needed


def get_aug_params(value, center=0):
    if isinstance(value, float):
        return random.uniform(center - value, center + value)
    elif len(value) == 2:
        return random.uniform(value[0], value[1])
    else:
        raise ValueError(
            "Affine params should be either a sequence containing two values\
             or single float values. Got {}".format(value)
        )


def get_affine_matrix(
    target_size,
    degrees=10,
    translate=0.1,
    scales=0.1,
    shear=10,
):
    twidth, theight = target_size

    # Rotation and Scale
    angle = get_aug_params(degrees)
    scale = get_aug_params(scales, center=1.0)

    if scale <= 0.0:
        raise ValueError("Argument scale should be positive")

    R = cv2.getRotationMatrix2D(angle=angle, center=(0, 0), scale=scale)

    M = np.ones([2, 3])
    # Shear
    shear_x = math.tan(get_aug_params(shear) * math.pi / 180)
    shear_y = math.tan(get_aug_params(shear) * math.pi / 180)

    M[0] = R[0] + shear_y * R[1]
    M[1] = R[1] + shear_x * R[0]

    # Translation
    translation_x = get_aug_params(translate) * twidth  # x translation (pixels)
    translation_y = get_aug_params(translate) * theight  # y translation (pixels)

    M[0, 2] = translation_x
    M[1, 2] = translation_y

    return M, scale


def apply_affine_to_bboxes(targets, target_size, M, scale):
    num_gts = len(targets)

    # warp corner points
    twidth, theight = target_size
    corner_points = np.ones((4 * num_gts, 3))
    corner_points[:, :2] = targets[:, [0, 1, 2, 3, 0, 3, 2, 1]].reshape(
        4 * num_gts, 2
    )  # x1y1, x2y2, x1y2, x2y1
    corner_points = corner_points @ M.T  # apply affine transform
    corner_points = corner_points.reshape(num_gts, 8)

    # create new boxes
    corner_xs = corner_points[:, 0::2]
    corner_ys = corner_points[:, 1::2]
    new_bboxes = (
        np.concatenate(
            (corner_xs.min(1), corner_ys.min(1), corner_xs.max(1), corner_ys.max(1))
        )
        .reshape(4, num_gts)
        .T
    )

    # clip boxes
    new_bboxes[:, 0::2] = new_bboxes[:, 0::2].clip(0, twidth)
    new_bboxes[:, 1::2] = new_bboxes[:, 1::2].clip(0, theight)

    targets[:, :4] = new_bboxes

    return targets


def random_affine(
    img,
    targets=(),
    target_size=(640, 640),
    degrees=10,
    translate=0.1,
    scales=0.1,
    shear=10,
):
    M, scale = get_affine_matrix(target_size, degrees, translate, scales, shear)

    img = cv2.warpAffine(img, M, dsize=target_size, borderValue=(114, 114, 114))

    # Transform label coordinates
    if len(targets) > 0:
        targets = apply_affine_to_bboxes(targets, target_size, M, scale)

    return img, targets


def _mirror(image, boxes, prob=0.5):
    _, width, _ = image.shape
    if random.random() < prob:
        image = image[:, ::-1]
        boxes[:, 0::2] = width - boxes[:, 2::-2]
    return image, boxes


def preproc(img, input_size, swap=(2, 0, 1)):
    if len(img.shape) == 3:
        padded_img = np.ones((input_size[0], input_size[1], 3), dtype=np.uint8) * 114
    else:
        padded_img = np.ones(input_size, dtype=np.uint8) * 114

    r = min(input_size[0] / img.shape[0], input_size[1] / img.shape[1])
    resized_img = cv2.resize(
        img,
        (int(img.shape[1] * r), int(img.shape[0] * r)),
        interpolation=cv2.INTER_LINEAR,
    ).astype(np.uint8)
    padded_img[: int(img.shape[0] * r), : int(img.shape[1] * r)] = resized_img

    padded_img = padded_img.transpose(swap)
    padded_img = np.ascontiguousarray(padded_img, dtype=np.float32)
    return padded_img, r


def mask_erosion(mask, kernel_size=5):
    if kernel_size <= 1:
        return mask
    # Creating kernel
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    # Using cv2.erode() method
    mask = cv2.erode(mask, kernel)
    return mask


def augment_overlay(image, targets, overlay_info,
                    category_mapping, sqrt_area_min, sqrt_area_max, affine_ratio,
                    obj_alpha_min,
                    ground_mask):
    # image: opencv style bgr image, HxWx3
    # targets: Nx5 numpy array, x1 y1 x2 y2 label_id
    if ground_mask is None:
        return False, image, targets

    counter = 0
    max_retry = 5
    while True:
        overlay_item = random.choice(overlay_info)
        obj_img = cv2.imread(overlay_item[0])
        obj_mask = cv2.imread(overlay_item[1], cv2.IMREAD_UNCHANGED)

        # print(obj_mask.shape)
        obj_category = overlay_item[2]
        if obj_category in category_mapping.keys():
            obj_label_id = category_mapping[obj_category]
        else:
            return False, image, targets

        aug_image, bbox, overlay_mask = augment_object_detection(
            image, obj_img, obj_mask,
            sqrt_area_min, sqrt_area_max, affine_ratio,
            obj_alpha_min
        )
        if aug_image is None:
            return False, image, targets
        bbox = np.asarray([[bbox[0], bbox[1], bbox[2], bbox[3], obj_label_id]], dtype=np.float32)

        is_aug_done = True
        # check if the overlay obj overlaps with existing bbox label
        for i in range(targets.shape[0]):
            if is_overlapping(targets[i, 0:4], bbox[0, 0:4]):
                is_aug_done = False
                break
        if ground_mask is not None and is_aug_done:
            # check if the overlay obj is within the ground mask
            ground_obj_overlap_mask = np.logical_and(ground_mask.astype(np.bool_), overlay_mask)
            ground_obj_overlap_ratio = np.sum(ground_obj_overlap_mask) / (np.sum(overlay_mask) + 0.0001)
            # print("ground_obj_overlap_ratio = ", ground_obj_overlap_ratio)
            if ground_obj_overlap_ratio < 0.9:
                is_aug_done = False
        if is_aug_done:
            aug_targets = np.concatenate((targets, bbox), axis=0)
            break
        else:
            counter += 1
            if counter > max_retry:
                # augmentation failed
                aug_image = image
                aug_targets = targets
                break
            else:
                continue

    return is_aug_done, aug_image, aug_targets


class TrainTransform:
    def __init__(self, max_labels=50, flip_prob=0.5, hsv_prob=1.0,
                 trashnet_overlay_prob=0, trashnet_info_path=None, trashnet_category_mapping=None,
                 taco_overlay_prob=0, taco_info_path=None, taco_category_mapping=None,
                 sqrt_area_min=9, sqrt_area_max=64, affine_ratio=0.2,
                 obj_alpha_min=0.5, overlay_times=1, ground_mask_positive_ratio_thres=0.4):
        self.max_labels = max_labels
        self.flip_prob = flip_prob
        self.hsv_prob = hsv_prob

        self.trashnet_overlay_prob = trashnet_overlay_prob
        self.trashnet_category_mapping = trashnet_category_mapping

        self.taco_overlay_prob = taco_overlay_prob
        self.taco_category_mapping = taco_category_mapping

        self.sqrt_area_min = sqrt_area_min
        self.sqrt_area_max = sqrt_area_max
        self.affine_ratio = affine_ratio
        self.obj_alpha_min = obj_alpha_min
        self.overlay_times = overlay_times
        self.ground_mask_positive_ratio_thres = ground_mask_positive_ratio_thres

        if self.trashnet_overlay_prob > 0.0001:
            # load object and masked info
            # /mnt/ssd1/data/trashnet/data/dataset-resized/info.json
            with open(trashnet_info_path, "r") as f:
                self.trashnet_overlay_info = json.load(f)
                self.trashnet_overlay_info = [x for x in self.trashnet_overlay_info if x[2] in self.trashnet_category_mapping.keys()]
                print(self.trashnet_category_mapping)
                print("self.trashnet_overlay_info = ", len(self.trashnet_overlay_info))

        if self.taco_overlay_prob > 0.0001:
            # load object and masked info
            # /mnt/ssd1/data/TACO/data/info.json
            with open(taco_info_path, "r") as f:
                self.taco_overlay_info = json.load(f)
                self.taco_overlay_info = [x for x in self.taco_overlay_info if x[2] in self.taco_category_mapping.keys()]
                print(self.taco_category_mapping)
                print("self.taco_overlay_info = ", len(self.taco_overlay_info))

    def __call__(self, image, targets, input_dim, img_file):
        ground_mask = None
        ground_mask_positive_ratio = None
        if isinstance(img_file, str):
            img_name, _ = os.path.splitext(img_file)
            ground_mask_path = img_name + "_samMask.png"
            if (self.trashnet_overlay_prob + self.taco_overlay_prob > 0) and os.path.exists(ground_mask_path):
                ground_mask = cv2.imread(ground_mask_path, cv2.IMREAD_UNCHANGED)

        for i in range(self.overlay_times):
            if random.random() < self.trashnet_overlay_prob:
                if (ground_mask is not None) and (ground_mask_positive_ratio is None):
                    ground_mask_positive_ratio = np.sum(ground_mask) / (ground_mask.shape[0] * ground_mask.shape[1])
                if (ground_mask is not None) and (ground_mask_positive_ratio < self.ground_mask_positive_ratio_thres):
                    ground_mask = None

                is_aug_done, image, targets = augment_overlay(
                    image, targets, self.trashnet_overlay_info, self.trashnet_category_mapping,
                    self.sqrt_area_min, self.sqrt_area_max, self.affine_ratio,
                    self.obj_alpha_min,
                    ground_mask
                )
                # print("trashnet: ", is_aug_done)

        for i in range(self.overlay_times):
            if random.random() < self.taco_overlay_prob:
                if (ground_mask is not None) and (ground_mask_positive_ratio is None):
                    ground_mask_positive_ratio = np.sum(ground_mask) / (ground_mask.shape[0] * ground_mask.shape[1])
                if (ground_mask is not None) and (ground_mask_positive_ratio < self.ground_mask_positive_ratio_thres):
                    ground_mask = None

                is_aug_done, image, targets = augment_overlay(
                    image, targets, self.taco_overlay_info, self.taco_category_mapping,
                    self.sqrt_area_min, self.sqrt_area_max, self.affine_ratio,
                    self.obj_alpha_min,
                    ground_mask
                )
                # print("taco: ", is_aug_done)

        boxes = targets[:, :4].copy()
        labels = targets[:, 4].copy()
        if len(boxes) == 0:
            targets = np.zeros((self.max_labels, 5), dtype=np.float32)
            image, r_o = preproc(image, input_dim)
            return image, targets, ground_mask

        image_o = image.copy()
        targets_o = targets.copy()
        height_o, width_o, _ = image_o.shape
        boxes_o = targets_o[:, :4]
        labels_o = targets_o[:, 4]
        # bbox_o: [xyxy] to [c_x,c_y,w,h]
        boxes_o = xyxy2cxcywh(boxes_o)

        if random.random() < self.hsv_prob:
            augment_hsv(image)
        image_t, boxes = _mirror(image, boxes, self.flip_prob)
        height, width, _ = image_t.shape
        image_t, r_ = preproc(image_t, input_dim)
        # boxes [xyxy] 2 [cx,cy,w,h]
        boxes = xyxy2cxcywh(boxes)
        boxes *= r_

        mask_b = np.minimum(boxes[:, 2], boxes[:, 3]) > 1
        boxes_t = boxes[mask_b]
        labels_t = labels[mask_b]

        if len(boxes_t) == 0:
            image_t, r_o = preproc(image_o, input_dim)
            boxes_o *= r_o
            boxes_t = boxes_o
            labels_t = labels_o

        labels_t = np.expand_dims(labels_t, 1)

        targets_t = np.hstack((labels_t, boxes_t))
        padded_labels = np.zeros((self.max_labels, 5))
        padded_labels[range(len(targets_t))[: self.max_labels]] = targets_t[
            : self.max_labels
        ]
        padded_labels = np.ascontiguousarray(padded_labels, dtype=np.float32)
        return image_t, padded_labels, ground_mask


class ValTransform:
    """
    Defines the transformations that should be applied to test PIL image
    for input into the network

    dimension -> tensorize -> color adj

    Arguments:
        resize (int): input dimension to SSD
        rgb_means ((int,int,int)): average RGB of the dataset
            (104,117,123)
        swap ((int,int,int)): final order of channels

    Returns:
        transform (transform) : callable transform to be applied to test/val
        data
    """

    def __init__(self, swap=(2, 0, 1), legacy=False):
        self.swap = swap
        self.legacy = legacy

    # assume input is cv2 img for now
    def __call__(self, img, res, input_size, img_file):
        img, _ = preproc(img, input_size, self.swap)
        if self.legacy:
            img = img[::-1, :, :].copy()
            img /= 255.0
            img -= np.array([0.485, 0.456, 0.406]).reshape(3, 1, 1)
            img /= np.array([0.229, 0.224, 0.225]).reshape(3, 1, 1)
        return img, np.zeros((1, 5)), None
