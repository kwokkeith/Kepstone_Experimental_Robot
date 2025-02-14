#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Copyright (c) Megvii, Inc. and its affiliates.
import copy
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pycocotools.coco import COCO

from yolox.data.dataloading import get_yolox_datadir
from yolox.data.datasets.datasets_wrapper import CacheDataset, cache_read_img
from yolox.utils.boxes import cxcywh2xyxy


def remove_useless_info(coco):
    """
    Remove useless info in coco dataset. COCO object is modified inplace.
    This function is mainly used for saving memory (save about 30% mem).
    """
    if isinstance(coco, COCO):
        dataset = coco.dataset
        dataset.pop("info", None)
        dataset.pop("licenses", None)
        for img in dataset["images"]:
            img.pop("license", None)
            img.pop("coco_url", None)
            img.pop("date_captured", None)
            img.pop("flickr_url", None)
        if "annotations" in coco.dataset:
            for anno in coco.dataset["annotations"]:
                anno.pop("segmentation", None)


class COCODataset(CacheDataset):
    """
    COCO dataset class.
    """

    def __init__(
        self,
        data_dir=None,
        json_file="instances_train2017.json",
        name="train2017",
        img_size=(416, 416),
        preproc=None,
        cache=False,
        cache_type="ram",
    ):
        """
        COCO dataset initialization. Annotation data are read into memory by COCO API.
        Args:
            data_dir (str): dataset root directory
            json_file (str): COCO json file name
            name (str): COCO data name (e.g. 'train2017' or 'val2017')
            img_size (int): target image size after pre-processing
            preproc: data augmentation strategy
        """
        if data_dir is None:
            data_dir = os.path.join(get_yolox_datadir(), "COCO")
        self.data_dir = data_dir
        self.json_file = json_file

        self.coco = COCO(os.path.join(self.data_dir, "annotations", self.json_file))
        remove_useless_info(self.coco)
        self.ids = self.coco.getImgIds()
        self.num_imgs = len(self.ids)
        self.class_ids = sorted(self.coco.getCatIds())
        self.cats = self.coco.loadCats(self.coco.getCatIds())
        self._classes = tuple([c["name"] for c in self.cats])
        self.name = name
        self.img_size = img_size
        self.preproc = preproc
        self.annotations = self._load_coco_annotations()

        path_filename = [os.path.join(name, anno[3]) for anno in self.annotations]
        super().__init__(
            input_dimension=img_size,
            num_imgs=self.num_imgs,
            data_dir=data_dir,
            cache_dir_name=f"cache_{name}",
            path_filename=path_filename,
            cache=cache,
            cache_type=cache_type
        )

    def __len__(self):
        return self.num_imgs

    def _load_coco_annotations(self):
        return [self.load_anno_from_ids(_ids) for _ids in self.ids]

    def load_anno_from_ids(self, id_):
        im_ann = self.coco.loadImgs(id_)[0]
        width = im_ann["width"]
        height = im_ann["height"]
        anno_ids = self.coco.getAnnIds(imgIds=[int(id_)], iscrowd=False)
        annotations = self.coco.loadAnns(anno_ids)
        objs = []
        for obj in annotations:
            x1 = np.max((0, obj["bbox"][0]))
            y1 = np.max((0, obj["bbox"][1]))
            x2 = np.min((width, x1 + np.max((0, obj["bbox"][2]))))
            y2 = np.min((height, y1 + np.max((0, obj["bbox"][3]))))
            if obj["area"] > 0 and x2 >= x1 and y2 >= y1:
                obj["clean_bbox"] = [x1, y1, x2, y2]
                objs.append(obj)

        num_objs = len(objs)

        res = np.zeros((num_objs, 5))
        for ix, obj in enumerate(objs):
            cls = self.class_ids.index(obj["category_id"])
            res[ix, 0:4] = obj["clean_bbox"]
            res[ix, 4] = cls

        r = min(self.img_size[0] / height, self.img_size[1] / width)
        res[:, :4] *= r

        img_info = (height, width)
        resized_info = (int(height * r), int(width * r))

        file_name = (
            im_ann["file_name"]
            if "file_name" in im_ann
            else "{:012}".format(id_) + ".jpg"
        )

        return (res, img_info, resized_info, file_name)

    def load_anno(self, index):
        return self.annotations[index][0]

    def load_resized_img(self, index):
        img, img_file = self.load_image(index)
        r = min(self.img_size[0] / img.shape[0], self.img_size[1] / img.shape[1])
        resized_img = cv2.resize(
            img,
            (int(img.shape[1] * r), int(img.shape[0] * r)),
            interpolation=cv2.INTER_LINEAR,
        ).astype(np.uint8)
        return resized_img, img_file

    def load_image(self, index):
        file_name = self.annotations[index][3]

        img_file = os.path.join(self.data_dir, self.name, file_name)

        img = cv2.imread(img_file)
        assert img is not None, f"file named {img_file} not found"

        return img, img_file

    @cache_read_img(use_cache=True)
    def read_img(self, index):
        return self.load_resized_img(index)

    def pull_item(self, index):
        id_ = self.ids[index]
        label, origin_image_size, _, _ = self.annotations[index]
        img, img_file = self.read_img(index)

        return img, copy.deepcopy(label), origin_image_size, np.array([id_]), img_file

    @CacheDataset.mosaic_getitem
    def __getitem__(self, index):
        """
        One image / label pair for the given index is picked up and pre-processed.

        Args:
            index (int): data index

        Returns:
            img (numpy.ndarray): pre-processed image
            padded_labels (torch.Tensor): pre-processed label data.
                The shape is :math:`[max_labels, 5]`.
                each label consists of [class, xc, yc, w, h]:
                    class (float): class index.
                    xc, yc (float) : center of bbox whose values range from 0 to 1.
                    w, h (float) : size of bbox whose values range from 0 to 1.
            info_img : tuple of h, w.
                h, w (int): original shape of the image
            img_id (int): same as the input index. Used for evaluation.
        """
        img, target, img_info, img_id, img_file = self.pull_item(index)

        if self.preproc is not None:
            img, target, ground_mask = self.preproc(img, target, self.input_dim, img_file)
        return img, target, img_info, img_id


if __name__ == "__main__":
    from yolox.data.data_augment import TrainTransform

    trashnet_category_mapping = {
        "cardboard": 0,
        "glass": 0,
        "metal": 0,
        "paper": 0,
        "plastic": 0,
        "trash": 0
    }
    taco_category_mapping = {
        "Battery": 0,
        "Blist pack": 0,
        "Bottle": 0,
        # "Bottle cap": 0,
        "Can": 0,
        # "Cigarette": 0,
        "Cup": 0,
        "Glass jar": 0,
        "Lid": 0,
        "Paper": 0,
        "Paper bag": 0,
        "Plastic bag & wrapper": 0,
        "Plastic Container": 0,
    }

    preproc = TrainTransform(
        max_labels=50,
        flip_prob=0,
        hsv_prob=1.0,
        trashnet_overlay_prob=1,
        trashnet_info_path="/mnt/ssd1/data/trashnet/data/dataset-resized/info.json",
        trashnet_category_mapping=trashnet_category_mapping,
        taco_overlay_prob=1,
        taco_info_path="/mnt/ssd1/data/TACO/data/info.json",
        taco_category_mapping=taco_category_mapping,
        sqrt_area_min=64,
        sqrt_area_max=200,
        affine_ratio=0.3,
        erode_kernel=0.04,
        obj_alpha_min=1,
        overlay_times=1,
        ground_mask_positive_ratio_thres=0.35
    )
    dataset = COCODataset(
        data_dir="/mnt/ssd1/data/mtl_garbage_sewage",
        json_file="test.json",
        name="",
        img_size=(480, 640),
        preproc=preproc,
        cache=False,
        cache_type="ram"
    )

    for item in dataset:
        img, target, img_info, img_id = item
        # print(target)
        # continue

        plt.figure()
        img = np.ascontiguousarray(img.astype(np.uint8).transpose((1, 2, 0)))

        # if ground_mask is not None:
        #     print(np.max(ground_mask))
        #     ground_mask_expanded = np.expand_dims(ground_mask, 2)
        #     ground_mask_expanded_neg = 1 - ground_mask_expanded
        #     img_new = img * ground_mask_expanded_neg
        #
        #     color_mask = np.concatenate(
        #         (ground_mask_expanded * 255, ground_mask_expanded * 0, ground_mask_expanded * 0),
        #         axis=2
        #     )
        #     img_new = img_new + img * ground_mask_expanded * 0.5 + color_mask * 0.5
        #     img = img_new.astype(np.uint8)

        # draw bbox
        for i in range(target.shape[0]):
            if np.allclose(target[i, :], 0.0):
                break
            cxcywh = target[i:i+1, 1:]
            xyxy = cxcywh2xyxy(cxcywh)[0, :].astype(np.int32)
            cv2.rectangle(img,
                          (xyxy[0], xyxy[1]),
                          (xyxy[2], xyxy[3]),
                          (0, 0, 255),
                          2)

        plt.imshow(img)
        plt.show()
