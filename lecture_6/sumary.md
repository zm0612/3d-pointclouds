## 1.基于图像的目标检测总结

### 1.1 什么是目标检测

一张图像其中会有很多的目标，所以在执行目标检测时，需要对目标进行定位，然后找出可能存在目标的区域，然后对这个区域执行目标分类。

### 1.2 基础知识

- IoU：box的交集比上并集；
- TP：True Positives
- FP：False Positives
- Precision：TP / (TP + FP)
- Recall:  TP / (TP + FN)
- AP: Average Precision
- Non-Maximum Suppression(NMS，非极大值抑制)：目的是从很多box中选择score和confidence最大的那一个。

### 1.3 Two Stage 

Detection = Localization + Classification

__(1). RCNN__

通过对图像进行颜色聚类操作，预先得到很多的region proposals，然后对每一个region proposal执行卷积操作，对每一个卷积结果执行分类和回归。

> 该方法会对每一个区域训练一个卷积网络，没有达到共享网络的目的，速度很慢！

__(2). Fast RCNN__

首先对图像执行卷积操作，然后对卷积之后的特征图执行region proposal操作，region proposal的提取依然采用的是传统的方法(例如颜色聚类)，然后将每一个region proposal进行ROI Pooling得到大小一样的特征，最后执行CNN和MLP，然后对结果进行回归和分类。

__(3). Faster RCNN__

该检测算法提出了Region Proposal Network，通过网络的方法自动提取region proposal。它也是一样，先执行一步卷积操作，然后在特征图上执行region proposal network，在特征图上每一个地方都会给一个anchor，以这个anchor为中心给出多个尺度的box，对每一个box执行物体判断，如果该box是一个物体，然后对box执行回归和分类，回归是为了获得更好的box尺寸。

__(4). Mask RCNN__

Mask RCNN的region proposal方法与Fast RCNN是一样的，但是其在ROI Pooling时引入了一种新的更加科学的插值方法，并且它在分类网络中还增加了实力分割

### 1.4 One Stage Detector (No ROI Pooling) 

首先对图像执行一次CNN，然后特征图的每一个grid给一个anchor，然后直接执行检测，判断这个是什么物体，与RCNN的区别是不会再执行RPN去检测是不是物体，而是直接就输出是什么物体。

__(1) SSD__

对图像执行不同的卷积，获得多个特征图，然后在每一个特征图上执行one stage的检测，一共会得到8732个检测结果，然后从这些结果中找到正确的检测的结果。

> 一般情况下Two-Stage比One-Stage效果更好，但是速度会慢！

## 3.基于三维点云的目标检测总结

如果能正确的理解图像上的目标检测概念，理解三维点云基本上就没有问题，它们之间的原理十分类似，三维点云的总结可以直接看ppt！









