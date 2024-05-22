# introudction

This repo focus on cloud segmentation.

### asses
Cloud dataset.

### linefit_ground_segmentation
##### 1.

[原论文](https://github.com/blue-stone-j/papers/blob/main/2010%20Fast%20Segmentation%20of%203D%20Point%20Clouds%20for%20Ground%20Vehicles.pdf)为《Fast Segmentation of 3D Point Clouds for Ground Vehicles》(2010)
原作者的开源代码链接：https://github.com/lorenwel/linefit_ground_segmentation

该代码来自别人已经注释过的代码。由于无法确定来源，因此暂无法给出原代码链接。

##### 3.
程序中存在bug以及可以改进的地方。我没有对程序作任何改动，关于bug和改进的建议在注释中写出。我的建议可能错误甚至引入新的bug。因此，请谨慎使用本程序和采纳建议。如果可以，请反馈bug和建议，我会添加到注释中。

### patchworkpp
Divide cloud to by radial direction and tangential direction. And then judge whether this segment belongs to ground.

### euclidean_cluster
Cluster by euclidean distance between points and merge points close to each other into one cluster.