# introduction

This repo focus on cloud segmentation.

### assets

Cloud dataset.

### euclidean_cluster

Cluster by euclidean distance between points and merge points close to each other into one cluster.

### linefit_ground_segmentation

[原论文](https://github.com/blue-stone-j/papers/blob/main/2010%20Fast%20Segmentation%20of%203D%20Point%20Clouds%20for%20Ground%20Vehicles.pdf)为《Fast Segmentation of 3D Point Clouds for Ground Vehicles》(2010)
原作者的开源代码链接：https://github.com/lorenwel/linefit_ground_segmentation

该代码来自别人已经注释过的代码。由于无法确定来源，因此暂无法给出原代码链接。

程序中存在bug以及可以改进的地方。我没有对程序作任何改动，关于bug和改进的建议在注释中写出。我的建议可能错误甚至引入新的bug。因此，请谨慎使用本程序和采纳建议。如果可以，请反馈bug和建议，我会添加到注释中。

### patchworkpp-noted

经过注释的 patchwork++ 的代码。[论文](https://github.com/blue-stone-j/papers/blob/main/2022%20Patchwork%2B%2B%20Fast%20and%20Robust%20Ground%20Segmentation%20Solving%20Partial%20Under-Segmentation%20Using%203D%20Point%20Cloud.pdf)为《2022 Patchwork++ Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud》。

这篇论文提出了一种提取地面的方法。

### ray_ground_filter

autoware-points_preprocessor-1.4

### ring_ground_filter

autoware-points_preprocessor-1.4

*Problem:* Some parts of vertical objects are detected as ground points.

*FIX:* One workaround for this is to set the "clipping_threshold" parameter in the launch file.
	  By setting this threshold, any points higher than this threshold will be detected as vertical points.
	  However, because all points that are higher than the threshold will be detected as vertical points, slopes might be incorrectly detected as vertical points as well.

---

*Problem:* Some small objects like curbs are missing.
*FIX:* Try to lower the "gap_thres" parameter.
	  However, lowering this parameter too much might result in mis-detecting slopes as vertical points.
	  Usually, 0.15 - 0.2 is enough for detecting curbs.

---

*Problem:* Ring shaped noise (ground points being detected as vertical points) occurs nearby the vehicle.
*FIX:* Try to lower the "points_distance" parameter.
	  However, lowering this parameter too much might result in mis-detecting vertical objects which are far away from the vehicle as ground points.

---

*Problem:* Line shaped noise (in radial direction) occurs near edges of vertical objects.
*FIX:* Decrease the "min_points" parameter. However, by doing so, some parts of vertical objects will be mis-detected as ground points.

### patchworkpp

Divide cloud to by radial direction and tangential direction. And then judge whether this segment belongs to ground.