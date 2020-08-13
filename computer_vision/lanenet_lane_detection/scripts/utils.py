import numpy as np
from sklearn.cluster import MeanShift, KMeans

def embedding_post_processing(embedding, bin_seg, band_width = 1.5, max_num_lane = 5):
    
    cluster_result = np.zeros(bin_seg.shape, dtype = np.int32)
    cluster_list = embedding[bin_seg>0]

    if len(cluster_list) == 0:
        return cluster_result

    mean_shift = MeanShift(bandwidth= 1.5, bin_seeding= True, n_jobs= -1)
    #k_mean = KMeans(n_clusters= 5, random_state=0)

    mean_shift.fit(cluster_list)

    labels = mean_shift.labels_
    cluster_result[bin_seg>0] = labels + 1

    cluster_result[cluster_result > max_num_lane] = 0
    for idx in np.unique(cluster_result):
        if len(cluster_result[cluster_result==idx]) < 15:
            cluster_result[cluster_result==idx] = 0

    return cluster_result