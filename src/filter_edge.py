import tqdm 
import numpy as np
import open3d as pn

def filter_edges(pcl, radius_shearch = 10,intensity = .5):
    pcl = pn.voxel_down_sample(pcl, radius_shearch/5)
    pcl_points = np.asarray(pcl.points)
    
    pcl_tree = pn.KDTreeFlann(pcl)
    results = []
    bar = tqdm.tqdm(total = len(pcl_points))
    for i in range(len(pcl.points)):
        [k, idx, _] = pcl_tree.search_radius_vector_3d(pcl.points[i],radius_shearch)
        nearest_points = pcl_points[idx]
        tmp = nearest_points - pcl_points[i]  
    
        scalar_product = tmp.dot(tmp.T)
        norm = np.sqrt(np.diag(scalar_product))
        tmp = np.array([tmp[i]/norm[i] for i in range(len(tmp))])[1:]
        scalar_product = abs(tmp.dot(tmp.T))
        if len(scalar_product) >0:
            results.append(scalar_product.mean())
        else:
             results.append(0)
        bar.update(1)
    bar.close()
    results = np.array(results)
    id_edge = np.argwhere(results < np.percentile(results, int(intensity*100)))[:,0]
    resutls = pn.PointCloud()
    resutls.points = pn.Vector3dVector(pcl_points[id_edge])
    
    return resutls
