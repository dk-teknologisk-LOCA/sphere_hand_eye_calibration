from utils import fit_sphere

pc_data_dic = {'OG': ['data/direct_data/pointcloud/05062023/kinect_20230605_122349_points2_1.npy', -0.1, 0.5, 0.0, -1.0, 2.0, 0.0],  # noqa: E501
            'painted': ['data/direct_data/pointcloud/23062023/kinect_20230623_131252_points2_1.npy', -0.3, 0.3, 0.170, -0.2, 1.5, 0.95],  # noqa: E501
            'painted creaform': ['data/direct_data/pointcloud/23062023/kinect_20230623_150100_points2_2.npy', -0.22, 0.1, 0.2, 0.0, 1.0, 0.4],  # noqa: E501
            'bad angle painted creaform': ['data/direct_data/pointcloud/26062023/kinect_20230626_112102_points2_1.npy', -0.1, 0.18, 0.0, -0.2, 1.05, 0.75],  # noqa: E501
            'centered at 100 0 5': ['data/direct_data/pointcloud/26062023/kinect_20230626_130250_points2_2.npy', -0.2, 0.1, 0.15, -0.1, 0.7, 0.5], # noqa: E501
            'al robot': ['data/direct_data/pointcloud/26062023/kinect_20230626_162521_points2_3.npy', -0.2, 0.1, 0.15, -0.1, 0.7 , 0.5],  # noqa: E501
            'robot_test_1': ['data/direct_data/pointcloud/29062023/kinect_20230629_164559_points2_1.npy', -0.2, 0.2, 0.1, -0.1, 400+75+11.58], # noqa: E501
            'robot_test_2': ['data/direct_data/pointcloud/29062023/kinect_20230629_164600_points2_2.npy', -0.2, 0.2, 0.1, -0.1, 500+75+11.58]} # noqa: E501

imgs_data_dic = {'OG': ['data/direct_data/imgs/05062023/kinect_20230605_122347_color_1.jpg', 'data/direct_data/imgs/05062023/kinect_20230605_122347_depth_1.npy', 'data/direct_data/imgs/05062023/kinect_20230605_122347_ir_1.npy'], # noqa: E501
            'painted': ['data/direct_data/imgs/23062023/kinect_20230623_131252_color_1.jpg', 'data/direct_data/imgs/23062023/kinect_20230623_131252_depth_1.npy', 'data/direct_data/imgs/23062023/kinect_20230623_131252_ir_1.npy'], # noqa: E501
            'painted creaform': ['data/direct_data/imgs/23062023/kinect_20230623_150101_color_2.jpg', 'data/direct_data/imgs/23062023/kinect_20230623_150101_depth_2.npy', 'data/direct_data/imgs/23062023/kinect_20230623_150101_ir_2.npy'], # noqa: E501
            'bad angle painted creaform': ['data/direct_data/imgs/26062023/kinect_20230626_112102_color_1.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_112102_depth_1.npy', 'data/direct_data/imgs/26062023/kinect_20230626_112102_ir_1.npy'], # noqa: E501
            'centered at 100 0 5': ['data/direct_data/imgs/26062023/kinect_20230626_130249_color_2.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_130249_depth_2.npy', 'data/direct_data/imgs/26062023/kinect_20230626_130249_ir_2.npy'], # noqa: E501
            'al robot': ['data/direct_data/imgs/26062023/kinect_20230626_162520_color_3.jpg', 'data/direct_data/imgs/26062023/kinect_20230626_162520_depth_3.npy', 'data/direct_data/imgs/26062023/kinect_20230626_162520_ir_3.npy']} # noqa: E501

dic_name = "robot_test_1"
center = fit_sphere.find_center(pc_data_dic, dic_name, plot_on = 0)
print("Error on Center: ", center[-1]*1000 - pc_data_dic[dic_name][-1])

dic_name = "robot_test_2"
center = fit_sphere.find_center(pc_data_dic, dic_name, plot_on = 0)
print("Error on Center: ", center[-1]*1000 - pc_data_dic[dic_name][-1])