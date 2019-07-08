from __future__ import print_function
import sys
import os.path
import json
import rosbag
import h5py
import numpy as np
import progress.bar
import cv_bridge 

import cv2

def convert_to_h5(bag_filename):
    
    bag_directory, _ = os.path.split(os.path.abspath(bag_filename))
    bag = rosbag.Bag(bag_filename,'r')
    
    # Extract parameters
    param = None
    for topic, msg, t in bag.read_messages(topics=['/puzzleboxes_param']):
        param = json.loads(msg.data)
        break # only need 1
    if param is None:
        raise PuzzlebagConvertException('no /puzzleboxes_param message in file - trial too short??')

    # Extract background image
    bg_image_ros = None
    for topic, msg, t in bag.read_messages(topics=['/puzzleboxes_bg_image']):
        bg_image_ros = msg
        break # only need 1
    if bg_image_ros is None:
        raise PuzzlebagConvertException('no /puzzleboxes_bg_image message in file - trial too short??')
    bridge = cv_bridge.CvBridge()
    bg_image_cv = bridge.imgmsg_to_cv2(bg_image_ros)

    index_to_attr = {}
    index_to_data = {}
    for i, protocol in enumerate(param['regions']['protocols']):
        if protocol['fly']:
            # Create attributes for each region
            attr_dict = {'index': i} 
            for k,v in protocol.items():
                attr_dict[k] = v
            attr_dict['center'] = param['regions']['centers'][i]
            attr_dict['param'] = param
            index_to_attr[i] = attr_dict
    
            # Create datasets for each region (initially empty)
            data_keys = ['elapsed_t', 'led_enabled', 'object_found','x', 'y','classifier','led'] 
            data_dict = {}
            for k in data_keys:
                data_dict[k] = []
            index_to_data[i] = data_dict
        
    info = bag.get_type_and_topic_info()
    message_count = info.topics['/puzzleboxes_data'].message_count
    progress_bar = progress.bar.Bar('Processing', max=message_count)
    
    # Read messages and populate datasets
    for msg_count, msg_item in enumerate(bag.read_messages(topics=['/puzzleboxes_data'])):
        topic, msg, t = msg_item
        for i, region_data in enumerate(msg.region_data_list):
            if i in index_to_data:
                index_to_data[i]['elapsed_t'].append(msg.elapsed_time)
                index_to_data[i]['led_enabled'].append(msg.led_enabled)
                index_to_data[i]['object_found'].append(region_data.object_found)
                index_to_data[i]['x'].append(region_data.x)
                index_to_data[i]['y'].append(region_data.y)
                index_to_data[i]['classifier'].append(region_data.classifier)
                index_to_data[i]['led'].append(region_data.led)
        progress_bar.next()
    progress_bar.finish()
    
    # Save data to hdf5 file
    for i, attr_dict in index_to_attr.items():
        hdf5_filename = '{}_{}.hdf5'.format(param['datetime'],i)
        hdf5_filepath = os.path.join(bag_directory,hdf5_filename)
        hdf5_file = h5py.File(hdf5_filepath,'w')
        print('creating: {}'.format(hdf5_filename))
        # Add attributes
        for k, v in attr_dict.items():
            try:
                hdf5_file.attrs[k] = v
            except TypeError:
                hdf5_file.attrs[k] = json.dumps(v)
    
        # Add datasets
        for k, v in index_to_data[i].items():
            hdf5_file.create_dataset(k,data=np.array(v))

        # Add background image
        hdf5_file.create_dataset('bg_image', data=bg_image_cv)
        hdf5_file.close()


class PuzzlebagConvertException(Exception):
    pass


def main():
    file_list = sys.argv[1:]
    num_files = len(file_list)
    for i, bag_filename in enumerate(file_list):
        print('({}/{}) converting {}'.format(i+1, num_files, bag_filename))
        try:
            convert_to_h5(bag_filename)
        except Exception, e:
            print(' failed: {}'.format(str(e)))

# ---------------------------------------------------------------------------------------
if __name__  == '__main__':
    main()

