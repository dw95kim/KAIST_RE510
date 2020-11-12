# set up Python environment: numpy for numerical routines, and matplotlib for plotting
import numpy as np

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

# Path for model, weight - Alexnet
model_def = 'prototxt/deploy_bn.prototxt'
model_weights = 'model/200520_model_bn_iter_170000.caffemodel'


# Path for mean
mu = np.load('GTSRB_Test/Final_Test/GTSRB_Test_imagenet_mean.npy')
# mu = np.load('KAIST_Test/test_data_mean.npy')

# Path for label, dataset
labels_file = 'GTSRB_Test/Final_Test/test.txt'
dataset_path = 'GTSRB_Test/Final_Test'

# Path for test label,
# labels_file = 'KAIST_Test/test.txt'
# dataset_path = 'KAIST_Test'

# set display defaults
plt.rcParams['figure.figsize'] = (10, 10)        # large images
plt.rcParams['image.interpolation'] = 'nearest'  # don't interpolate: show square pixels
plt.rcParams['image.cmap'] = 'gray'  # use grayscale output rather than a (potentially misleading) color heatmap

# caffe_root = '../'  # this file is expected to be in {caffe_root}/examples
# import sys
# sys.path.insert(0, caffe_root + 'python')

import caffe

# ================== ======================================= ===================
# ================== load net and set up input preprocessing ===================
# ================== ======================================= ===================

caffe.set_mode_gpu()

net = caffe.Net(model_def,         # defines the structure of the model
                model_weights,     # contains the trained weights
                caffe.TEST)        # use test mode (e.g. don't perform dropout)

mu = mu.mean(1).mean(1)  # average over pixels to obtain the mean (BGR) pixel values

# create transformer for the input called 'data'
transformer = caffe.io.Transformer({'data' : net.blobs['data'].data.shape})

transformer.set_transpose('data', (2,0,1))  # move image channels to outermost dimension
transformer.set_mean('data', mu)            # subtract the dataset-mean value in each channel
transformer.set_raw_scale('data', 255)      # rescale from [0, 1] to [0, 255]
transformer.set_channel_swap('data', (2,1,0))  # swap channels from RGB to BGR

# Alexnet
net.blobs['data'].reshape(1,        # batch size
                          3,         # 3-channel (BGR) images
                          200, 200)  # image size is 227x227


labels = np.loadtxt(labels_file, str, delimiter='\t')

test_count = np.zeros(43)
test_total = np.zeros(43)

# for data_path in data_lists:
for i in range(labels.size):
    # glab image
    test_data_path = dataset_path + "/" + labels[i][:18] # test data
    # test_data_path = dataset_path + "/" + labels[i][:28] # kaist data
    ground_truth = int(labels[i][-2:])

    image = caffe.io.load_image(test_data_path)
    transformed_image = transformer.preprocess('data', image)
    net.blobs['data'].data[...] = transformed_image

    # perform classification
    output = net.forward()
    output_prob = output['prob'][0] # the output probability vector for the first image in the batch

    pred_class = output_prob.argmax()
    
    # Check correction
    check = (ground_truth == pred_class)
    if check == True:
        test_count[ground_truth] += 1
        print("test_data_path :", test_data_path)
    test_total[ground_truth] += 1
    print("%i th iter. output is %i, label is %i" %(i, pred_class, ground_truth))


test_accuracies = test_count/test_total
test_accuracies[np.isnan(test_accuracies)] = 0

test_class = np.array(range(43))
print("test_count : ", test_count)
print("test_total : ", test_total)
print("test test_total :", test_accuracies)
print("average accuracy :", np.average(test_accuracies))
plt.bar(test_class, test_accuracies)
plt.savefig("200521_GTSRB_accuracy.png")
#plt.show()
