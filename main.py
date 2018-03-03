import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    # load the prebuilt model
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    # get the desired tensors from the graph
    graph = tf.get_default_graph()
    input_tensor = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7_out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)
    
    return input_tensor, keep_prob, layer3_out, layer4_out, layer7_out
tests.test_load_vgg(load_vgg, tf)

def conv1x1_layer(input, num_classes):
    return tf.layers.conv2d(input, num_classes, (1, 1), strides=(1,1), padding='same',
                            kernel_initializer= tf.random_normal_initializer(stddev=0.01), 
                            kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3))

def upsample_layer(input, num_classes, kernel, strides):
    return tf.layers.conv2d_transpose(input, num_classes, kernel, strides=strides, padding='same',
                            kernel_initializer= tf.random_normal_initializer(stddev=0.01), 
                            kernel_regularizer= tf.contrib.layers.l2_regularizer(1e-3))

def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """

    # scale the output of layers 3&4 in the skip connections as suggested by the authors of the FCN-8 paper
    vgg3_scaled = tf.multiply(vgg_layer3_out, 0.0001)
    vgg4_scaled = tf.multiply(vgg_layer4_out, 0.01)

    # reduce the output of the pretrained VGG network to the number of classes we want
    vgg3_nc = conv1x1_layer(vgg3_scaled, num_classes)
    vgg4_nc = conv1x1_layer(vgg4_scaled, num_classes)
    vgg7_nc = conv1x1_layer(vgg_layer7_out, num_classes)

    # upsample output and add skip connection with output of layer 4
    output = upsample_layer(vgg7_nc, num_classes, (4, 4), strides=(2, 2))
    output = tf.add(output, vgg4_nc)

    # upsample output and add skip connection with output of layer 3
    output = upsample_layer(output, num_classes, (4, 4), strides=(2, 2))
    output = tf.add(output, vgg3_nc)

    # upsample to final size
    output = upsample_layer(output, num_classes, 16, strides=(8, 8))

    return output

tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # reshape logits to a 2D tensor: row = pixels, column = classes
    logits_2d = tf.reshape(nn_last_layer, [-1, num_classes])
    labels_2d = tf.reshape(correct_label, [-1, num_classes])

    # define loss function (cross entropy and regularization)
    ce_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits_2d, labels=labels_2d))
    reg_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES)
    loss = ce_loss + 0.1 * sum(reg_losses)

    # create the optimizer
    optimizer = tf.train.AdamOptimizer(learning_rate).minimize(loss)

    return logits_2d, optimizer, loss

tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # initialize variables
    sess.run(tf.global_variables_initializer())

    # run epochs
    for epoch in range(epochs):
        print("Epoch {:3d} ".format(epoch), end='')
        epoch_loss = 0
        for batch_x, batch_y in get_batches_fn(batch_size):
            _, loss = sess.run([train_op, cross_entropy_loss], feed_dict={
                                    input_image: batch_x, 
                                    correct_label: batch_y, 
                                    keep_prob: 0.80,
                                    learning_rate: 0.0005})
            epoch_loss += loss
            print(".", end='', flush=True)
                               
        print(" loss = {:.3f}".format(epoch_loss))

tests.test_train_nn(train_nn)

def run():
    num_classes = 2
    epochs = 50
    batch_size = 5
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # Build NN using load_vgg, layers, and optimize function
        input_tensor, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)

        last_layer = layers(layer3_out, layer4_out, layer7_out, num_classes)

        correct_label = tf.placeholder(tf.float32, [None, None, None, num_classes], name='correct_label')
        learning_rate = tf.placeholder(tf.float32, name='learning_rate')

        logits, optimizer, loss = optimize(last_layer, correct_label, learning_rate, num_classes)

        # Train NN using the train_nn function
        train_nn(sess, epochs, batch_size, get_batches_fn, optimizer, loss, input_tensor, correct_label, keep_prob, learning_rate)

        # Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_tensor)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
